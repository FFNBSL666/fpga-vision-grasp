// 抓放主控：接收有效图像、运行板端模型、锁定两点，并执行保存的动作。
// 阅读顺序：配置检查 → 动作目标与串口 → 相机/模型 → 选点 → 主状态机。
// stage 表示动作阶段，state 表示该阶段内发送、等待或查表等步骤。

`timescale 1ns / 1ps
// 使用 FPGA 接收并识别的新图像结果，选择抓取点和放置点。
// 执行保存的完整动作序列，再等待下一轮两次选点。
// 姿态表保留当前固定高度和初始观察姿态。
module confirmed_grasp_core #(
    parameter integer CLK_HZ = 50000000, CAMERA_BAUD = 921600, SERVO_BAUD = 115200,
    parameter integer MIN_MOVE_MS = 2000, MOVE_SETTLE_MS = 200, OBS_SETTLE_MS = 500, PLACE_SETTLE_MS = 500,
    parameter integer CONTINUOUS = 1, MAPPED_MODE = 0, PLACE_MODE = 0, BASE_DIRECT_COMMAND = 0,
    parameter integer PATH_MOVE_MS = 20, PATH_SETTLE_MS = 10
)(
    input wire clk, rst_n, camera_rx, enable, calibration_valid, request_toggle, continue_toggle, estop, capture_only,
    input wire [15:0] move_ms,
    input wire [95:0] observation_pose, approach_pose, descent_pose, grip_pose, lift_pose,
    input wire [7:0] calibrated_pixel_x, calibrated_pixel_y, calibrated_radius,
    output wire servo_tx, busy, pregrasp_ready,
    output reg fault,
    output reg [7:0] error_code,
    output reg [3:0] state,
    output reg [31:0] accepted_cycles, sent_frames, completed_cycles,
    output reg cycle_done,
    output wire frame_good, frame_bad, model_busy, model_result_valid, observe_ready,
    output wire [49:0] detections,
    output reg [15:0] target_pixel,
    output reg [15:0] placement_pixel,
    output reg [15:0] base_command_p,
    output reg [31:0] base_command_count,
    output wire [15:0] source_base_p, destination_base_p,
    output wire [2:0] active_joint,
    output reg [31:0] actuator_command_count,
    output wire [1:0] selection_phase,
    output reg [7:0] result_frame_id,
    output wire head_valid, head_last,
    output wire [8:0] head_index,
    output wire signed [31:0] head_acc
);

    // 主状态编号保持原值；stage 与 state 分开描述动作目标和执行步骤。
    localparam [3:0] IDLE = 0, ISSUE = 1, WAIT_TX = 2, WAIT_MOVE = 3, OBSERVE = 4, HOLD = 5, PAUSE = 6;
    localparam [3:0] ISSUE_BASE = 7, WAIT_BASE_TX = 8, WAIT_BASE_MOVE = 9, FINISH_STAGE = 10;
    localparam [3:0] WAIT_PLACE_SETTLE = 11;
    localparam [3:0] LOAD_PATH = 12;
    localparam [3:0] FIRST_ISSUE = BASE_DIRECT_COMMAND != 0 ? ISSUE_BASE : ISSUE;
    localparam integer MS_DIV = CLK_HZ / 1000;
    reg seen_toggle, seen_continue;
    reg [3:0] stage;
    reg [95:0] saved_observe, saved_approach, saved_descent, saved_grip, saved_lift;
    reg [95:0] saved_place_approach, saved_place_descent, saved_release, saved_retreat;
    reg [15:0] saved_duration, saved_wait, elapsed_ms;
    reg [7:0] saved_x, saved_y, saved_radius;
    reg [31:0] ms_counter;
    reg base_command_known;
    reg [2:0] upper_joint;
    reg [5:1] upper_known;
    reg [95:0] last_upper_pose;
    reg placement_lowered;
    reg [6:0] path_height;
    reg [2:0] path_load_cycles, path_shift;
    reg [4:0] path_substep;
    reg [47:0] path_start;
    reg [15:0] active_wait;
    assign source_base_p = saved_approach[15:0];
    assign destination_base_p = saved_place_approach[15:0];
    wire tick_ms = ms_counter == MS_DIV - 1;
    wire live = rst_n && enable && calibration_valid && !fault && !estop;
    assign busy = state != IDLE && state != PAUSE && state != HOLD;
    assign pregrasp_ready = state == PAUSE && live;

    // 只检查协议 P 值范围 500～2500；这里不增加机械限位或角度补偿。
    function pose_valid;
        input [95:0] pose;
        integer i;
        begin
            pose_valid = 1;
            for (i = 0; i < 6; i = i + 1)
            if (pose[i * 16 +: 16] < 500 || pose[i * 16 +: 16] > 2500) pose_valid = 0;
        end
    endfunction
    wire config_valid = calibration_valid && move_ms >= MIN_MOVE_MS && move_ms <= 9999 &&
    calibrated_pixel_x < 96 && calibrated_pixel_y < 96 && calibrated_radius < 96 &&
    pose_valid(observation_pose) && pose_valid(approach_pose) &&
    pose_valid(descent_pose) && pose_valid(grip_pose) && pose_valid(lift_pose);

    // 动作阶段：0 观察、1 接近、2 下降、3 合爪、4 抬升、5 搬运准备，
    // 6 接近放置点、7 放置下降、8 松爪、9 撤离、10 返回观察。
    wire [95:0] selected_pose = (stage == 1) ? saved_approach :
    (stage == 2) ? saved_descent : (stage == 3) ? saved_grip :
    (stage == 4) ? saved_lift :
    (stage == 5) ? {16'd1700, saved_observe[79:16],
    (BASE_DIRECT_COMMAND != 0 ? saved_lift[15:0] : saved_observe[15:0])} :
    (stage == 6) ? saved_place_approach : (stage == 7) ? saved_place_descent :
    (stage == 8) ? saved_release : (stage == 9) ? saved_retreat : saved_observe;

    // 发送通路：原有底座专用发送器与上部关节发送器，按状态选择输出。
    wire master_ready, master_done, master_error, master_tx, unused_oe;
    wire base_ready, base_done, base_error, base_tx;
    wire upper_ready, upper_done, upper_error, upper_tx;
    wire dedicated_base = BASE_DIRECT_COMMAND != 0 &&
    (state == ISSUE_BASE || state == WAIT_BASE_TX || state == WAIT_BASE_MOVE);
    assign master_ready = dedicated_base ? base_ready : upper_ready;
    assign master_done = dedicated_base ? base_done : upper_done;
    assign master_error = base_error || upper_error;
    assign master_tx = dedicated_base ? base_tx : upper_tx;
    // 在观察或搬运的避障姿态下先转底座，再让上部关节接近目标。
    // 垂直运动采用间隔1毫米的逆运动学路径点，每次单关节微步不超过3P。
    // 上一关节的定时等待结束后，才开始发送下一关节的指令。
    // 计数器记录指令提交次数，不代表机械位置反馈。
    wire base_only = state == ISSUE_BASE || state == WAIT_BASE_TX;
    // 新锁定的抓取或放置方向始终单独提交底座指令，
    // 并执行定时等待，即使目标 P 值与之前提交的数值相同。

    // 新抓取点或放置点始终单独提交底座目标，并等待原设定的运动时间。
    wire force_target_bearing = stage == 1 || (PLACE_MODE != 0 && stage == 6);
    wire base_unchanged = base_command_known && base_command_p == selected_pose[15:0] && !force_target_bearing;
    wire base_submit = state == ISSUE_BASE && !base_unchanged;

    // 下降和抬升采用原有逐毫米路径；关节微步与动作顺序保持不变。
    wire path_stage = BASE_DIRECT_COMMAND != 0 && MAPPED_MODE != 0 &&
    (stage == 2 || stage == 4 || (PLACE_MODE != 0 && (stage == 7 || stage == 9)));
    wire reverse_upper = path_stage || stage == 1 || stage == 6;
    wire [2:0] next_upper = reverse_upper && upper_joint <= 3 ?
    (upper_joint == 1 ? 3'd4 : upper_joint - 1'b1) : upper_joint + 1'b1;
    wire [15:0] path_pixel = stage >= 7 ? placement_pixel : target_pixel;
    wire [35:0] path_pose;
    wire path_valid;
    vertical_path_lookup path_lookup (
        .clk(clk),
        .pixel(path_pixel),
        .height_mm(path_height),
        .pose(path_pose),
        .valid(path_valid)
    );

    // 原定高度：下降至 20 mm，抬升至 90 mm，沿用当前高度基准。
    wire [6:0] path_end_height = (stage == 2) ? 7'd20 : (stage == 7) ? 7'd20 : 7'd90;
    wire path_down = stage == 2 || stage == 7;
    function [15:0] delta_abs;
        input [15:0] a, b;
        begin
            delta_abs = a > b ? a - b : b - a;
        end
    endfunction
    wire [15:0] delta1 = delta_abs({4'd0, path_pose[11:0]}, path_start[15:0]);
    wire [15:0] delta2 = delta_abs({4'd0, path_pose[23:12]}, path_start[31:16]);
    wire [15:0] delta3 = delta_abs({4'd0, path_pose[35:24]}, path_start[47:32]);
    wire [15:0] delta12 = delta1 > delta2 ? delta1 : delta2;
    wire [15:0] delta_max = delta12 > delta3 ? delta12 : delta3;

    // 对路径起止 P 值做原有整数插值；不改变步数、位宽和舍入方式。
    function [15:0] interpolate;
        input [15:0] start_p, end_p;
        input [4:0] step;
        input [2:0] shift;
        reg signed [31:0] difference, scaled, value;
        begin
            difference = $signed({1'b0, end_p}) - $signed({1'b0, start_p});
            scaled = difference * $signed({1'b0, step});
            value = $signed({1'b0, start_p}) + (scaled >>> shift);
            interpolate = value[15:0];
        end
    endfunction
    wire [1:0] path_joint_index = upper_joint >= 1 && upper_joint <= 3 ? upper_joint - 1'b1 : 2'd0;
    wire [15:0] path_joint_target = interpolate(path_start[path_joint_index * 16 +: 16],
        {4'd0, path_pose[path_joint_index * 12 +: 12]}, path_substep, path_shift);
    wire [15:0] upper_target = path_stage && upper_joint <= 3 ? path_joint_target : selected_pose[upper_joint * 16 +: 16];
    wire [15:0] command_duration = path_stage && !base_only ? PATH_MOVE_MS : saved_duration;
    wire upper_unchanged = upper_known[upper_joint] && last_upper_pose[upper_joint * 16 +: 16] == upper_target;
    assign active_joint = (state == ISSUE_BASE || state == WAIT_BASE_TX || state == WAIT_BASE_MOVE) ? 3'd0 :
    ((BASE_DIRECT_COMMAND != 0 && (state == ISSUE || state == WAIT_TX || state == WAIT_MOVE)) ? upper_joint : 3'd7);
    wire [3:0] command_count = BASE_DIRECT_COMMAND != 0 ? 4'd1 : 4'd6;
    wire [47:0] command_ids = BASE_DIRECT_COMMAND != 0 ?
    (base_only ? 48'd0 : {40'd0, 5'd0, upper_joint}) :
    {8'd5, 8'd4, 8'd3, 8'd2, 8'd1, 8'd0};
    wire [95:0] command_pose = BASE_DIRECT_COMMAND != 0 && !base_only ?
    {80'd0, upper_target} : selected_pose;

    // 松爪条件：放置下降完成并置位 placement_lowered 后，才允许释放。
    wire release_permitted = PLACE_MODE == 0 || stage != 8 || placement_lowered;

    // 底座 000 号指令提交与完成握手。
    base_servo_link #(
        .CLK_HZ(CLK_HZ),
        .BAUD(SERVO_BAUD)
    ) base_link (
        .clk(clk),
        .rst_n(rst_n),
        .enable(live),
        .request(base_submit && live),
        .position(selected_pose[15:0]),
        .move_ms(saved_duration),
        .ready(base_ready),
        .done(base_done),
        .error(base_error),
        .tx(base_tx)
    );

    // 上部关节按原顺序发送；发送结束后由状态机等待动作时间。
    servo_command_master #(
        .CLK_HZ(CLK_HZ),
        .BAUD(SERVO_BAUD),
        .MAX_SERVOS(6),
        .INCLUDE_GROUP_TAG(1)
    ) actuator (
        .clk(clk),
        .rst_n(rst_n),
        .enable(live),
        .cmd_valid(state == ISSUE && (BASE_DIRECT_COMMAND == 0 || !upper_unchanged) && live && release_permitted),
        .cmd_ready(upper_ready),
        .servo_count(command_count),
        .ids(command_ids),
        .positions(command_pose),
        .times_ms({6 {command_duration}}),
        .group_id(16'd0),
        .cmd_done(upper_done),
        .cmd_error(upper_error),
        .tx(upper_tx),
        .oe(unused_oe)
    );
    assign servo_tx = live ? master_tx : 1'b1;

    // 相机与模型链路：RGB888 数据流，使用 ready/valid 进行传输握手。
    wire [23:0] rgb;
    wire rgb_valid, rgb_ready, rgb_last, model_rgb_ready;
    wire buffer_start, buffer_good;
    wire [7:0] buffer_id;
    wire frame_submit, result_accept, model_timeout, in_flight;
    wire [31:0] submit_epoch;
    wire [7:0] submit_frame_id;
    reg [31:0] active_epoch;
    reg [7:0] active_frame_id;
    reg job_active;
    wire model_error, fv, ov;
    wire [7:0] fc, fx, fy, oc, ox, oy;

    // 50 位检测总线从高到低：物体 Y/X/置信度/有效位，再到手指同组字段。
    assign detections = {oy, ox, oc, ov, fy, fx, fc, fv};
    assign rgb_ready = job_active ? model_rgb_ready : 1'b1;
    camera_rgb_pipeline #(
        .CLK_HZ(CLK_HZ),
        .BAUD(CAMERA_BAUD)
    ) camera (
        .clk(clk),
        .rst_n(rst_n),
        .rx(camera_rx),
        .frame_good(frame_good),
        .frame_bad(frame_bad),
        .frame_id(),
        .rgb_data(rgb),
        .rgb_valid(rgb_valid),
        .rgb_ready(rgb_ready),
        .rgb_last(rgb_last),
        .processing(),
        .buffer_frame_start(buffer_start),
        .buffer_frame_good(buffer_good),
        .buffer_frame_id(buffer_id)
    );

    // 只接纳观察阶段的完整新帧，携带任务代次与帧号，排除旧结果。
    observation_frame_gate #(
        .SETTLE_MS(OBS_SETTLE_MS),
        .RESULT_TIMEOUT_MS(5000)
    ) frame_gate (
        .clk(clk),
        .rst_n(rst_n),
        .tick_ms(tick_ms),
        .enable(live),
        .fault(fault),
        .estop(estop),
        .robot_busy(state != OBSERVE),
        .at_observation_pose(state == OBSERVE),
        .cancel_capture(1'b0),
        .frame_start(buffer_start),
        .frame_good(buffer_good),
        .frame_bad(frame_bad),
        .frame_id(buffer_id),
        .model_ready(!model_busy && !job_active),
        .model_result_valid(model_result_valid),
        .result_epoch(active_epoch),
        .result_frame_id(active_frame_id),
        .observe_ready(observe_ready),
        .frame_submit(frame_submit),
        .submit_epoch(submit_epoch),
        .submit_frame_id(submit_frame_id),
        .result_accept(result_accept),
        .clear_targets(),
        .model_timeout(model_timeout),
        .in_flight(in_flight)
    );

    // 原有 FPGA 量化模型：输出手指、物体有效标志、置信度和图像坐标。
    native_qnn model (
        .clk(clk),
        .rst_n(rst_n),
        .start(frame_submit),
        .rgb_data(rgb),
        .rgb_valid(rgb_valid && job_active),
        .rgb_last(rgb_last),
        .rgb_ready(model_rgb_ready),
        .busy(model_busy),
        .error(model_error),
        .head_valid(head_valid),
        .head_last(head_last),
        .head_index(head_index),
        .head_acc(head_acc),
        .result_valid(model_result_valid),
        .finger_valid(fv),
        .object_valid(ov),
        .finger_confidence(fc),
        .finger_x(fx),
        .finger_y(fy),
        .object_confidence(oc),
        .object_x(ox),
        .object_y(oy)
    );
    wire [7:0] dx = ox > saved_x ? ox - saved_x : saved_x - ox;
    wire [7:0] dy = oy > saved_y ? oy - saved_y : saved_y - oy;
    wire target_in_window = ov && oc >= 128 && dx <= saved_radius && dy <= saved_radius;
    wire map_response, map_valid, map_request;
    wire [7:0] selected_x, selected_y;
    wire [95:0] mapped_approach, mapped_descent, mapped_grip, mapped_lift;
    wire [95:0] mapped_place_approach, mapped_place_descent, mapped_release, mapped_retreat;
    wire [15:0] mapped_place_pixel;

    // 两点模式选用 V4 圆内选择器；其他构建分支沿用原有行为。
    generate
        if (MAPPED_MODE != 0 && PLACE_MODE != 0) begin : pick_place_mapping
            wire [15:0] source_pixel;
            pick_place_target_mapper mapper (
                .clk(clk),
                .rst_n(rst_n),
                .enable(live && state == OBSERVE && !capture_only),
                .result_valid(result_accept),
                .detections(detections),
                .response(map_response),
                .valid(map_valid),
                .phase(selection_phase),
                .source_pixel(source_pixel),
                .destination_pixel(mapped_place_pixel),
                .source_approach(mapped_approach),
                .source_descent(mapped_descent),
                .source_grip(mapped_grip),
                .source_lift(mapped_lift),
                .destination_approach(mapped_place_approach),
                .destination_descent(mapped_place_descent),
                .destination_release(mapped_release),
                .destination_retreat(mapped_retreat)
            );
            assign selected_x = source_pixel[7:0];
            assign selected_y = source_pixel[15:8];
            assign map_request = 0;
        end else if (MAPPED_MODE != 0) begin : mapping
            assign selection_phase = 0;
            assign mapped_place_pixel = 0;
            assign mapped_place_approach = 0;
            assign mapped_place_descent = 0;
            assign mapped_release = 0;
            assign mapped_retreat = 0;
            pointing_target_selector selector (
                .clk(clk),
                .rst_n(rst_n),
                .enable(live && state == OBSERVE && !capture_only),
                .result_valid(result_accept),
                .detections(detections),
                .map_response(map_response),
                .map_valid(map_valid),
                .request(map_request),
                .selected_x(selected_x),
                .selected_y(selected_y)
            );
            pixel_pose_lookup lookup (
                .clk(clk),
                .rst_n(rst_n),
                .request(map_request),
                .pixel_x(selected_x),
                .pixel_y(selected_y),
                .response(map_response),
                .valid(map_valid),
                .approach_pose(mapped_approach),
                .descent_pose(mapped_descent),
                .grip_pose(mapped_grip),
                .lift_pose(mapped_lift)
            );
        end else begin : no_mapping
            assign selection_phase = 0;
            assign mapped_place_pixel = 0;
            assign mapped_place_approach = 0;
            assign mapped_place_descent = 0;
            assign mapped_release = 0;
            assign mapped_retreat = 0;
            assign map_response = 0;
            assign map_valid = 0;
            assign map_request = 0;
            assign selected_x = 0;
            assign selected_y = 0;
            assign mapped_approach = 0;
            assign mapped_descent = 0;
            assign mapped_grip = 0;
            assign mapped_lift = 0;
        end
    endgenerate

    // 图像任务寄存器：记录当前提交的帧，并管理一次输入流的有效期。
    always @(posedge clk) begin
        if (!rst_n) begin
            ms_counter <= 0;
            job_active <= 0;
            active_epoch <= 0;
            active_frame_id <= 0;
        end else begin
            ms_counter <= tick_ms ? 0 : ms_counter + 1'b1;
            if (frame_submit) begin
                job_active <= 1;
                active_epoch <= submit_epoch;
                active_frame_id <= submit_frame_id;
            end
            if (job_active && rgb_valid && model_rgb_ready && rgb_last) job_active <= 0;
            if (model_error) job_active <= 0;
        end
    end

    // 主时序：复位/故障优先，其余状态按原有动作与等待顺序推进。
    always @(posedge clk) begin
        if (!rst_n) begin
            state <= IDLE;
            stage <= 0;
            seen_toggle <= 0;
            seen_continue <= 0;
            fault <= 0;
            error_code <= 0;
            cycle_done <= 0;
            saved_observe <= 0;
            saved_approach <= 0;
            saved_descent <= 0;
            saved_grip <= 0;
            saved_lift <= 0;
            saved_place_approach <= 0;
            saved_place_descent <= 0;
            saved_release <= 0;
            saved_retreat <= 0;
            placement_pixel <= 0;
            saved_duration <= 0;
            saved_wait <= 0;
            elapsed_ms <= 0;
            saved_x <= 0;
            saved_y <= 0;
            saved_radius <= 0;
            accepted_cycles <= 0;
            sent_frames <= 0;
            completed_cycles <= 0;
            target_pixel <= 0;
            result_frame_id <= 0;
            base_command_p <= 0;
            base_command_count <= 0;
            base_command_known <= 0;
            upper_joint <= 1;
            upper_known <= 0;
            last_upper_pose <= 0;
            actuator_command_count <= 0;
            placement_lowered <= 0;
            path_height <= 90;
            path_load_cycles <= 0;
            path_shift <= 0;
            path_substep <= 1;
            path_start <= 0;
            active_wait <= 0;
        end else begin
            cycle_done <= 0;
            if (!enable) begin
                state <= IDLE;
                stage <= 0;
                seen_toggle <= request_toggle;
                seen_continue <= continue_toggle;
                fault <= 0;
                error_code <= 0;
                elapsed_ms <= 0;
                base_command_known <= 0;
                upper_joint <= 1;
                upper_known <= 0;
                placement_lowered <= 0;
            end else if (estop || model_error || master_error || model_timeout ||
                (!calibration_valid && state != IDLE)) begin
                state <= HOLD;
                fault <= 1;
                error_code <= estop ? 8'd2 : model_error ? 8'd3 : master_error ? 8'd4 : model_timeout ? 8'd5 : 8'd6;
                seen_toggle <= request_toggle;
                seen_continue <= continue_toggle;
            end else if (!fault) begin
                // 提前发生的继续信号变化直接消耗，不排队触发后续抓取。
                if (continue_toggle != seen_continue) begin
                    seen_continue <= continue_toggle;
                    if (state == PAUSE) begin
                        stage <= 2;
                        state <= FIRST_ISSUE;
                    end
                end
                if (request_toggle != seen_toggle) begin
                    seen_toggle <= request_toggle;
                    if (state == IDLE) begin
                        if (!config_valid) begin
                            fault <= 1;
                            state <= HOLD;
                            error_code <= 1;
                        end
                        else begin
                            saved_observe <= observation_pose;
                            saved_approach <= approach_pose;
                            saved_descent <= descent_pose;
                            saved_grip <= grip_pose;
                            saved_lift <= lift_pose;
                            saved_duration <= move_ms;
                            saved_wait <= move_ms + MOVE_SETTLE_MS;
                            saved_x <= calibrated_pixel_x;
                            saved_y <= calibrated_pixel_y;
                            saved_radius <= calibrated_radius;
                            target_pixel <= 0;
                            stage <= 0;
                            state <= FIRST_ISSUE;
                            accepted_cycles <= accepted_cycles + 1'b1;
                        end
                    end
                end
                case (state)

                    // 先发底座目标，再等待发送完成与动作时间，随后才进入上部关节步骤。
                    ISSUE_BASE : if (base_unchanged) begin
                        upper_joint <= reverse_upper ? 3'd3 : 3'd1;
                        if (path_stage) begin
                            path_height <= path_down ? 7'd89 : 7'd21;
                            path_start <= last_upper_pose[63:16];
                            path_load_cycles <= 0;
                            state <= LOAD_PATH;
                        end else state <= ISSUE;
                    end
                    else if (master_ready) begin
                        base_command_p <= selected_pose[15:0];
                        base_command_count <= base_command_count + 1'b1;
                        base_command_known <= 1;
                        state <= WAIT_BASE_TX;
                        actuator_command_count <= actuator_command_count + 1'b1;
                    end
                    WAIT_BASE_TX : if (master_done) begin
                        elapsed_ms <= 0;
                        state <= WAIT_BASE_MOVE;
                    end
                    WAIT_BASE_MOVE : if (tick_ms) begin
                        if (elapsed_ms + 16'd1 >= saved_wait) begin
                            elapsed_ms <= 0;
                            upper_joint <= reverse_upper ? 3'd3 : 3'd1;
                            if (path_stage) begin
                                path_height <= path_down ? 7'd89 : 7'd21;
                                path_start <= last_upper_pose[63:16];
                                path_load_cycles <= 0;
                                state <= LOAD_PATH;
                            end else state <= ISSUE;
                        end
                        else elapsed_ms <= elapsed_ms + 1'b1;
                    end
                    ISSUE : if (!release_permitted) begin
                        state <= HOLD;
                        fault <= 1;
                        error_code <= 8'd17;
                    end
                    else if (BASE_DIRECT_COMMAND != 0 && upper_unchanged) begin
                        if (upper_joint == 5) state <= FINISH_STAGE;
                        else upper_joint <= next_upper;
                    end else if (master_ready) begin
                        if (BASE_DIRECT_COMMAND != 0) begin
                            last_upper_pose[upper_joint * 16 +: 16] <= upper_target;
                            upper_known[upper_joint] <= 1;
                            actuator_command_count <= actuator_command_count + 1'b1;
                        end
                        state <= WAIT_TX;
                        active_wait <= command_duration + (path_stage ? PATH_SETTLE_MS : MOVE_SETTLE_MS);
                    end
                    WAIT_TX : if (master_done) begin
                        if (BASE_DIRECT_COMMAND == 0) sent_frames <= sent_frames + 1'b1;
                        elapsed_ms <= 0;
                        state <= WAIT_MOVE;
                    end
                    WAIT_MOVE : if (tick_ms) begin
                        if (elapsed_ms + 16'd1 >= active_wait) begin
                            elapsed_ms <= 0;
                            if (BASE_DIRECT_COMMAND != 0) begin
                                if (upper_joint == 5) state <= FINISH_STAGE;
                                else begin
                                    upper_joint <= next_upper;
                                    state <= ISSUE;
                                end
                            end else if (stage == 0) state <= OBSERVE;
                            else if (stage == 1 && CONTINUOUS == 0) state <= PAUSE;
                            else if ((stage == 4 && PLACE_MODE == 0) || (stage == 10 && PLACE_MODE != 0)) begin
                                state <= PLACE_MODE != 0 ? OBSERVE : HOLD;
                                completed_cycles <= completed_cycles + 1'b1;
                                cycle_done <= 1;
                            end else begin
                                stage <= stage + 1'b1;
                                state <= FIRST_ISSUE;
                            end
                        end else elapsed_ms <= elapsed_ms + 1'b1;
                    end

                    // 等待同步 ROM，再细分当前毫米路径，保持原有每次不超过 3P 的规则。
                    LOAD_PATH : begin
                        // 完成两次同步 ROM 读取后，再保存插值起点。
                        // 对每毫米路径细分，使每次关节位置变化不超过3P。
                        if (path_load_cycles < 3) path_load_cycles <= path_load_cycles + 1'b1;
                        else if (!path_valid || !(& upper_known[3:1]) || delta_max > 48 ||
                            path_pose[11:0] < 500 || path_pose[11:0] > 2500 ||
                            path_pose[23:12] < 500 || path_pose[23:12] > 2500 ||
                            path_pose[35:24] < 500 || path_pose[35:24] > 2500) begin
                            state <= HOLD;
                            fault <= 1;
                            error_code <= 8'd18;
                        end else begin
                            path_shift <= delta_max <= 3 ? 0 : delta_max <= 6 ? 1 : delta_max <= 12 ? 2 : delta_max <= 24 ? 3 : 4;
                            path_substep <= 1;
                            upper_joint <= 3;
                            state <= ISSUE;
                        end
                    end

                    // 完成当前动作或微步后，再决定继续路径、进入放置等待或返回观察。
                    FINISH_STAGE : if (path_stage && path_substep <(5'd1 << path_shift)) begin
                        path_substep <= path_substep + 1'b1;
                        upper_joint <= 3;
                        state <= ISSUE;
                    end else if (path_stage && path_height != path_end_height) begin
                        path_height <= path_down ? path_height - 1'b1 : path_height + 1'b1;
                        path_start <= last_upper_pose[63:16];
                        path_load_cycles <= 0;
                        upper_joint <= 3;
                        state <= LOAD_PATH;
                    end else begin
                        sent_frames <= sent_frames + 1'b1;
                        upper_joint <= 1;
                        if (PLACE_MODE != 0 && stage == 7) begin
                            // 放置下降期间保持夹爪闭合；所有关节的定时等待结束后，
                            // 才允许在保持当前臂身姿态的情况下松爪。
                            if (saved_place_descent[79:0] != saved_release[79:0] ||
                                saved_place_descent[95:80] != 1700 || saved_release[95:80] != 1300) begin
                                state <= HOLD;
                                fault <= 1;
                                error_code <= 8'd17;
                            end else begin
                                placement_lowered <= 1;
                                elapsed_ms <= 0;
                                state <= WAIT_PLACE_SETTLE;
                            end
                        end else if (stage == 0) state <= OBSERVE;
                        else if (stage == 1 && CONTINUOUS == 0) state <= PAUSE;
                        else if ((stage == 4 && PLACE_MODE == 0) || (stage == 10 && PLACE_MODE != 0)) begin
                            state <= PLACE_MODE != 0 ? OBSERVE : HOLD;
                            completed_cycles <= completed_cycles + 1'b1;
                            cycle_done <= 1;
                        end else begin
                            if (stage == 8) placement_lowered <= 0;
                            stage <= stage + 1'b1;
                            state <= FIRST_ISSUE;
                        end
                    end

                    // 下降完成后保留原有停稳时间，再进入松爪阶段。
                    WAIT_PLACE_SETTLE : if (tick_ms) begin
                        if (elapsed_ms + 16'd1 >= PLACE_SETTLE_MS) begin
                            elapsed_ms <= 0;
                            stage <= 8;
                            state <= FIRST_ISSUE;
                        end
                        else elapsed_ms <= elapsed_ms + 1'b1;
                    end

                    // 等待新两点选择；查表姿态有效时一次保存，随后执行完整抓放。
                    OBSERVE : begin
                        if (result_accept) result_frame_id <= active_frame_id;
                        if (MAPPED_MODE != 0) begin
                            if (map_response && !capture_only) begin
                                if (map_valid && pose_valid(mapped_approach) && pose_valid(mapped_descent) &&
                                    pose_valid(mapped_grip) && pose_valid(mapped_lift) &&
                                    (PLACE_MODE == 0 || (pose_valid(mapped_place_approach) && pose_valid(mapped_place_descent) &&
                                    pose_valid(mapped_release) && pose_valid(mapped_retreat)))) begin
                                    saved_approach <= mapped_approach;
                                    saved_descent <= mapped_descent;
                                    saved_grip <= mapped_grip;
                                    saved_lift <= mapped_lift;
                                    saved_place_approach <= mapped_place_approach;
                                    saved_place_descent <= mapped_place_descent;
                                    saved_release <= mapped_release;
                                    saved_retreat <= mapped_retreat;
                                    placement_pixel <= mapped_place_pixel;
                                    placement_lowered <= 0;
                                    if (PLACE_MODE != 0 && completed_cycles != 0) accepted_cycles <= accepted_cycles + 1'b1;
                                    target_pixel <= {selected_y, selected_x};
                                    error_code <= 0;
                                    stage <= 1;
                                    state <= FIRST_ISSUE;
                                end else error_code <= 8'd16;  // 该点没有有效姿态映射，继续保持观察状态。
                            end
                        end else if (result_accept && target_in_window) begin
                            target_pixel <= {oy, ox};
                            stage <= 1;
                            state <= FIRST_ISSUE;
                        end
                    end
                    default : begin
                    end
                endcase
            end
        end
    end
endmodule
