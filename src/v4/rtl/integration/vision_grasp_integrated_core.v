// 保留的单点集成流程：观察、接收新帧、模型定位、抓取和返回。
// V4 两点搬运使用 confirmed_grasp_core，此模块不负责 3 cm 圆内选择。

`timescale 1ns / 1ps
// 一次新请求的流程：观察姿态、新相机帧、FPGA 量化神经网络，
// 再经过已标定的单点窗口、接近、合爪、抬升和返回。
// 此流程不使用演示姿态表、自动启动、重复请求或示例角度。
module vision_grasp_integrated_core #(
    parameter integer CLK_HZ = 50000000, CAMERA_BAUD = 921600, SERVO_BAUD = 115200,
    parameter integer MIN_MOVE_MS = 2000, MOVE_SETTLE_MS = 200, OBS_SETTLE_MS = 500
)(
    input wire clk, rst_n, camera_rx, enable, calibration_valid, request_toggle, estop,
    input wire [15:0] move_ms,
    input wire [95:0] observation_pose, approach_pose, grip_pose, lift_pose,
    input wire [7:0] calibrated_pixel_x, calibrated_pixel_y, calibrated_radius,
    output wire servo_tx, busy,
    output reg fault,
    output reg [7:0] error_code,
    output reg [3:0] state,
    output reg [31:0] accepted_cycles, sent_frames, completed_cycles,
    output reg cycle_done,
    output wire frame_good, frame_bad, model_busy, model_result_valid, observe_ready,
    output wire [49:0] detections,
    output reg [15:0] target_pixel,
    output reg [7:0] result_frame_id,
    output wire head_valid, head_last,
    output wire [8:0] head_index,
    output wire signed [31:0] head_acc
);
    localparam [3:0] IDLE = 0, ISSUE = 1, WAIT_TX = 2, WAIT_MOVE = 3, OBSERVE = 4, HOLD = 5;
    localparam integer MS_DIV = CLK_HZ / 1000;
    reg seen_toggle;
    reg [2:0] stage;
    reg [95:0] saved_observe, saved_approach, saved_grip, saved_lift;
    reg [15:0] saved_duration, saved_wait, elapsed_ms;
    reg [7:0] saved_x, saved_y, saved_radius;
    reg [31:0] ms_counter;
    wire tick_ms = ms_counter == MS_DIV - 1;
    wire live = rst_n && enable && calibration_valid && !fault && !estop;
    assign busy = state != IDLE;
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
    pose_valid(grip_pose) && pose_valid(lift_pose);
    wire [95:0] selected_pose = (stage == 1) ? saved_approach :
    (stage == 2) ? saved_grip : (stage == 3) ? saved_lift : saved_observe;
    wire master_ready, master_done, master_error, master_tx, unused_oe;
    servo_command_master #(
        .CLK_HZ(CLK_HZ),
        .BAUD(SERVO_BAUD),
        .MAX_SERVOS(6),
        .INCLUDE_GROUP_TAG(1)
    ) actuator (
        .clk(clk),
        .rst_n(rst_n),
        .enable(live),
        .cmd_valid(state == ISSUE && live),
        .cmd_ready(master_ready),
        .servo_count(4'd6),
        .ids({8'd5, 8'd4, 8'd3, 8'd2, 8'd1, 8'd0}),
        .positions(selected_pose),
        .times_ms({6 {saved_duration}}),
        .group_id(16'd0),
        .cmd_done(master_done),
        .cmd_error(master_error),
        .tx(master_tx),
        .oe(unused_oe)
    );
    assign servo_tx = live ? master_tx : 1'b1;

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
    always @(posedge clk) begin
        if (!rst_n) begin
            state <= IDLE;
            stage <= 0;
            seen_toggle <= 0;
            fault <= 0;
            error_code <= 0;
            cycle_done <= 0;
            saved_observe <= 0;
            saved_approach <= 0;
            saved_grip <= 0;
            saved_lift <= 0;
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
        end else begin
            cycle_done <= 0;
            if (!enable) begin
                state <= IDLE;
                stage <= 0;
                seen_toggle <= request_toggle;
                fault <= 0;
                error_code <= 0;
                elapsed_ms <= 0;
            end else if (estop || model_error || master_error || model_timeout ||
                (!calibration_valid && state != IDLE)) begin
                state <= HOLD;
                fault <= 1;
                error_code <= estop ? 8'd2 : model_error ? 8'd3 : master_error ? 8'd4 : model_timeout ? 8'd5 : 8'd6;
                seen_toggle <= request_toggle;
            end else if (!fault) begin
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
                            saved_grip <= grip_pose;
                            saved_lift <= lift_pose;
                            saved_duration <= move_ms;
                            saved_wait <= move_ms + MOVE_SETTLE_MS;
                            saved_x <= calibrated_pixel_x;
                            saved_y <= calibrated_pixel_y;
                            saved_radius <= calibrated_radius;
                            target_pixel <= 0;
                            stage <= 0;
                            state <= ISSUE;
                            accepted_cycles <= accepted_cycles + 1'b1;
                        end
                    end
                end
                case (state)
                    ISSUE : if (master_ready) state <= WAIT_TX;
                    WAIT_TX : if (master_done) begin
                        sent_frames <= sent_frames + 1'b1;
                        elapsed_ms <= 0;
                        state <= WAIT_MOVE;
                    end
                    WAIT_MOVE : if (tick_ms) begin
                        if (elapsed_ms + 16'd1 >= saved_wait) begin
                            elapsed_ms <= 0;
                            if (stage == 0) state <= OBSERVE;
                            else if (stage == 4) begin
                                state <= IDLE;
                                completed_cycles <= completed_cycles + 1'b1;
                                cycle_done <= 1;
                            end else begin
                                stage <= stage + 1'b1;
                                state <= ISSUE;
                            end
                        end else elapsed_ms <= elapsed_ms + 1'b1;
                    end
                    OBSERVE : if (result_accept) begin
                        result_frame_id <= active_frame_id;
                        if (target_in_window) begin
                            target_pixel <= {oy, ox};
                            stage <= 1;
                            state <= ISSUE;
                        end
                    end
                    default : begin
                    end
                endcase
            end
        end
    end
endmodule
