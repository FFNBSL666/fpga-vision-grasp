// 板级入口：连接时钟、复位、相机串口、舵机串口及状态灯。
// V4 构建参数启用 confirmed 分支；旧演示和诊断分支仍由原参数选择。
// 阅读顺序：板级入口 → confirmed_grasp_system → confirmed_grasp_core。

`timescale 1ns / 1ps
// 历史说明：已确认的单点抓取分支由参数选择；当前构建使用哪个分支，以实际参数为准。
// 诊断分支仍保留，由对应构建脚本显式选择。
// 物理按键低电平有效；X 工程用于参考串口协议，不作为动作方案。
module fpga_board_top #(
    parameter integer CLK_HZ = 50000000,
    parameter integer CAMERA_BAUD = 921600,
    parameter integer CONFIRMED_GRASP_ENABLE = 0,
    parameter integer POINTING_GRASP_ENABLE = 0,
    parameter integer POINTING_PLACE_ENABLE = 0,
    parameter integer ACTUATOR_OUTPUT_ENABLE = 0,
    parameter integer LEGACY_DEMO_ENABLE = 0,
    parameter integer MODEL_DIAGNOSTIC_ENABLE = 0,
    parameter integer ENABLE_MODEL_ILA = 0,
    parameter integer MODEL_BACKEND = 1,
    parameter integer MODEL_CONFIDENCE_THRESHOLD = 128,
    parameter integer KEY_DEBOUNCE_CYCLES = 500000
)(
    input wire clk_50m,
    input wire rst_key_n,
    input wire enable_sw,
    input wire estop_sw,
    input wire camera_uart_rx,
    output wire servo_uart_tx,
    output wire busy_led,
    output wire fault_led,
    output wire locked_led
);

    // 板级复位、按键和急停输入。
    wire rst_n, run_enable;
    wire stop_active = ~estop_sw;
    wire core_fault;
    reset_sync resetter (
        .clk(clk_50m),
        .rst_n_in(rst_key_n),
        .rst_n(rst_n)
    );
    board_run_control #(
        .DEBOUNCE_CYCLES(KEY_DEBOUNCE_CYCLES)
    ) buttons (
        .clk(clk_50m),
        .rst_n(rst_n),
        .start_n(enable_sw),
        .estop_n(estop_sw),
        .fault(core_fault),
        .run_enable(run_enable)
    );

    // 构建期分支选择：当前 V4 启用 confirmed 两点抓放链路。
    generate
        if (CONFIRMED_GRASP_ENABLE != 0) begin : confirmed
            wire stop_sync, confirmed_busy, confirmed_fault, done, valid, ready;
            wire [3:0] confirmed_state;
            wire [7:0] error_code, frame_id;
            wire [31:0] accepted, sent, completed;
            wire [49:0] detections;
            wire [15:0] target;
            wire [15:0] destination;
            wire [15:0] base_command_p, source_base_p, destination_base_p;
            wire [31:0] base_command_count;
            wire [2:0] active_joint;
            wire [31:0] actuator_command_count;
            wire [1:0] selection_phase;
            wire capture_only;

            // 50 MHz 时钟分频计数；等待第二点时使用第 24 位产生闪烁。
            reg [24:0] phase_blink;
            always @(posedge clk_50m) begin
                if (!rst_n) phase_blink <= 0;
                else phase_blink <= phase_blink + 1'b1;
            end
            input_sync stop_filter (
                .clk(clk_50m),
                .rst_n(rst_n),
                .signal_in(stop_active),
                .signal_out(stop_sync)
            );
            confirmed_grasp_system #(
                .CLK_HZ(CLK_HZ),
                .CAMERA_BAUD(CAMERA_BAUD),
                .MAPPED_MODE(POINTING_GRASP_ENABLE),
                .PLACE_MODE(POINTING_PLACE_ENABLE),
                .BASE_DIRECT_COMMAND(1)
            ) system (
                .clk(clk_50m),
                .rst_n(rst_n),
                .camera_rx(camera_uart_rx),
                .enable(ACTUATOR_OUTPUT_ENABLE != 0),
                .estop(stop_sync),
                .capture_only(capture_only),
                .servo_tx(servo_uart_tx),
                .busy(confirmed_busy),
                .fault(confirmed_fault),
                .error_code(error_code),
                .state(confirmed_state),
                .accepted_cycles(accepted),
                .sent_frames(sent),
                .completed_cycles(completed),
                .cycle_done(done),
                .model_result_valid(valid),
                .observe_ready(ready),
                .detections(detections),
                .target_pixel(target),
                .placement_pixel(destination),
                .selection_phase(selection_phase),
                .base_command_p(base_command_p),
                .base_command_count(base_command_count),
                .source_base_p(source_base_p),
                .destination_base_p(destination_base_p),
                .active_joint(active_joint),
                .actuator_command_count(actuator_command_count),
                .result_frame_id(frame_id),
                .head_valid(),
                .head_last(),
                .head_index(),
                .head_acc()
            );

            // VIO 仅观察/控制调试信号；指令计数不表示舵机实测位移。
            confirmed_grasp_monitor monitor (
                .clk(clk_50m),
                .probe_in0(confirmed_state),
                .probe_in1(confirmed_busy),
                .probe_in2(confirmed_fault),
                .probe_in3(error_code),
                .probe_in4(sent),
                .probe_in5(completed),
                .probe_in6(detections),
                .probe_in7(valid),
                .probe_in8(frame_id),
                .probe_in9(target),
                .probe_in10(ready),
                .probe_in11(done),
                .probe_in12(selection_phase),
                .probe_in13(destination),
                .probe_in14(base_command_p),
                .probe_in15(base_command_count),
                .probe_in16(source_base_p),
                .probe_in17(destination_base_p),
                .probe_in18(active_joint),
                .probe_in19(actuator_command_count),
                .probe_out0(capture_only)
            );
            assign core_fault = confirmed_fault;
            // H15 对应底板 PL_LED0：等待选择物体时不亮；
            // 仅在等待第二次选择放置点时闪烁。
            // 第二次选点被接纳或取消后，立即停止闪烁。

            // PL_LED0：仅观察状态且 phase=1 时闪烁，第二点锁定后停止。
            assign busy_led = !confirmed_fault && !stop_sync && confirmed_state == 4 &&
            selection_phase == 1 ? phase_blink[24] : 1'b0;
            assign fault_led = confirmed_fault || stop_sync;
            assign locked_led = (ACTUATOR_OUTPUT_ENABLE == 0) || confirmed_fault || stop_sync;
        end else if (LEGACY_DEMO_ENABLE != 0) begin : legacy
            wire busy, locked;
            fpga_grasp_top #(
                .CLK_HZ(CLK_HZ),
                .CAMERA_BAUD(115200),
                .ACTUATOR_OUTPUT_ENABLE(ACTUATOR_OUTPUT_ENABLE),
                .MODEL_BACKEND(MODEL_BACKEND),
                .MODEL_CONFIDENCE_THRESHOLD(MODEL_CONFIDENCE_THRESHOLD)
            ) core (
                .clk(clk_50m),
                .rst_n_in(rst_key_n),
                .camera_rx(camera_uart_rx),
                .arm_enable_in(run_enable),
                .estop_in(stop_active),
                .servo_rx(1'b1),
                .servo_tx(servo_uart_tx),
                .servo_oe(),
                .busy(busy),
                .fault(core_fault),
                .state_debug(),
                .fault_code(),
                .gesture_debug(),
                .region_debug(),
                .object_x_debug(),
                .object_y_debug(),
                .object_found_debug(),
                .frame_good_debug(),
                .cycle_done(),
                .feedback_id_debug(),
                .feedback_position_debug(),
                .feedback_valid_debug(),
                .actuator_locked(locked)
            );
            // 依据底板原理图第3页，PL 指示灯由高电平点亮。
            assign busy_led = busy;
            assign fault_led = core_fault;
            // N15 对应 ATK MODULE 第1脚 GBC_LED，不是底板上的第三个 PL 指示灯。
            assign locked_led = locked;
        end else if (MODEL_DIAGNOSTIC_ENABLE != 0) begin : model_check
            wire good, bad, model_busy, model_error, result_valid;
            wire [49:0] detections;
            wire [7:0] result_frame_id;
            wire [31:0] model_cycles;
            reg [31:0] recent_count, blink_count;
            reg error_latched, object_seen;
            model_diagnostic_core #(
                .CLK_HZ(CLK_HZ),
                .BAUD(CAMERA_BAUD)
            ) recognition (
                .clk(clk_50m),
                .rst_n(rst_n),
                .camera_rx(camera_uart_rx),
                .frame_good(good),
                .frame_bad(bad),
                .model_busy(model_busy),
                .model_error(model_error),
                .result_valid(result_valid),
                .detections(detections),
                .result_frame_id(result_frame_id),
                .model_cycles(model_cycles),
                .head_valid(),
                .head_last(),
                .head_index(),
                .head_acc()
            );
            always @(posedge clk_50m) begin
                if (!rst_n) begin
                    recent_count <= 0;
                    blink_count <= 0;
                    error_latched <= 0;
                    object_seen <= 0;
                end
                else begin
                    blink_count <= (blink_count >= CLK_HZ / 5 - 1) ? 0 : blink_count + 1'b1;
                    if (good) recent_count <= CLK_HZ * 2;
                    else if (recent_count != 0) recent_count <= recent_count - 1'b1;
                    if (bad || model_error) error_latched <= 1;
                    if (result_valid) object_seen <= detections[25];
                end
            end
            assign core_fault = error_latched;
            assign busy_led = (recent_count != 0);
            assign fault_led = error_latched ? (blink_count < CLK_HZ / 10) : (object_seen && recent_count != 0);
            assign servo_uart_tx = 1'b1;
            assign locked_led = 1'b1;
            if (ENABLE_MODEL_ILA != 0) begin : debug
                model_ila ila (
                    .clk(clk_50m),
                    .probe0(result_valid),
                    .probe1(detections),
                    .probe2(result_frame_id),
                    .probe3(model_cycles)
                );
            end
        end else begin : camera_check
            wire good, bad, tensor_valid, tensor_last, processing;
            (* MARK_DEBUG = "TRUE" *) wire [23:0] tensor_rgb;
            wire [7:0] frame_id;
            reg [31:0] recent_count;
            reg error_latched;
            (* DONT_TOUCH = "TRUE" *) camera_rgb_pipeline #(.CLK_HZ(CLK_HZ),
                .BAUD(CAMERA_BAUD)) camera(
                .clk(clk_50m),
                .rst_n(rst_n),
                .rx(camera_uart_rx),
                .frame_good(good),
                .frame_bad(bad),
                .frame_id(frame_id),
                .rgb_data(tensor_rgb),
                .rgb_valid(tensor_valid),
                .rgb_ready(1'b1),
                .rgb_last(tensor_last),
                .processing(processing)
            );
            always @(posedge clk_50m) begin
                if (!rst_n) begin
                    recent_count <= 0;
                    error_latched <= 0;
                end
                else begin
                    if (tensor_valid && tensor_last) recent_count <= CLK_HZ * 2;
                    else if (recent_count != 0) recent_count <= recent_count - 1'b1;
                    if (bad) error_latched <= 1;
                end
            end
            assign core_fault = error_latched;
            assign servo_uart_tx = 1'b1;
            assign busy_led = (recent_count != 0);
            assign fault_led = error_latched;
            assign locked_led = 1'b1;
        end
    endgenerate
endmodule
