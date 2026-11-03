// 运行封装：启动延时结束后提交初始姿态请求，再由核心持续等待新选点。
// 本层负责连接配置与核心；具体选点、坐标查表和舵机时序在下级模块。

`timescale 1ns / 1ps
// 配置或复位后，恢复一次已配置的观察姿态。
// 在搬运模式中，每组新的两点选择触发一次连贯抓放流程。
module confirmed_grasp_system #(
    parameter integer CLK_HZ = 50000000, CAMERA_BAUD = 921600,
    parameter integer START_DELAY_MS = 2000, MOVE_MS = 5000, MIN_MOVE_MS = 2000,
    parameter integer MOVE_SETTLE_MS = 200, OBS_SETTLE_MS = 500,
    parameter integer TARGET_X = 46, TARGET_Y = 38, TARGET_RADIUS = 4, MAPPED_MODE = 0, PLACE_MODE = 0,
    parameter integer BASE_DIRECT_COMMAND = 0, PATH_MOVE_MS = 20, PATH_SETTLE_MS = 10
)(
    input wire clk, rst_n, camera_rx, enable, estop, capture_only,
    output wire servo_tx, busy, fault,
    output wire [7:0] error_code,
    output wire [3:0] state,
    output wire [31:0] accepted_cycles, sent_frames, completed_cycles,
    output wire cycle_done, model_result_valid, observe_ready,
    output wire [49:0] detections,
    output wire [15:0] target_pixel,
    output wire [15:0] placement_pixel,
    output wire [15:0] base_command_p, source_base_p, destination_base_p,
    output wire [31:0] base_command_count,
    output wire [2:0] active_joint,
    output wire [31:0] actuator_command_count,
    output wire [1:0] selection_phase,
    output wire [7:0] result_frame_id,
    output wire head_valid, head_last,
    output wire [8:0] head_index,
    output wire signed [31:0] head_acc
);
    `include "confirmed_grasp_profile.vh"

    // 启动延时与一次性请求标志；延时结束后恢复保存的观察姿态。
    localparam integer START_CYCLES = (CLK_HZ / 1000) * START_DELAY_MS;
    reg [31:0] startup_count;
    reg requested;
    // 禁用或急停会消耗本次启动请求，不排队保留动作。
    always @(posedge clk) begin
        if (!rst_n) begin
            startup_count <= 0;
            requested <= 0;
        end
        else if (!enable || estop) begin
            startup_count <= 0;
            requested <= 0;
        end
        else if (!requested) begin
            if (START_CYCLES == 0 || startup_count >= START_CYCLES - 1) requested <= 1;
            else startup_count <= startup_count + 1'b1;
        end
    end
    wire unused_pregrasp, unused_good, unused_bad, unused_model_busy;

    // 把姿态常量、运行参数与板级信号交给抓放核心。
    confirmed_grasp_core #(
        .CLK_HZ(CLK_HZ),
        .CAMERA_BAUD(CAMERA_BAUD),
        .MIN_MOVE_MS(MIN_MOVE_MS),
        .MOVE_SETTLE_MS(MOVE_SETTLE_MS),
        .OBS_SETTLE_MS(OBS_SETTLE_MS),
        .CONTINUOUS(1),
        .MAPPED_MODE(MAPPED_MODE),
        .PLACE_MODE(PLACE_MODE),
        .BASE_DIRECT_COMMAND(BASE_DIRECT_COMMAND),
        .PATH_MOVE_MS(PATH_MOVE_MS),
        .PATH_SETTLE_MS(PATH_SETTLE_MS)
    ) core (
        .clk(clk),
        .rst_n(rst_n),
        .camera_rx(camera_rx),
        .enable(enable),
        .calibration_valid(1'b1),
        .request_toggle(requested),
        .continue_toggle(1'b0),
        .estop(estop),
        .capture_only(capture_only),
        .move_ms(MOVE_MS[15:0]),
        .observation_pose(CONFIRMED_OBSERVE),
        .approach_pose(CONFIRMED_APPROACH),
        .descent_pose(CONFIRMED_DESCENT),
        .grip_pose(CONFIRMED_GRIP),
        .lift_pose(CONFIRMED_LIFT),
        .calibrated_pixel_x(TARGET_X[7:0]),
        .calibrated_pixel_y(TARGET_Y[7:0]),
        .calibrated_radius(TARGET_RADIUS[7:0]),
        .servo_tx(servo_tx),
        .busy(busy),
        .pregrasp_ready(unused_pregrasp),
        .fault(fault),
        .error_code(error_code),
        .state(state),
        .accepted_cycles(accepted_cycles),
        .sent_frames(sent_frames),
        .completed_cycles(completed_cycles),
        .cycle_done(cycle_done),
        .frame_good(unused_good),
        .frame_bad(unused_bad),
        .model_busy(unused_model_busy),
        .model_result_valid(model_result_valid),
        .observe_ready(observe_ready),
        .detections(detections),
        .target_pixel(target_pixel),
        .result_frame_id(result_frame_id),
        .placement_pixel(placement_pixel),
        .selection_phase(selection_phase),
        .base_command_p(base_command_p),
        .base_command_count(base_command_count),
        .source_base_p(source_base_p),
        .destination_base_p(destination_base_p),
        .active_joint(active_joint),
        .actuator_command_count(actuator_command_count),
        .head_valid(head_valid),
        .head_last(head_last),
        .head_index(head_index),
        .head_acc(head_acc)
    );
endmodule
