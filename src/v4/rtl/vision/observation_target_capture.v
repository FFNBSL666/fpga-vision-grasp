// 观察定位接口：组合新帧门控与目标选择，不在本模块实现神经网络运算。

`timescale 1ns / 1ps
// 面向两类目标 FPGA 模型、带观察姿态条件的前端。
// 本模块用于实际链路集成，不实现模型推理运算。
// observation_pose 由已标定的运动控制器提供；这个信号
// 不代表存在物理位置传感器，也不根据照片推断关节零位。
module observation_target_capture #(
    parameter integer SETTLE_MS = 500, RESULT_TIMEOUT_MS = 5000, HOLD_MS = 2000,
    parameter integer FRAME_TIMEOUT_MS = 1500, STABLE_TOL = 4, CONF_THRESHOLD = 128
)(
    input wire clk, rst_n, tick_ms, enable, fault, estop, robot_busy, at_observation_pose,
    input wire frame_start, frame_good, frame_bad,
    input wire [7:0] frame_id,
    input wire model_ready, model_result_valid,
    input wire [31:0] result_epoch,
    input wire [7:0] result_frame_id,
    input wire object_valid, finger_valid,
    input wire [7:0] object_confidence, finger_confidence,
    input wire [7:0] object_x, object_y, finger_x, finger_y,
    input wire clear_capture,
    output wire observe_ready, frame_submit,
    output wire [31:0] submit_epoch,
    output wire [7:0] submit_frame_id,
    output wire result_accept, model_timeout, in_flight,
    output wire object_latched,
    output wire [7:0] object_center_x, object_center_y,
    output wire finger_tracking,
    output wire [15:0] stable_ms,
    output wire target_valid,
    output wire [7:0] target_x, target_y,
    output wire place_request
);
    wire invalidate;
    observation_frame_gate #(
        .SETTLE_MS(SETTLE_MS),
        .RESULT_TIMEOUT_MS(RESULT_TIMEOUT_MS)
    ) gate (
        .clk(clk),
        .rst_n(rst_n),
        .tick_ms(tick_ms),
        .enable(enable),
        .fault(fault),
        .estop(estop),
        .robot_busy(robot_busy),
        .at_observation_pose(at_observation_pose),
        .cancel_capture(clear_capture),
        .frame_start(frame_start),
        .frame_good(frame_good),
        .frame_bad(frame_bad),
        .frame_id(frame_id),
        .model_ready(model_ready),
        .model_result_valid(model_result_valid),
        .result_epoch(result_epoch),
        .result_frame_id(result_frame_id),
        .observe_ready(observe_ready),
        .frame_submit(frame_submit),
        .submit_epoch(submit_epoch),
        .submit_frame_id(submit_frame_id),
        .result_accept(result_accept),
        .clear_targets(invalidate),
        .model_timeout(model_timeout),
        .in_flight(in_flight)
    );
    point_hold_capture #(
        .HOLD_MS(HOLD_MS),
        .FRAME_TIMEOUT_MS(FRAME_TIMEOUT_MS),
        .STABLE_TOL(STABLE_TOL),
        .CONF_THRESHOLD(CONF_THRESHOLD)
    ) capture (
        .clk(clk),
        .rst_n(rst_n),
        .tick_ms(tick_ms),
        .frame_valid(result_accept),
        .object_valid(object_valid),
        .finger_valid(finger_valid),
        .object_confidence(object_confidence),
        .finger_confidence(finger_confidence),
        .object_x(object_x),
        .object_y(object_y),
        .finger_x(finger_x),
        .finger_y(finger_y),
        .clear_capture(clear_capture || invalidate),
        .object_latched(object_latched),
        .object_center_x(object_center_x),
        .object_center_y(object_center_y),
        .finger_tracking(finger_tracking),
        .stable_ms(stable_ms),
        .target_valid(target_valid),
        .target_x(target_x),
        .target_y(target_y),
        .place_request(place_request)
    );
endmodule
