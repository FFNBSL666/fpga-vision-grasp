// 板端模型封装：顺序计算核心产生检测头累加值，解码器输出手指与物体结果。
// 模型计算和结果解码均在 FPGA 中完成；本模块不驱动机械臂。

`timescale 1ns / 1ps
// 候选模型的计算和解码均在 FPGA 逻辑及 RAM 中完成。
module native_qnn(
    input wire clk, rst_n, start,
    input wire [23:0] rgb_data,
    input wire rgb_valid, rgb_last,
    output wire rgb_ready, busy, error,
    output wire head_valid, head_last,
    output wire [8:0] head_index,
    output wire signed [31:0] head_acc,
    output wire result_valid, finger_valid, object_valid,
    output wire [7:0] finger_confidence, finger_x, finger_y, object_confidence, object_x, object_y
);
    wire engine_busy, decode_busy, engine_error, decode_error;
    assign busy = engine_busy || decode_busy || head_valid;
    assign error = engine_error || decode_error;
    qnn_sequential_core core (
        .clk(clk),
        .rst_n(rst_n),
        .start(start),
        .rgb_data(rgb_data),
        .rgb_valid(rgb_valid),
        .rgb_last(rgb_last),
        .rgb_ready(rgb_ready),
        .busy(engine_busy),
        .error(engine_error),
        .done(),
        .head_valid(head_valid),
        .head_last(head_last),
        .head_index(head_index),
        .head_acc(head_acc)
    );
    qnn_result_decode decode (
        .clk(clk),
        .rst_n(rst_n),
        .head_valid(head_valid),
        .head_last(head_last),
        .head_index(head_index),
        .head_acc(head_acc),
        .busy(decode_busy),
        .result_valid(result_valid),
        .error(decode_error),
        .finger_valid(finger_valid),
        .object_valid(object_valid),
        .finger_confidence(finger_confidence),
        .finger_x(finger_x),
        .finger_y(finger_y),
        .object_confidence(object_confidence),
        .object_x(object_x),
        .object_y(object_y)
    );
endmodule
