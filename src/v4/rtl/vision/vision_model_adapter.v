// 阅读提示：这是旧接口适配层；V4 主链路实际使用 native_qnn 模型封装。

`timescale 1ns / 1ps
// 视觉模型适配层。
//
// 这个模块保留原工程 gesture_cnn_demo 的时序接口，便于把真正的
// FINN/QNN 推理 IP 接入原有抓取流程。当前 MODEL_BACKEND=0 时使用
// 原工程的亮度质心演示算法，保证原有仿真和舵机控制逻辑保持不变。
// MODEL_BACKEND=1 预留给后续 FINN 模型；没有接入真实模型时不会产生
// 有效结果，从而避免把未训练的占位逻辑误当成真实识别结果。
module vision_model_adapter #(
    parameter integer IMAGE_W = 80,
    parameter integer IMAGE_H = 60,
    parameter integer THRESHOLD = 200,
    parameter integer MIN_PIXELS = 4,
    parameter integer MODEL_BACKEND = 0,
    parameter integer CONFIDENCE_THRESHOLD = 128
)(
    input wire clk,
    input wire rst_n,
    input wire enable,
    input wire frame_start,
    input wire pixel_valid,
    input wire [7:0] pixel,
    input wire [7:0] x,
    input wire [7:0] y,
    input wire frame_good,
    // 真实FINN/QNN硬件IP的结果接口；当前工程暂时由顶层置为无效。
    input wire fin_result_valid,
    input wire [1:0] fin_class,
    input wire [7:0] fin_confidence,
    output reg result_valid,
    output reg [1:0] gesture,
    output reg [7:0] confidence
);
    reg [23:0] bright_count;
    reg [23:0] bright_sum_x;

    // 旧算法的结果仍然在 FPGA 内完成，作为新模型接入前的安全回退路径。
    always @(posedge clk) begin
        if (!rst_n) begin
            bright_count <= 24'd0;
            bright_sum_x <= 24'd0;
            result_valid <= 1'b0;
            gesture <= 2'd3;
            confidence <= 8'd0;
        end else begin
            result_valid <= 1'b0;

            if (frame_start) begin
                bright_count <= 24'd0;
                bright_sum_x <= 24'd0;
            end else if (enable && pixel_valid &&
                (y < IMAGE_H / 2) && (pixel >= THRESHOLD)) begin
                bright_count <= bright_count + 1'b1;
                bright_sum_x <= bright_sum_x + x;
            end

            if (frame_good && enable) begin
                if (MODEL_BACKEND == 0) begin
                    result_valid <= 1'b1;
                    if (bright_count < MIN_PIXELS) begin
                        gesture <= 2'd3;
                        confidence <= 8'd0;
                    end else begin
                        // 当前回退算法只有“有效/无效”置信度，不冒充模型概率。
                        confidence <= 8'd255;
                        if (bright_sum_x < bright_count * (IMAGE_W / 3))
                        gesture <= 2'd0;
                        else if (bright_sum_x < bright_count *
                            ((IMAGE_W * 2) / 3))
                        gesture <= 2'd1;
                        else
                        gesture <= 2'd2;
                    end
                end else begin
                    // 真实FINN IP的结果必须先经过置信度门限。
                    if (fin_result_valid &&
                        (fin_confidence >= CONFIDENCE_THRESHOLD)) begin
                        result_valid <= 1'b1;
                        gesture <= fin_class;
                        confidence <= fin_confidence;
                    end else begin
                        // 未接入模型或置信度不足时保持无效，禁止误触发。
                        result_valid <= 1'b0;
                        gesture <= 2'd3;
                        confidence <= 8'd0;
                    end
                end
            end
        end
    end

endmodule
