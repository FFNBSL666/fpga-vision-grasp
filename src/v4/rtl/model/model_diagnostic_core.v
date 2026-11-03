// 视觉诊断链路：接收相机图像、缩放、推理并输出检测结果。
// 此模块没有舵机输出接口，板级诊断分支按原有参数选择。

`timescale 1ns / 1ps
// 相机 UART 输入先校验 RGB565，再由 FPGA 缩放为96乘96，最后进入量化模型。
// 这是完整的模型验证通路，不连接电机控制接口。
module model_diagnostic_core #(
    parameter integer CLK_HZ = 50000000, BAUD = 921600)(
    input wire clk, rst_n, camera_rx,
    output wire frame_good, frame_bad, model_busy, model_error, result_valid,
    output wire [49:0] detections,
    output wire [7:0] result_frame_id,
    output wire [31:0] model_cycles,
    output wire head_valid, head_last,
    output wire [8:0] head_index,
    output wire signed [31:0] head_acc
);
    wire [23:0] rgb;
    wire rgb_valid, rgb_last, rgb_ready, model_rgb_ready;
    wire buffer_start, buffer_good;
    wire [7:0] buffer_id;
    reg job_active, start;
    reg [7:0] saved_id;
    reg [31:0] cycles;
    wire fv, ov;
    wire [7:0] fc, fx, fy, oc, ox, oy;
    assign rgb_ready = job_active ? model_rgb_ready : 1'b1;
    assign result_frame_id = saved_id;
    assign model_cycles = cycles;
    assign detections = {oy, ox, oc, ov, fy, fx, fc, fv};
    camera_rgb_pipeline #(
        .CLK_HZ(CLK_HZ),
        .BAUD(BAUD)
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
    native_qnn model (
        .clk(clk),
        .rst_n(rst_n),
        .start(start),
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
        .result_valid(result_valid),
        .finger_valid(fv),
        .object_valid(ov),
        .finger_confidence(fc),
        .finger_x(fx),
        .finger_y(fy),
        .object_confidence(oc),
        .object_x(ox),
        .object_y(oy)
    );
    always @(posedge clk) begin
        if (!rst_n) begin
            job_active <= 0;
            start <= 0;
            saved_id <= 0;
            cycles <= 0;
        end
        else begin
            start <= 0;
            if (buffer_good) begin
                if (!model_busy && !job_active) begin
                    start <= 1;
                    job_active <= 1;
                    saved_id <= buffer_id;
                end
                else job_active <= 0;
            end
            if (job_active && rgb_valid && model_rgb_ready && rgb_last) job_active <= 0;
            if (model_error) job_active <= 0;
            if (start) cycles <= 0;
            else if (model_busy) cycles <= cycles + 1'b1;
        end
    end
endmodule
