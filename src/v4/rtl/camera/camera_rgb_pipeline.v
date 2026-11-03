// 图像接收链路：UART 字节 → RGB565 数据包校验 → 帧缓存 → 96×96 RGB888。
// buffer_frame_* 对应真正接入缓存的帧，供上层新帧门控使用。

`timescale 1ns / 1ps
module camera_rgb_pipeline #(
    parameter integer CLK_HZ = 50000000, BAUD = 921600)(
    input wire clk, rst_n, rx,
    output wire frame_good, frame_bad,
    output wire [7:0] frame_id,
    output wire [23:0] rgb_data,
    output wire rgb_valid,
    input wire rgb_ready,
    output wire rgb_last, processing,
    // 这些信号描述真正接纳进 RAM 的图像帧；仅有原始 CRC 正确信号，
    // 仍可能对应处理上一帧期间被整帧丢弃的数据。
    output wire buffer_frame_start, buffer_frame_good,
    output wire [7:0] buffer_frame_id
);
    wire [7:0] data;
    wire valid, error, start, pv;
    wire [15:0] pixel, addr;

    // 第一层：把相机串行位流恢复为字节。
    uart_rx #(
        .CLK_HZ(CLK_HZ),
        .BAUD(BAUD)
    ) uart (
        .clk(clk),
        .rst_n(rst_n),
        .rx(rx),
        .data(data),
        .valid(valid),
        .frame_error(error)
    );

    // 第二层：解析原有图像包格式并验证整帧 CRC。
    camera_rgb565_rx #(
        .CLK_HZ(CLK_HZ)
    ) packet (
        .clk(clk),
        .rst_n(rst_n),
        .byte_data(data),
        .byte_valid(valid),
        .byte_error(error),
        .frame_start(start),
        .pixel_valid(pv),
        .pixel(pixel),
        .pixel_addr(addr),
        .frame_good(frame_good),
        .frame_bad(frame_bad),
        .frame_id(frame_id)
    );

    // 第三层：缓存、双线性缩放，并输出模型需要的 RGB 像素流。
    rgb565_resize96 resize (
        .clk(clk),
        .rst_n(rst_n),
        .frame_start(start),
        .frame_good(frame_good),
        .pixel_valid(pv),
        .pixel(pixel),
        .pixel_addr(addr),
        .frame_id(frame_id),
        .buffer_frame_start(buffer_frame_start),
        .buffer_frame_good(buffer_frame_good),
        .buffer_frame_id(buffer_frame_id),
        .rgb_data(rgb_data),
        .rgb_valid(rgb_valid),
        .rgb_ready(rgb_ready),
        .rgb_last(rgb_last),
        .processing(processing)
    );
endmodule
