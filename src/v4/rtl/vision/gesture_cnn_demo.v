// 模块：手势接口演示（gesture_cnn_demo）
// 作用：用画面上半幅亮区域的位置演示左、中、右选择。
// 输入：像素、坐标、有效帧、模块使能。
// 输出：手势类别和结果有效脉冲。
// 注意：没有真实CNN；以后替换算法，保留接口。
// 阅读：每行右侧说明这行代码在做什么。

`timescale 1ns / 1ps // 仿真时间单位1ns，最小精度1ps；不产生硬件时钟。
module gesture_cnn_demo #( // 定义这个模块；括号中写参数或输入输出接口。
    parameter integer IMAGE_W = 80, // 设置图像宽度；这些常量在综合前确定。
    parameter integer IMAGE_H = 60, // 设置图像高度；这些常量在综合前确定。
    parameter integer THRESHOLD = 200, // 设置亮像素阈值；这些常量在综合前确定。
    parameter integer MIN_PIXELS = 4, // 设置最少亮像素数量；这些常量在综合前确定。
    parameter integer DEMO_MODE = 1 // 设置演示模式开关；这些常量在综合前确定。
) ( // 结束参数列表，开始写输入输出接口。
    input wire clk, // 声明接口：时钟。
    input wire rst_n, // 声明接口：内部低电平复位。
    input wire enable, // 声明接口：模块使能。
    input wire frame_start, // 声明接口：开始一帧的脉冲。
    input wire pixel_valid, // 声明接口：像素有效标志。
    input wire [7:0] pixel, // 声明接口：灰度像素。
    input wire [7:0] x, // 声明接口：当前像素横坐标。
    input wire [7:0] y, // 声明接口：当前像素纵坐标。
    input wire frame_good, // 声明接口：完整好帧脉冲。
    output reg result_valid, // 声明接口：结果有效脉冲。
    output reg [1:0] gesture // 声明接口：手势类别。
); // 结束输入输出接口定义。
    reg [23:0] count; // 声明24位寄存器：上半幅亮像素总数。
    reg [23:0] sum_x; // 声明24位寄存器：横坐标总和。
    always @(posedge clk) begin // 每个时钟上升沿更新下面的数据。
        if (!rst_n) begin // 判断内部低电平复位为0；满足条件才进入后面的步骤。
            count <= 0; // 上半幅亮像素总数设为0。
            sum_x <= 0; // 横坐标总和设为0。
            result_valid <= 0; // 结果有效脉冲设为0。
            gesture <= 3; // 手势类别设为3。
        end // 结束当前代码块。
        else begin // 前一个条件不满足时，执行下面这一块。
            result_valid <= 0; // 结果有效脉冲设为0。
            if (frame_start) begin // 判断开始一帧的脉冲；满足条件才进入后面的步骤。
                count <= 0; // 上半幅亮像素总数设为0。
                sum_x <= 0; // 横坐标总和设为0。
            end // 结束当前代码块。
            else if (pixel_valid && y < IMAGE_H / 2 && pixel >= THRESHOLD) begin // 判断像素有效标志 && 当前像素纵坐标 < 图像高度 / 2 && 灰度像素 >= 亮像素阈值；满足条件才进入后面的步骤。
                count <= count + 1'b1; // 上半幅亮像素总数设为上半幅亮像素总数加1。
                sum_x <= sum_x + x; // 横坐标总和设为横坐标总和 + 当前像素横坐标。
            end // 结束当前代码块。
            if (frame_good && enable) begin // 判断完整好帧脉冲 && 模块使能；满足条件才进入后面的步骤。
                result_valid <= 1; // 结果有效脉冲设为1。
                if (DEMO_MODE == 0 || count < MIN_PIXELS) gesture <= 3; // 判断演示模式开关 == 0 || 上半幅亮像素总数 < 最少亮像素数量；手势类别设为3。
                else if (sum_x < count * (IMAGE_W / 3)) gesture <= 0; // 判断横坐标总和 < 上半幅亮像素总数 * (图像宽度 / 3)；手势类别设为0。
                else if (sum_x < count * ((IMAGE_W * 2) / 3)) gesture <= 1; // 判断横坐标总和 < 上半幅亮像素总数 * ((图像宽度 * 2) / 3)；手势类别设为1。
                else gesture <= 2; // 否则，手势类别设为2。
            end // 结束当前代码块。
        end // 结束当前代码块。
    end // 结束当前代码块。
endmodule // 本模块结束。
