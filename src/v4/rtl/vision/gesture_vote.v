// 模块：连续帧确认（gesture_vote）
// 作用：同一手势连续出现3帧，才允许它作为稳定选择。
// 输入：每帧的类别与结果有效脉冲。
// 输出：稳定手势和确认标志。
// 注意：未知手势会取消确认。
// 阅读：每行右侧说明这行代码在做什么。

`timescale 1ns / 1ps // 仿真时间单位1ns，最小精度1ps；不产生硬件时钟。
module gesture_vote #( // 定义这个模块；括号中写参数或输入输出接口。
    parameter integer REQUIRED_FRAMES = 3 // 设置连续确认帧数；这些常量在综合前确定。
) ( // 结束参数列表，开始写输入输出接口。
    input wire clk, // 声明接口：时钟。
    input wire rst_n, // 声明接口：内部低电平复位。
    input wire sample_valid, // 声明接口：新样本有效标志。
    input wire [1:0] sample_gesture, // 声明接口：这帧手势。
    output reg stable_valid, // 声明接口：连续帧确认标志。
    output reg [1:0] stable_gesture // 声明接口：确认后的手势。
); // 结束输入输出接口定义。
    reg [1:0] last_gesture; // 声明2位寄存器：上一帧手势。
    reg [7:0] same_count; // 声明8位寄存器：同类连续帧数。
    always @(posedge clk) begin // 每个时钟上升沿更新下面的数据。
        if (!rst_n) begin // 判断内部低电平复位为0；满足条件才进入后面的步骤。
            stable_valid <= 0; // 连续帧确认标志设为0。
            stable_gesture <= 3; // 确认后的手势设为3。
            last_gesture <= 3; // 上一帧手势设为3。
            same_count <= 0; // 同类连续帧数设为0。
        end // 结束当前代码块。
        else if (sample_valid) begin // 判断新样本有效标志；满足条件才进入后面的步骤。
            stable_gesture <= sample_gesture; // 确认后的手势设为这帧手势。
            last_gesture   <= sample_gesture; // 上一帧手势设为这帧手势。
            if (sample_gesture == 3) begin // 判断这帧手势等于3；满足条件才进入后面的步骤。
                stable_valid <= 0; // 连续帧确认标志设为0。
                same_count   <= 0; // 同类连续帧数设为0。
            end // 结束当前代码块。
            else if (sample_gesture != last_gesture) begin // 判断这帧手势不等于上一帧手势；满足条件才进入后面的步骤。
                same_count   <= 1; // 同类连续帧数设为1。
                stable_valid <= (REQUIRED_FRAMES == 1); // 连续帧确认标志设为(连续确认帧数 == 1)。
            end // 结束当前代码块。
            else if (same_count < REQUIRED_FRAMES) begin // 判断同类连续帧数 < 连续确认帧数；满足条件才进入后面的步骤。
                same_count   <= same_count + 1'b1; // 同类连续帧数设为同类连续帧数加1。
                stable_valid <= (same_count + 1 >= REQUIRED_FRAMES); // 连续帧确认标志设为(同类连续帧数 + 1 >= 连续确认帧数)。
            end // 结束当前代码块。
            else stable_valid <= 1; // 否则，连续帧确认标志设为1。
        end // 结束当前代码块。
    end // 结束当前代码块。
endmodule // 本模块结束。
