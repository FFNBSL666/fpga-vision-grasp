// 模块：毫秒计时（tick_ms）
// 作用：数够一毫秒的时钟后，输出一个时钟宽的脉冲。
// 输入：时钟与复位。
// 输出：每毫秒一次的tick脉冲。
// 注意：CLK_HZ要改成开发板真实时钟频率。
// 阅读：每行右侧说明这行代码在做什么。

`timescale 1ns / 1ps // 仿真时间单位1ns，最小精度1ps；不产生硬件时钟。
module tick_ms #( // 定义这个模块；括号中写参数或输入输出接口。
    parameter integer CLK_HZ = 50000000 // 设置时钟频率；这些常量在综合前确定。
) ( // 结束参数列表，开始写输入输出接口。
    input  wire clk, // 声明接口：时钟。
    input  wire rst_n, // 声明接口：内部低电平复位。
    output reg  tick // 声明接口：毫秒脉冲。
); // 结束输入输出接口定义。
    localparam integer DIV = CLK_HZ / 1000; // 设置每毫秒所需时钟数；这些常量在综合前确定。
    reg [31:0] count; // 声明32位寄存器：毫秒计数器。
    always @(posedge clk) begin // 每个时钟上升沿更新下面的数据。
        if (!rst_n) begin // 判断内部低电平复位为0；满足条件才进入后面的步骤。
            count <= 0; // 毫秒计数器设为0。
            tick  <= 0; // 毫秒脉冲设为0。
        end // 结束当前代码块。
        else if (count == DIV - 1) begin // 判断毫秒计数器 == 每毫秒所需时钟数 - 1；满足条件才进入后面的步骤。
            count <= 0; // 毫秒计数器设为0。
            tick  <= 1; // 毫秒脉冲设为1。
        end // 结束当前代码块。
        else begin // 前一个条件不满足时，执行下面这一块。
            count <= count + 1'b1; // 毫秒计数器设为毫秒计数器加1。
            tick  <= 0; // 毫秒脉冲设为0。
        end // 结束当前代码块。
    end // 结束当前代码块。
endmodule // 本模块结束。
