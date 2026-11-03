// 模块：新帧有效门（frame_fresh_gate）
// 作用：只允许空闲阶段收到的完整新帧形成抓取请求。
// 输入：时钟、复位、帧开始、好帧、机械臂忙和故障。
// 输出：新帧允许标志。
// 注意：防止旧手势和旧目标拼成新的抓取请求。
// 阅读：每行右侧说明这行代码在做什么。

`timescale 1ns / 1ps // 仿真时间单位1ns，最小精度1ps；不产生硬件时钟。
module frame_fresh_gate ( // 定义这个模块；括号中写参数或输入输出接口。
    input  wire clk, // 声明接口：时钟。
    input  wire rst_n, // 声明接口：内部低电平复位。
    input  wire frame_start, // 声明接口：开始一帧的脉冲。
    input  wire frame_good, // 声明接口：完整好帧脉冲。
    input  wire robot_busy, // 声明接口：机械臂流程忙标志。
    input  wire fault, // 声明接口：故障锁。
    output reg  frame_allowed // 声明接口：当前有效新帧允许标志。
); // 结束输入输出接口定义。
    always @(posedge clk) begin // 每个时钟上升沿更新下面的数据。
        if (!rst_n) frame_allowed <= 1'b0; // 判断内部低电平复位为0；当前有效新帧允许标志设为0。
        else if (robot_busy || fault || frame_start) frame_allowed <= 1'b0; // 判断机械臂流程忙标志 || 故障锁 || 开始一帧的脉冲；当前有效新帧允许标志设为0。
        else if (frame_good) frame_allowed <= 1'b1; // 判断完整好帧脉冲；当前有效新帧允许标志设为1。
    end // 结束当前代码块。
endmodule // 本模块结束。
