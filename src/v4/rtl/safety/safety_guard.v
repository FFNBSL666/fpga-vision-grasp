// 模块：故障保护（safety_guard）
// 作用：发现急停、断帧、连续坏帧、指令错误或观察异常就锁住故障。
// 输入：使能、急停、图像状态、舵机状态、计时。
// 输出：fault故障锁和fault_code原因。
// 注意：停止发新指令不等于真实舵机立即停机。
// 阅读：每行右侧说明这行代码在做什么。

`timescale 1ns / 1ps // 仿真时间单位1ns，最小精度1ps；不产生硬件时钟。
module safety_guard #( // 定义这个模块；括号中写参数或输入输出接口。
    parameter integer FRAME_TIMEOUT_MS = 5000, // 设置图像超时毫秒数；这些常量在综合前确定。
    parameter integer MAX_BAD_FRAMES   = 3, // 设置连续坏帧上限；这些常量在综合前确定。
    parameter integer POSE_TIMEOUT_MS  = 10000 // 设置动作发送超时毫秒数；这些常量在综合前确定。
) ( // 结束参数列表，开始写输入输出接口。
    input wire clk, // 声明接口：时钟。
    input wire rst_n, // 声明接口：内部低电平复位。
    input wire tick, // 声明接口：毫秒脉冲。
    input wire armed, // 声明接口：内部流程允许标志。
    input wire estop, // 声明接口：急停信号。
    input wire frame_good, // 声明接口：完整好帧脉冲。
    input wire frame_bad, // 声明接口：坏帧脉冲。
    input wire command_error, // 声明接口：指令错误。
    input wire slip_alarm, // 声明接口：疑似滑移报警。
    input wire monitor_timeout, // 声明接口：观察不足超时脉冲。
    input wire pose_busy, // 声明接口：六关节发送忙标志。
    output reg fault, // 声明接口：故障锁。
    output reg [3:0] fault_code // 声明接口：故障原因编号。
); // 结束输入输出接口定义。
    reg [31:0] frame_age; // 声明32位寄存器：距离好帧的毫秒数。
    reg [31:0] pose_age; // 声明32位寄存器：动作发送持续毫秒数。
    reg [ 7:0] bad_count; // 声明寄存器：连续坏帧数。
    always @(posedge clk) begin // 每个时钟上升沿更新下面的数据。
        if (!rst_n) begin // 判断内部低电平复位为0；满足条件才进入后面的步骤。
            fault <= 0; // 故障锁设为0。
            fault_code <= 0; // 故障原因编号设为0。
            frame_age <= 0; // 距离好帧的毫秒数设为0。
            pose_age <= 0; // 动作发送持续毫秒数设为0。
            bad_count <= 0; // 连续坏帧数设为0。
        end // 结束当前代码块。
        else begin // 前一个条件不满足时，执行下面这一块。
            if (!armed || frame_good) frame_age <= 0; // 判断!内部流程允许标志 || 完整好帧脉冲；距离好帧的毫秒数设为0。
            else if (tick && frame_age < FRAME_TIMEOUT_MS) frame_age <= frame_age + 1'b1; // 判断毫秒脉冲 && 距离好帧的毫秒数 < 图像超时毫秒数；距离好帧的毫秒数设为距离好帧的毫秒数加1。
            if (!pose_busy) pose_age <= 0; // 判断六关节发送忙标志为0；动作发送持续毫秒数设为0。
            else if (tick && pose_age < POSE_TIMEOUT_MS) pose_age <= pose_age + 1'b1; // 判断毫秒脉冲 && 动作发送持续毫秒数 < 动作发送超时毫秒数；动作发送持续毫秒数设为动作发送持续毫秒数加1。
            if (frame_good || !armed) bad_count <= 0; // 判断完整好帧脉冲 || !内部流程允许标志；连续坏帧数设为0。
            else if (frame_bad && bad_count < MAX_BAD_FRAMES) // 条件满足时执行；满足条件才进入后面的步骤。
                bad_count <= bad_count + 1'b1; // 连续坏帧数设为连续坏帧数加1。
            if (!fault) begin // 判断故障锁为0；满足条件才进入后面的步骤。
                if (estop) begin // 判断急停信号；满足条件才进入后面的步骤。
                    fault <= 1; // 故障锁设为1。
                    fault_code <= 1; // 故障原因编号设为1。
                end // 结束当前代码块。
                else if (armed && frame_age >= FRAME_TIMEOUT_MS) begin // 判断内部流程允许标志 && 距离好帧的毫秒数 >= 图像超时毫秒数；满足条件才进入后面的步骤。
                    fault <= 1; // 故障锁设为1。
                    fault_code <= 2; // 故障原因编号设为2。
                end // 结束当前代码块。
                else if (armed && bad_count >= MAX_BAD_FRAMES) begin // 判断内部流程允许标志 && 连续坏帧数 >= 连续坏帧上限；满足条件才进入后面的步骤。
                    fault <= 1; // 故障锁设为1。
                    fault_code <= 3; // 故障原因编号设为3。
                end // 结束当前代码块。
                else if (command_error) begin // 判断指令错误；满足条件才进入后面的步骤。
                    fault <= 1; // 故障锁设为1。
                    fault_code <= 4; // 故障原因编号设为4。
                end // 结束当前代码块。
                else if (slip_alarm) begin // 判断疑似滑移报警；满足条件才进入后面的步骤。
                    fault <= 1; // 故障锁设为1。
                    fault_code <= 5; // 故障原因编号设为5。
                end // 结束当前代码块。
                else if (monitor_timeout) begin // 判断观察不足超时脉冲；满足条件才进入后面的步骤。
                    fault <= 1; // 故障锁设为1。
                    fault_code <= 6; // 故障原因编号设为6。
                end // 结束当前代码块。
                else if (pose_busy && pose_age >= POSE_TIMEOUT_MS) begin // 判断六关节发送忙标志 && 动作发送持续毫秒数 >= 动作发送超时毫秒数；满足条件才进入后面的步骤。
                    fault <= 1; // 故障锁设为1。
                    fault_code <= 7; // 故障原因编号设为7。
                end // 结束当前代码块。
            end // 结束当前代码块。
        end // 结束当前代码块。
    end // 结束当前代码块。
endmodule // 本模块结束。
