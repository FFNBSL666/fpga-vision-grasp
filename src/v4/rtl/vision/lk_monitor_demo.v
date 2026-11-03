// 模块：滑移接口演示（lk_monitor_demo）
// 作用：保存一帧中心坐标，再检查后续中心是否移动过大。
// 输入：观察使能、有效物体中心。
// 输出：是否观察过两帧、疑似滑移报警。
// 注意：不是LK光流，也不能测力或替代触觉传感器。
// 阅读：每行右侧说明这行代码在做什么。

`timescale 1ns / 1ps // 仿真时间单位1ns，最小精度1ps；不产生硬件时钟。
module lk_monitor_demo #( // 定义这个模块；括号中写参数或输入输出接口。
    parameter integer MOVE_LIMIT = 4 // 设置移动报警阈值；这些常量在综合前确定。
) ( // 结束参数列表，开始写输入输出接口。
    input wire clk, // 声明接口：时钟。
    input wire rst_n, // 声明接口：内部低电平复位。
    input wire enable, // 声明接口：模块使能。
    input wire sample_valid, // 声明接口：新样本有效标志。
    input wire object_found, // 声明接口：找到物体标志。
    input wire [7:0] center_x, // 声明接口：物体中心横坐标。
    input wire [7:0] center_y, // 声明接口：物体中心纵坐标。
    output reg checked, // 声明接口：已经比较过两帧的标志。
    output reg slip_alarm // 声明接口：疑似滑移报警。
); // 结束输入输出接口定义。
    reg has_reference; // 声明寄存器：已有参考坐标标志。
    reg [7:0] ref_x; // 声明8位寄存器：参考中心横坐标。
    reg [7:0] ref_y; // 声明8位寄存器：参考中心纵坐标。
    wire [7:0] dx = center_x >= ref_x ? center_x - ref_x : ref_x - center_x; // 横向移动距离设为物体中心横坐标 >= 参考中心横坐标 ? 物体中心横坐标 - 参考中心横坐标 : 参考中心横坐标 - 物体中心横坐标。
    wire [7:0] dy = center_y >= ref_y ? center_y - ref_y : ref_y - center_y; // 纵向移动距离设为物体中心纵坐标 >= 参考中心纵坐标 ? 物体中心纵坐标 - 参考中心纵坐标 : 参考中心纵坐标 - 物体中心纵坐标。
    always @(posedge clk) begin // 每个时钟上升沿更新下面的数据。
        if (!rst_n) begin // 判断内部低电平复位为0；满足条件才进入后面的步骤。
            has_reference <= 0; // 已有参考坐标标志设为0。
            ref_x <= 0; // 参考中心横坐标设为0。
            ref_y <= 0; // 参考中心纵坐标设为0。
            checked <= 0; // 已经比较过两帧的标志设为0。
            slip_alarm <= 0; // 疑似滑移报警设为0。
        end // 结束当前代码块。
        else if (!enable) begin // 判断模块使能为0；满足条件才进入后面的步骤。
            has_reference <= 0; // 已有参考坐标标志设为0。
            checked <= 0; // 已经比较过两帧的标志设为0。
            slip_alarm <= 0; // 疑似滑移报警设为0。
        end // 结束当前代码块。
        else if (sample_valid) begin // 判断新样本有效标志；满足条件才进入后面的步骤。
            if (!object_found) slip_alarm <= 1; // 判断找到物体标志为0；疑似滑移报警设为1。
            else if (!has_reference) begin // 判断已有参考坐标标志为0；满足条件才进入后面的步骤。
                ref_x <= center_x; // 参考中心横坐标设为物体中心横坐标。
                ref_y <= center_y; // 参考中心纵坐标设为物体中心纵坐标。
                has_reference <= 1; // 已有参考坐标标志设为1。
            end // 结束当前代码块。
            else begin // 前一个条件不满足时，执行下面这一块。
                checked <= 1; // 已经比较过两帧的标志设为1。
                if (dx > MOVE_LIMIT || dy > MOVE_LIMIT) slip_alarm <= 1; // 判断横向移动距离 > 移动报警阈值 || 纵向移动距离 > 移动报警阈值；疑似滑移报警设为1。
            end // 结束当前代码块。
        end // 结束当前代码块。
    end // 结束当前代码块。
endmodule // 本模块结束。
