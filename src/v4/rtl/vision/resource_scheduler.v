// 模块：阶段切换（resource_scheduler）
// 作用：空闲时开手势接口，观察阶段开光流接口。
// 输入：机械臂忙、观察阶段标志。
// 输出：手势使能cnn_enable、光流使能lk_enable。
// 注意：这里只切换使能，没有实现CNN/LK的DSP共享。
// 阅读：每行右侧说明这行代码在做什么。

`timescale 1ns / 1ps // 仿真时间单位1ns，最小精度1ps；不产生硬件时钟。
module resource_scheduler ( // 定义这个模块；括号中写参数或输入输出接口。
    input  wire robot_busy, // 声明接口：机械臂流程忙标志。
    input  wire monitor_phase, // 声明接口：观察阶段标志。
    output wire cnn_enable, // 声明接口：手势接口使能。
    output wire lk_enable // 声明接口：光流接口使能。
); // 结束输入输出接口定义。
    assign cnn_enable = !robot_busy; // 抓取流程不忙时，允许识别手势。
    assign lk_enable  = monitor_phase; // 只在观察阶段使能光流接口。
endmodule // 本模块结束。
