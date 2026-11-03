// 模块：区域匹配（target_mapper）
// 作用：把物体中心分到左、中、右区，检查是否与手势相同。
// 输入：物体中心、物体存在标志、稳定手势。
// 输出：目标区域和抓取请求。
// 注意：只处理三个固定区域，不是任意三维定位。
// 阅读：每行右侧说明这行代码在做什么。

`timescale 1ns / 1ps // 仿真时间单位1ns，最小精度1ps；不产生硬件时钟。
module target_mapper #( // 定义这个模块；括号中写参数或输入输出接口。
    parameter integer IMAGE_W = 80 // 设置图像宽度；这些常量在综合前确定。
) ( // 结束参数列表，开始写输入输出接口。
    input wire object_valid, // 声明接口：物体结果有效标志。
    input wire object_found, // 声明接口：找到物体标志。
    input wire [7:0] center_x, // 声明接口：物体中心横坐标。
    input wire gesture_valid, // 声明接口：手势确认标志。
    input wire [1:0] gesture, // 声明接口：手势类别。
    output wire request_valid, // 声明接口：抓取请求。
    output wire [1:0] target_region // 声明接口：物体所在区域。
); // 结束输入输出接口定义。
    assign target_region = center_x<IMAGE_W/3 ? 2'd0 : // 根据右边条件，产生左边的连接信号。
                           center_x<(IMAGE_W*2)/3 ? 2'd1 : 2'd2; // 接续上一行：物体中心横坐标<(图像宽度*2)/3 ? 2'd1 : 2'd2。
    assign request_valid = object_valid && object_found && gesture_valid && // 根据右边条件，产生左边的连接信号。
                           gesture==target_region && gesture!=2'd3; // 接续上一行：手势类别==物体所在区域 && 手势类别!=2'd3。
endmodule // 本模块结束。
