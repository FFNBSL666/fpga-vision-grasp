// 模块：相机数据包解析（camera_frame_rx）
// 作用：寻找帧头、核对尺寸、接收像素，最后检查SUM8。
// 输入：相机串口收到的字节和错误标志。
// 输出：像素与坐标、帧开始、有效帧和坏帧脉冲。
// 注意：格式是自定义教学协议，相机Python需要按约定发送。
// 阅读：每行右侧说明这行代码在做什么。

`timescale 1ns / 1ps // 仿真时间单位1ns，最小精度1ps；不产生硬件时钟。
module camera_frame_rx #( // 定义这个模块；括号中写参数或输入输出接口。
    parameter integer IMAGE_W = 80, // 设置图像宽度；这些常量在综合前确定。
    parameter integer IMAGE_H = 60, // 设置图像高度；这些常量在综合前确定。
    parameter integer CLK_HZ = 50000000, // 设置时钟频率；这些常量在综合前确定。
    parameter integer TIMEOUT_MS = 100 // 设置字节间隔超时毫秒数；这些常量在综合前确定。
) ( // 结束参数列表，开始写输入输出接口。
    input wire clk, // 声明接口：时钟。
    input wire rst_n, // 声明接口：内部低电平复位。
    input wire [7:0] byte_data, // 声明接口：字节内容。
    input wire byte_valid, // 声明接口：字节有效标志。
    input wire byte_error, // 声明接口：字节错误。
    output reg frame_start, // 声明接口：开始一帧的脉冲。
    output reg pixel_valid, // 声明接口：像素有效标志。
    output reg [7:0] pixel, // 声明接口：灰度像素。
    output reg [7:0] x, // 声明接口：当前像素横坐标。
    output reg [7:0] y, // 声明接口：当前像素纵坐标。
    output reg frame_good, // 声明接口：完整好帧脉冲。
    output reg frame_bad, // 声明接口：坏帧脉冲。
    output reg [7:0] frame_id // 声明接口：帧号。
); // 结束输入输出接口定义。
    localparam [2:0] HEAD1 = 0; // 给这些步骤编号：HEAD1为等待A5帧头。
    localparam [2:0] HEAD2 = 1; // 给这些步骤编号：HEAD2为等待5A帧头。
    localparam [2:0] SEQ = 2; // 给这些步骤编号：SEQ为读取帧号。
    localparam [2:0] WIDTH = 3; // 给这些步骤编号：WIDTH为等待宽度的状态。
    localparam [2:0] HEIGHT = 4; // 给这些步骤编号：HEIGHT为读取高度。
    localparam [2:0] PIXELS = 5; // 给这些步骤编号：PIXELS为接收像素。
    localparam [2:0] CHECK = 6; // 给这些步骤编号：CHECK为检查校验。
    localparam integer MAX_GAP = (CLK_HZ / 1000) * TIMEOUT_MS; // 设置字节间隔上限；这些常量在综合前确定。
    reg [ 2:0] state; // 声明寄存器：当前状态。
    reg [ 7:0] sum8; // 声明寄存器：低8位校验和。
    reg [ 7:0] col; // 声明寄存器：当前列号。
    reg [ 7:0] row; // 声明寄存器：当前行号。
    reg [31:0] gap; // 声明32位寄存器：字节间隔计数。
    always @(posedge clk) begin // 每个时钟上升沿更新下面的数据。
        if (!rst_n) begin // 判断内部低电平复位为0；满足条件才进入后面的步骤。
            state <= HEAD1; // 当前状态设为等待A5帧头。
            frame_start <= 0; // 开始一帧的脉冲设为0。
            pixel_valid <= 0; // 像素有效标志设为0。
            frame_good <= 0; // 完整好帧脉冲设为0。
            frame_bad <= 0; // 坏帧脉冲设为0。
            frame_id <= 0; // 帧号设为0。
            pixel <= 0; // 灰度像素设为0。
            x <= 0; // 当前像素横坐标设为0。
            y <= 0; // 当前像素纵坐标设为0。
            sum8 <= 0; // 低8位校验和设为0。
            col <= 0; // 当前列号设为0。
            row <= 0; // 当前行号设为0。
            gap <= 0; // 字节间隔计数设为0。
        end // 结束当前代码块。
        else begin // 前一个条件不满足时，执行下面这一块。
            frame_start <= 0; // 开始一帧的脉冲设为0。
            pixel_valid <= 0; // 像素有效标志设为0。
            frame_good  <= 0; // 完整好帧脉冲设为0。
            frame_bad   <= 0; // 坏帧脉冲设为0。
            if (byte_error || (state != HEAD1 && gap == MAX_GAP - 1)) begin // 判断字节错误 || (当前状态 != 等待A5帧头 && 字节间隔计数 == 字节间隔上限 - 1)；满足条件才进入后面的步骤。
                state <= HEAD1; // 当前状态设为等待A5帧头。
                gap <= 0; // 字节间隔计数设为0。
                frame_bad <= 1; // 坏帧脉冲设为1。
            end // 结束当前代码块。
            else if (byte_valid) begin // 判断字节有效标志；满足条件才进入后面的步骤。
                gap <= 0; // 字节间隔计数设为0。
                case (state) // 根据当前状态，选择下面的分支。
                    HEAD1: if (byte_data == 8'hA5) state <= HEAD2; // 判断字节内容 == 8'hA5；当前状态设为等待5A帧头。
                    HEAD2: // 进入等待5A帧头分支。
                    if (byte_data == 8'h5A) state <= SEQ; // 判断字节内容 == 8'h5A；当前状态设为读取帧号。
                    else if (byte_data != 8'hA5) state <= HEAD1; // 判断字节内容 != 8'hA5；当前状态设为等待A5帧头。
                    SEQ: begin // 处于读取帧号时执行；开始该步骤。
                        frame_id <= byte_data; // 帧号设为字节内容。
                        sum8 <= byte_data; // 低8位校验和设为字节内容。
                        state <= WIDTH; // 当前状态设为读取宽度。
                    end // 结束当前代码块。
                    WIDTH: // 进入等待宽度的状态分支。
                    if (byte_data == IMAGE_W) begin // 判断字节内容等于图像宽度；满足条件才进入后面的步骤。
                        sum8  <= sum8 + byte_data; // 低8位校验和设为低8位校验和 + 字节内容。
                        state <= HEIGHT; // 当前状态设为读取高度。
                    end // 结束当前代码块。
                    else begin // 前一个条件不满足时，执行下面这一块。
                        state <= HEAD1; // 当前状态设为等待A5帧头。
                        frame_bad <= 1; // 坏帧脉冲设为1。
                    end // 结束当前代码块。
                    HEIGHT: // 进入读取高度分支。
                    if (byte_data == IMAGE_H) begin // 判断字节内容等于图像高度；满足条件才进入后面的步骤。
                        sum8 <= sum8 + byte_data; // 低8位校验和设为低8位校验和 + 字节内容。
                        col <= 0; // 当前列号设为0。
                        row <= 0; // 当前行号设为0。
                        state <= PIXELS; // 当前状态设为接收像素。
                        frame_start <= 1; // 开始一帧的脉冲设为1。
                    end // 结束当前代码块。
                    else begin // 前一个条件不满足时，执行下面这一块。
                        state <= HEAD1; // 当前状态设为等待A5帧头。
                        frame_bad <= 1; // 坏帧脉冲设为1。
                    end // 结束当前代码块。
                    PIXELS: begin // 处于接收像素时执行；开始该步骤。
                        pixel <= byte_data; // 灰度像素设为字节内容。
                        pixel_valid <= 1; // 像素有效标志设为1。
                        x <= col; // 当前像素横坐标设为当前列号。
                        y <= row; // 当前像素纵坐标设为当前行号。
                        sum8 <= sum8 + byte_data; // 低8位校验和设为低8位校验和 + 字节内容。
                        if (col == IMAGE_W - 1) begin // 判断当前列号 == 图像宽度 - 1；满足条件才进入后面的步骤。
                            col <= 0; // 当前列号设为0。
                            if (row == IMAGE_H - 1) state <= CHECK; // 判断当前行号 == 图像高度 - 1；当前状态设为检查校验。
                            else row <= row + 1'b1; // 否则，当前行号设为当前行号加1。
                        end // 结束当前代码块。
                        else col <= col + 1'b1; // 否则，当前列号设为当前列号加1。
                    end // 结束当前代码块。
                    CHECK: begin // 处于检查校验时执行；开始该步骤。
                        state <= HEAD1; // 当前状态设为等待A5帧头。
                        if (byte_data == sum8) frame_good <= 1; // 判断字节内容等于低8位校验和；完整好帧脉冲设为1。
                        else frame_bad <= 1; // 否则，坏帧脉冲设为1。
                    end // 结束当前代码块。
                    default: state <= HEAD1; // 未匹配任何分支时，回到预设状态。
                endcase // 结束状态或编号分支。
            end // 结束当前代码块。
            else if (state != HEAD1) gap <= gap + 1'b1; // 判断当前状态不等于等待A5帧头；字节间隔计数设为字节间隔计数加1。
            else gap <= 0; // 否则，字节间隔计数设为0。
        end // 结束当前代码块。
    end // 结束当前代码块。
endmodule // 本模块结束。
