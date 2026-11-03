// 模块：舵机回包演示（servo_feedback_parser）
// 作用：从假定的#001P1500!字符串中提取ID和位置。
// 输入：接收字节、有效脉冲、串口错误。
// 输出：回包ID、位置、有效脉冲和格式错误。
// 注意：回包格式尚未确认，当前只显示，不用于到位控制。
// 阅读：每行右侧说明这行代码在做什么。

`timescale 1ns / 1ps // 仿真时间单位1ns，最小精度1ps；不产生硬件时钟。
module servo_feedback_parser ( // 定义这个模块；括号中写参数或输入输出接口。
    input wire clk, // 声明接口：时钟。
    input wire rst_n, // 声明接口：内部低电平复位。
    input wire [7:0] byte_data, // 声明接口：字节内容。
    input wire byte_valid, // 声明接口：字节有效标志。
    input wire byte_error, // 声明接口：字节错误。
    output reg feedback_valid, // 声明接口：回包有效脉冲。
    output reg parse_error, // 声明接口：回包格式错误脉冲。
    output reg [7:0] feedback_id, // 声明接口：回包舵机ID。
    output reg [15:0] feedback_position // 声明接口：回包位置。
); // 结束输入输出接口定义。
    reg [3:0] index; // 声明4位寄存器：回包字符序号。
    reg [15:0] id_value; // 声明16位寄存器：正在解析的ID。
    reg [15:0] pos_value; // 声明16位寄存器：正在解析的位置。
    wire digit = byte_data >= 8'h30 && byte_data <= 8'h39; // 是十进制数字的标志设为字节内容 >= 8'h30 && 字节内容 <= 8'h39。
    always @(posedge clk) begin // 每个时钟上升沿更新下面的数据。
        if (!rst_n) begin // 判断内部低电平复位为0；满足条件才进入后面的步骤。
            index <= 0; // 回包字符序号设为0。
            id_value <= 0; // 正在解析的ID设为0。
            pos_value <= 0; // 正在解析的位置设为0。
            feedback_valid <= 0; // 回包有效脉冲设为0。
            parse_error <= 0; // 回包格式错误脉冲设为0。
            feedback_id <= 0; // 回包舵机ID设为0。
            feedback_position <= 0; // 回包位置设为0。
        end // 结束当前代码块。
        else begin // 前一个条件不满足时，执行下面这一块。
            feedback_valid <= 0; // 回包有效脉冲设为0。
            parse_error <= 0; // 回包格式错误脉冲设为0。
            if (byte_error) begin // 判断字节错误；满足条件才进入后面的步骤。
                index <= 0; // 回包字符序号设为0。
                parse_error <= 1; // 回包格式错误脉冲设为1。
            end // 结束当前代码块。
            else if (byte_valid) begin // 判断字节有效标志；满足条件才进入后面的步骤。
                if (byte_data == 8'h23) begin // 判断字节内容 == 8'h23；满足条件才进入后面的步骤。
                    index <= 1; // 回包字符序号设为1。
                    id_value <= 0; // 正在解析的ID设为0。
                    pos_value <= 0; // 正在解析的位置设为0。
                end // 结束当前代码块。
                else if (index >= 1 && index <= 3 && digit) begin // 判断回包字符序号 >= 1 && 回包字符序号 <= 3 && 是十进制数字的标志；满足条件才进入后面的步骤。
                    id_value <= id_value * 10 + byte_data - 8'h30; // 正在解析的ID设为正在解析的ID * 10 + 字节内容 - 8'h30。
                    index <= index + 1'b1; // 回包字符序号设为回包字符序号加1。
                end // 结束当前代码块。
                else if (index == 4 && byte_data == 8'h50) index <= 5; // 判断回包字符序号 == 4 && 字节内容 == 8'h50；回包字符序号设为5。
                else if (index >= 5 && index <= 8 && digit) begin // 判断回包字符序号 >= 5 && 回包字符序号 <= 8 && 是十进制数字的标志；满足条件才进入后面的步骤。
                    pos_value <= pos_value * 10 + byte_data - 8'h30; // 正在解析的位置设为正在解析的位置 * 10 + 字节内容 - 8'h30。
                    index <= index + 1'b1; // 回包字符序号设为回包字符序号加1。
                end // 结束当前代码块。
                else if (index == 9 && byte_data == 8'h21 && id_value <= 253) begin // 判断回包字符序号 == 9 && 字节内容 == 8'h21 && 正在解析的ID <= 253；满足条件才进入后面的步骤。
                    feedback_id <= id_value[7:0]; // 回包舵机ID设为正在解析的ID[7:0]。
                    feedback_position <= pos_value; // 回包位置设为正在解析的位置。
                    feedback_valid <= 1; // 回包有效脉冲设为1。
                    index <= 0; // 回包字符序号设为0。
                end // 结束当前代码块。
                else if (index != 0) begin // 判断回包字符序号不等于0；满足条件才进入后面的步骤。
                    index <= 0; // 回包字符序号设为0。
                    parse_error <= 1; // 回包格式错误脉冲设为1。
                end // 结束当前代码块。
            end // 结束当前代码块。
        end // 结束当前代码块。
    end // 结束当前代码块。
endmodule // 本模块结束。
