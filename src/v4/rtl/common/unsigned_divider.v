// 模块：分步除法（unsigned_divider）
// 作用：每个时钟算一位商，用较小的电路计算中心坐标。
// 输入：启动start、被除数numerator、除数denominator。
// 输出：商quotient、忙busy、完成done。
// 注意：默认24位，需要24个时钟；除数0返回0。
// 阅读：每行右侧说明这行代码在做什么。

`timescale 1ns / 1ps // 仿真时间单位1ns，最小精度1ps；不产生硬件时钟。
module unsigned_divider #( // 定义这个模块；括号中写参数或输入输出接口。
    parameter integer WIDTH = 24 // 设置除法数据位数；这些常量在综合前确定。
) ( // 结束参数列表，开始写输入输出接口。
    input wire clk, // 声明接口：时钟。
    input wire rst_n, // 声明接口：内部低电平复位。
    input wire start, // 声明接口：启动脉冲。
    input wire [WIDTH-1:0] numerator, // 声明接口：被除数。
    input wire [WIDTH-1:0] denominator, // 声明接口：除数。
    output reg [WIDTH-1:0] quotient, // 声明接口：除法结果。
    output reg busy, // 声明接口：忙标志。
    output reg done // 声明接口：完成脉冲。
); // 结束输入输出接口定义。
    reg [WIDTH-1:0] work_q; // 声明寄存器：正在计算的商。
    reg [WIDTH-1:0] divisor; // 声明寄存器：保存的除数。
    reg [WIDTH:0] remainder; // 声明寄存器：余数。
    reg [7:0] count; // 声明8位寄存器：已经计算的商位数。
    wire [WIDTH:0] shifted = {remainder[WIDTH-1:0], work_q[WIDTH-1]}; // 左移后的余数设为{余数[除法数据位数-1:0], 正在计算的商[除法数据位数-1]}。
    always @(posedge clk) begin // 每个时钟上升沿更新下面的数据。
        if (!rst_n) begin // 判断内部低电平复位为0；满足条件才进入后面的步骤。
            quotient <= 0; // 除法结果设为0。
            busy <= 0; // 忙标志设为0。
            done <= 0; // 完成脉冲设为0。
            work_q <= 0; // 正在计算的商设为0。
            divisor <= 0; // 保存的除数设为0。
            remainder <= 0; // 余数设为0。
            count <= 0; // 已经计算的商位数设为0。
        end // 结束当前代码块。
        else begin // 前一个条件不满足时，执行下面这一块。
            done <= 0; // 完成脉冲设为0。
            if (start && !busy) begin // 判断启动脉冲 && !忙标志；满足条件才进入后面的步骤。
                if (denominator == 0) begin // 判断除数等于0；满足条件才进入后面的步骤。
                    quotient <= 0; // 除法结果设为0。
                    done <= 1; // 完成脉冲设为1。
                end // 结束当前代码块。
                else begin // 前一个条件不满足时，执行下面这一块。
                    work_q <= numerator; // 正在计算的商设为被除数。
                    divisor <= denominator; // 保存的除数设为除数。
                    remainder <= 0; // 余数设为0。
                    count <= 0; // 已经计算的商位数设为0。
                    busy <= 1; // 忙标志设为1。
                end // 结束当前代码块。
            end // 结束当前代码块。
            else if (busy) begin // 判断忙标志；满足条件才进入后面的步骤。
                if (shifted >= {1'b0, divisor}) begin // 判断左移后的余数 >= {1'b0, 保存的除数}；满足条件才进入后面的步骤。
                    remainder <= shifted - {1'b0, divisor}; // 余数设为左移后的余数 - {1'b0, 保存的除数}。
                    work_q <= {work_q[WIDTH-2:0], 1'b1}; // 正在计算的商设为{正在计算的商[除法数据位数-2:0], 1'b1}。
                end // 结束当前代码块。
                else begin // 前一个条件不满足时，执行下面这一块。
                    remainder <= shifted; // 余数设为左移后的余数。
                    work_q <= {work_q[WIDTH-2:0], 1'b0}; // 正在计算的商设为{正在计算的商[除法数据位数-2:0], 1'b0}。
                end // 结束当前代码块。
                if (count == WIDTH - 1) begin // 判断已经计算的商位数 == 除法数据位数 - 1；满足条件才进入后面的步骤。
                    quotient <= {work_q[WIDTH-2:0], (shifted >= {1'b0, divisor})}; // 除法结果设为{正在计算的商[除法数据位数-2:0], (左移后的余数 >= {1'b0, 保存的除数})}。
                    busy <= 0; // 忙标志设为0。
                    done <= 1; // 完成脉冲设为1。
                end // 结束当前代码块。
                else count <= count + 1'b1; // 还没数够，计数器加1。
            end // 结束当前代码块。
        end // 结束当前代码块。
    end // 结束当前代码块。
endmodule // 本模块结束。
