// 模块：串口接收（uart_rx）
// 作用：把串行输入的0和1，恢复成一个8位字节。
// 输入：时钟、复位、串口rx。
// 输出：接收字节data、有效valid、停止位错误frame_error。
// 注意：采用8N1：8个数据位、无奇偶校验、1个停止位。
// 阅读：每行右侧说明这行代码在做什么。

`timescale 1ns / 1ps // 仿真时间单位1ns，最小精度1ps；不产生硬件时钟。
module uart_rx #( // 定义这个模块；括号中写参数或输入输出接口。
    parameter integer CLK_HZ = 50000000, // 设置时钟频率；这些常量在综合前确定。
    parameter integer BAUD   = 115200 // 设置串口波特率；这些常量在综合前确定。
) ( // 结束参数列表，开始写输入输出接口。
    input wire clk, // 声明接口：时钟。
    input wire rst_n, // 声明接口：内部低电平复位。
    input wire rx, // 声明接口：串口输入。
    output reg [7:0] data, // 声明接口：字节数据。
    output reg valid, // 声明接口：数据有效标志。
    output reg frame_error // 声明接口：停止位错误脉冲。
); // 结束输入输出接口定义。
    localparam integer DIV = CLK_HZ / BAUD; // 设置每个串口位所需时钟数；这些常量在综合前确定。
    localparam [1:0] IDLE = 0; // 给这些步骤编号：IDLE为等待状态。
    localparam [1:0] START = 1; // 给这些步骤编号：START为起始位状态。
    localparam [1:0] BITS = 2; // 给这些步骤编号：BITS为数据位状态。
    localparam [1:0] STOP = 3; // 给这些步骤编号：STOP为停止位状态。
    (* ASYNC_REG = "TRUE" *)reg [ 1:0] rx_ff; // 告诉Vivado这是同步寄存器，并声明它保存的两级数据。
    reg [ 1:0] state; // 声明寄存器：当前状态。
    reg [31:0] count; // 声明32位寄存器：当前位等待计数。
    reg [ 2:0] bit_index; // 声明寄存器：当前数据位编号。
    reg [ 7:0] shift; // 声明寄存器：收到的8位数据。
    always @(posedge clk) begin // 每个时钟上升沿更新下面的数据。
        if (!rst_n) rx_ff <= 2'b11; // 复位时接收线按串口空闲高电平处理。
        else rx_ff <= {rx_ff[0], rx}; // 串口输入经过两级同步再参与判断。
    end // 结束当前代码块。
    always @(posedge clk) begin // 每个时钟上升沿更新下面的数据。
        if (!rst_n) begin // 判断内部低电平复位为0；满足条件才进入后面的步骤。
            state <= IDLE; // 当前状态设为等待状态。
            count <= 0; // 当前位等待计数设为0。
            bit_index <= 0; // 当前数据位编号设为0。
            shift <= 0; // 收到的8位数据设为0。
            data <= 0; // 字节数据设为0。
            valid <= 0; // 数据有效标志设为0。
            frame_error <= 0; // 停止位错误脉冲设为0。
        end // 结束当前代码块。
        else begin // 前一个条件不满足时，执行下面这一块。
            valid <= 0; // 数据有效标志设为0。
            frame_error <= 0; // 停止位错误脉冲设为0。
            case (state) // 根据当前状态，选择下面的分支。
                IDLE: begin // 处于等待状态时执行；开始该步骤。
                    count <= 0; // 当前位等待计数设为0。
                    if (!rx_ff[1]) state <= START; // 判断!两级串口输入同步寄存器[1]；当前状态设为起始位状态。
                end // 结束当前代码块。
                START: // 进入起始位状态分支。
                if (count == DIV / 2 - 1) begin // 判断当前位等待计数 == 每个串口位所需时钟数 / 2 - 1；满足条件才进入后面的步骤。
                    count <= 0; // 当前位等待计数设为0。
                    if (!rx_ff[1]) begin // 判断!两级串口输入同步寄存器[1]；满足条件才进入后面的步骤。
                        state <= BITS; // 当前状态设为数据位状态。
                        bit_index <= 0; // 当前数据位编号设为0。
                    end // 结束当前代码块。
                    else state <= IDLE; // 否则，当前状态设为等待状态。
                end // 结束当前代码块。
                else count <= count + 1'b1; // 还没数够，计数器加1。
                BITS: // 进入数据位状态分支。
                if (count == DIV - 1) begin // 判断当前位等待计数 == 每个串口位所需时钟数 - 1；满足条件才进入后面的步骤。
                    count <= 0; // 当前位等待计数设为0。
                    shift[bit_index] <= rx_ff[1]; // 把采样电平放入当前数据位。
                    if (bit_index == 7) state <= STOP; // 判断当前数据位编号等于7；当前状态设为停止位状态。
                    else bit_index <= bit_index + 1'b1; // 还没到最后一位，继续处理下一位。
                end // 结束当前代码块。
                else count <= count + 1'b1; // 还没数够，计数器加1。
                STOP: // 进入停止位状态分支。
                if (count == DIV - 1) begin // 判断当前位等待计数 == 每个串口位所需时钟数 - 1；满足条件才进入后面的步骤。
                    count <= 0; // 当前位等待计数设为0。
                    state <= IDLE; // 当前状态设为等待状态。
                    if (rx_ff[1]) begin // 判断两级串口输入同步寄存器[1]；满足条件才进入后面的步骤。
                        data  <= shift; // 字节数据设为收到的8位数据。
                        valid <= 1; // 数据有效标志设为1。
                    end // 结束当前代码块。
                    else frame_error <= 1; // 否则，停止位错误脉冲设为1。
                end // 结束当前代码块。
                else count <= count + 1'b1; // 还没数够，计数器加1。
                default: state <= IDLE; // 未匹配任何分支时，回到预设状态。
            endcase // 结束状态或编号分支。
        end // 结束当前代码块。
    end // 结束当前代码块。
endmodule // 本模块结束。
