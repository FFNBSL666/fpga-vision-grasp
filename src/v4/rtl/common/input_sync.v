// 模块：外部信号同步（input_sync）
// 作用：把外部开关信号同步到FPGA时钟。
// 输入：时钟、复位、外部开关。
// 输出：同步后的开关状态。
// 注意：只同步，不做机械按键消抖。
// 阅读：每行右侧说明这行代码在做什么。

`timescale 1ns / 1ps // 仿真时间单位1ns，最小精度1ps；不产生硬件时钟。
module input_sync ( // 定义这个模块；括号中写参数或输入输出接口。
    input  wire clk, // 声明接口：时钟。
    input  wire rst_n, // 声明接口：内部低电平复位。
    input  wire signal_in, // 声明接口：外部开关。
    output wire signal_out // 声明接口：同步后的开关。
); // 结束输入输出接口定义。
    (* ASYNC_REG = "TRUE" *) reg [1:0] ff; // 告诉Vivado这是同步寄存器，并声明它保存的两级数据。
    always @(posedge clk) begin // 每个时钟上升沿更新下面的数据。
        if (!rst_n) ff <= 2'b00; // 复位时两个同步寄存器清零。
        else ff <= {ff[0], signal_in}; // 先采入外部信号，再延后一级输出。
    end // 结束当前代码块。
    assign signal_out = ff[1]; // 把第二级结果交给后续模块。
endmodule // 本模块结束。
