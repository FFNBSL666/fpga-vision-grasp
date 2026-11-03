// 模块：复位同步（reset_sync）
// 作用：复位立即生效，恢复时等待两个时钟，使后续模块稳定启动。
// 输入：时钟clk、外部低电平复位rst_n_in。
// 输出：内部低电平复位rst_n。
// 注意：两个触发器不是两秒，而是两个时钟。
// 阅读：每行右侧说明这行代码在做什么。

`timescale 1ns / 1ps // 仿真时间单位1ns，最小精度1ps；不产生硬件时钟。
module reset_sync ( // 定义这个模块；括号中写参数或输入输出接口。
    input  wire clk, // 声明接口：时钟。
    input  wire rst_n_in, // 声明接口：外部低电平复位。
    output wire rst_n // 声明接口：内部低电平复位。
); // 结束输入输出接口定义。
    (* ASYNC_REG = "TRUE" *) reg [1:0] sync_ff; // 告诉Vivado这是同步寄存器，并声明它保存的两级数据。
    always @(posedge clk or negedge rst_n_in) begin // 每个时钟上升沿运行；外部复位拉低时也立即运行。
        if (!rst_n_in) sync_ff <= 2'b00; // 判断外部低电平复位为0；两级复位同步寄存器设为两个0。
        else sync_ff <= {sync_ff[0], 1'b1}; // 每个时钟向后移入1，两个时钟后内部复位恢复。
    end // 结束当前代码块。
    assign rst_n = sync_ff[1]; // 使用第二级的输出作为内部复位。
endmodule // 本模块结束。
