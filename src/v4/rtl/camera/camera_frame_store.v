// 图像块 RAM：单写端口加同步读端口；只复位外围控制，不清空存储阵列。

`timescale 1ns / 1ps
// 独立的简单双端口块 RAM：同步读取，不对存储阵列执行复位。
module camera_frame_store(
    input wire clk, we,
    input wire [14:0] waddr, raddr,
    input wire [15:0] wdata,
    output reg [15:0] rdata
);
    (* ram_style = "block" *) reg [15:0] mem[0:32767];
    always @(posedge clk) if (we) mem[waddr] <= wdata;
    always @(posedge clk) rdata <= mem[raddr];
endmodule
