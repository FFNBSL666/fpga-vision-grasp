// 模型激活 RAM：8 位数据，单写端口与同步读端口，地址宽度由参数指定。

`timescale 1ns / 1ps
// 提供一个写端口和一个同步读端口，不复位存储阵列。
module qnn_ram8 #(
    parameter integer ADDR_BITS = 15)(
    input wire clk, we,
    input wire [ADDR_BITS - 1:0] waddr, raddr,
    input wire [7:0] wdata,
    output reg [7:0] rdata
);
    (* ram_style = "block" *) reg [7:0] mem[0:(1 << ADDR_BITS) - 1];
    always @(posedge clk) if (we) mem[waddr] <= wdata;
    always @(posedge clk) rdata <= mem[raddr];
endmodule
