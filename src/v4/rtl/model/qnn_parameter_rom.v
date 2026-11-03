// 模型参数 ROM：分别存储量化权重和整数阈值，输出随时钟同步更新。
// $readmemh 使用原有导出的参数文件；本次不修改模型数据。

`timescale 1ns / 1ps
module qnn_parameter_rom #(
    parameter WEIGHTS_FILE = "qnn_weights.mem",
    parameter THRESHOLDS_FILE = "qnn_thresholds.mem"
)(
    input wire clk,
    input wire [14:0] weight_addr,
    input wire [9:0] threshold_addr,
    output reg signed [7:0] weight_data,
    output reg [31:0] threshold_data
);
    (* rom_style = "block" *) reg [7:0] weights[0:32767];
    (* rom_style = "block" *) reg [31:0] thresholds[0:1023];
    initial begin
        $readmemh(WEIGHTS_FILE, weights);
        $readmemh(THRESHOLDS_FILE, thresholds);
    end
    always @(posedge clk) begin
        weight_data <= weights[weight_addr];
        threshold_data <= thresholds[threshold_addr];
    end
endmodule
