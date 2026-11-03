// UART 字节发送：8 位数据、无校验、1 位停止位，波特率由原参数设置。
// ready 表示可接收下一个字节，done 是当前字节发送完毕的脉冲。

`timescale 1ns / 1ps
// UART 发送器采用 8N1 格式，由舵机数据包发送控制器调用。
module uart_tx #(
    parameter integer CLK_HZ = 50000000,
    parameter integer BAUD = 115200
) (
    input wire clk,
    input wire rst_n,
    input wire [7:0] data,
    input wire valid,
    output wire ready,
    output reg tx,
    output reg done
);
    localparam integer DIV = CLK_HZ / BAUD;
    reg busy;
    reg [9:0] shift;
    reg [3:0] bit_index;
    reg [31:0] count;
    assign ready = !busy;
    always @(posedge clk) begin
        if (!rst_n) begin
            busy <= 0;
            tx <= 1;
            shift <= 10'h3ff;
            bit_index <= 0;
            count <= 0;
            done <= 0;
        end
        else begin
            done <= 0;
            if (!busy) begin
                tx <= 1;
                if (valid) begin
                    shift <= {1'b1, data, 1'b0};
                    tx <= 0;
                    busy <= 1;
                    bit_index <= 0;
                    count <= 0;
                end
            end else if (count == DIV - 1) begin
                count <= 0;
                if (bit_index == 9) begin
                    busy <= 0;
                    tx <= 1;
                    done <= 1;
                end
                else begin
                    shift <= {1'b1, shift[9:1]};
                    tx <= shift[1];
                    bit_index <= bit_index + 1'b1;
                end
            end else count <= count + 1'b1;
        end
    end
endmodule
