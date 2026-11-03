// 旧单舵机总线兼容接口：把原请求转换为当前串口发送器的单关节事务。

`timescale 1ns / 1ps
// 保留兼容的单舵机指令接口，并提供半双工方向使能 OE。
module servo_bus_master #(
    parameter integer CLK_HZ = 50000000,
    parameter integer BAUD = 115200,
    parameter integer TURN_BITS = 1
)(
    input wire clk,
    input wire rst_n,
    input wire enable,
    input wire cmd_valid,
    output wire cmd_ready,
    input wire [7:0] cmd_id,
    input wire [15:0] cmd_position,
    input wire [15:0] cmd_time,
    output wire cmd_done,
    output wire cmd_error,
    output wire tx,
    output wire oe
);
    servo_command_master #(
        .CLK_HZ(CLK_HZ),
        .BAUD(BAUD),
        .MAX_SERVOS(1),
        .INCLUDE_GROUP_TAG(0),
        .TURN_BITS(TURN_BITS)
    ) packet (
        .clk(clk),
        .rst_n(rst_n),
        .enable(enable),
        .cmd_valid(cmd_valid),
        .cmd_ready(cmd_ready),
        .servo_count(4'd1),
        .ids(cmd_id),
        .positions(cmd_position),
        .times_ms(cmd_time),
        .group_id(16'd0),
        .cmd_done(cmd_done),
        .cmd_error(cmd_error),
        .tx(tx),
        .oe(oe)
    );
endmodule
