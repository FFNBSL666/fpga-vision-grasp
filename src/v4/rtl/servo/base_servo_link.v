// 底座串口封装：以单舵机事务发送 000 号指令，语法沿用已有底座驱动。
// request/ready 完成提交握手；done 仅表示串口发送结束，不是舵机到位反馈。

`timescale 1ns/1ps
// 单独发送 ID000 底座事务，线上指令语法与已确认的底座独立测试一致。
// 完成信号只表示 UART 事务结束，不包含舵机位置反馈。
module base_servo_link #(
    parameter integer CLK_HZ = 50000000, BAUD = 115200
)(
    input wire clk, rst_n, enable, request,
    input wire [15:0] position, move_ms,
    output wire ready, done, error, tx
);
    wire unused_oe;

    // servo_count=1 且 ids=0，保持单条 #000PxxxxTxxxx! 指令语法。
    servo_command_master #(
        .CLK_HZ(CLK_HZ),
        .BAUD(BAUD),
        .MAX_SERVOS(6),
        .INCLUDE_GROUP_TAG(1)
    ) sender (
        .clk(clk),
        .rst_n(rst_n),
        .enable(enable),
        .cmd_valid(request),
        .cmd_ready(ready),
        .servo_count(4'd1),
        .ids(48'd0),
        .positions({80'd0, position}),
        .times_ms({6 {move_ms}}),
        .group_id(16'd0),
        .cmd_done(done),
        .cmd_error(error),
        .tx(tx),
        .oe(unused_oe)
    );
endmodule
