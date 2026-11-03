// 舵机指令发送：检查原有协议数值范围，保存请求，再编码并逐字节输出。
// IDLE 接收请求，LEAD 准备发送，SEND 输出数据，DRAIN 等最后字节结束，TURN 留出间隔。
// cmd_done 只表示本次串口事务结束；机械动作等待由上层状态机计时。

`timescale 1ns / 1ps
// 具有半双工方向使能的指令数据包发送控制器。
module servo_command_master #(
    parameter integer CLK_HZ = 50000000,
    parameter integer BAUD = 115200,
    parameter integer MAX_SERVOS = 6,
    parameter integer INCLUDE_GROUP_TAG = 1,
    parameter integer OE_LEAD_CYCLES = 2,
    parameter integer TURN_BITS = 1
) (
    input wire clk,
    input wire rst_n,
    input wire enable,
    input wire cmd_valid,
    output wire cmd_ready,
    input wire [3:0] servo_count,
    input wire [MAX_SERVOS * 8 - 1:0] ids,
    input wire [MAX_SERVOS * 16 - 1:0] positions,
    input wire [MAX_SERVOS * 16 - 1:0] times_ms,
    input wire [15:0] group_id,
    output reg cmd_done,
    output reg cmd_error,
    output wire tx,
    output wire oe
);
    localparam integer DIV = CLK_HZ / BAUD;
    localparam integer TURN_CYCLES = (TURN_BITS < 1) ? 1 : DIV * TURN_BITS;
    localparam [2:0] IDLE = 0, LEAD = 1, SEND = 2, DRAIN = 3, TURN = 4;
    reg [2:0] state;
    reg [31:0] counter;
    reg enc_start;
    reg [3:0] saved_count;
    reg [MAX_SERVOS * 8 - 1:0] saved_ids;
    reg [MAX_SERVOS * 16 - 1:0] saved_positions, saved_times;
    reg [15:0] saved_group;
    reg valid_range;
    integer n;

    // 检查指令数量、编号、P 值与时间字段；不修改原有边界条件。
    always @* begin
        valid_range = (servo_count >= 1) && (servo_count <= MAX_SERVOS);
        if (servo_count > 1 && INCLUDE_GROUP_TAG != 0 && group_id > 9999) valid_range = 0;
        for (n = 0; n < MAX_SERVOS; n = n + 1) if (n < servo_count) begin
            if (ids[n * 8 +: 8] > 254 || positions[n * 16 +: 16] < 500 || positions[n * 16 +: 16] > 2500 || times_ms[n * 16 +: 16] > 9999) valid_range = 0;
        end
    end
    assign cmd_ready = rst_n && enable && (state == IDLE);
    assign oe = rst_n && enable && ((state == LEAD) || (state == SEND) || (state == DRAIN));
    wire [7:0] byte_data;
    wire byte_valid, byte_ready, enc_done, tx_done;

    // 包编码器与 UART 用 byte_valid/byte_ready 握手，确保字节顺序连续。
    servo_group_encoder #(
        .MAX_SERVOS(MAX_SERVOS),
        .INCLUDE_GROUP_TAG(INCLUDE_GROUP_TAG)
    ) encoder (
        .clk(clk),
        .rst_n(rst_n && enable),
        .start(enc_start),
        .servo_count(saved_count),
        .ids(saved_ids),
        .positions(saved_positions),
        .times_ms(saved_times),
        .group_id(saved_group),
        .byte_data(byte_data),
        .byte_valid(byte_valid),
        .byte_ready(byte_ready),
        .done(enc_done)
    );
    uart_tx #(
        .CLK_HZ(CLK_HZ),
        .BAUD(BAUD)
    ) transmitter (
        .clk(clk),
        .rst_n(rst_n && enable),
        .data(byte_data),
        .valid(byte_valid),
        .ready(byte_ready),
        .tx(tx),
        .done(tx_done)
    );

    // 复位/禁用时清空本地事务，其余按原状态机完成一次发送。
    always @(posedge clk) begin
        if (!rst_n || !enable) begin
            state <= IDLE;
            counter <= 0;
            enc_start <= 0;
            saved_count <= 0;
            saved_ids <= 0;
            saved_positions <= 0;
            saved_times <= 0;
            saved_group <= 0;
            cmd_done <= 0;
            cmd_error <= 0;
        end
        else begin
            enc_start <= 0;
            cmd_done <= 0;
            cmd_error <= 0;
            case (state)
                IDLE : if (cmd_valid) begin
                    if (!valid_range) cmd_error <= 1;
                    else begin
                        saved_count <= servo_count;
                        saved_ids <= ids;
                        saved_positions <= positions;
                        saved_times <= times_ms;
                        saved_group <= group_id;
                        counter <= 0;
                        state <= LEAD;
                    end
                end
                LEAD : if (counter + 1 >= 2) begin
                    counter <= 0;
                    enc_start <= 1;
                    state <= SEND;
                end else counter <= counter + 1'b1;
                SEND : if (enc_done) state <= DRAIN;
                DRAIN : if (tx_done) begin
                    counter <= 0;
                    state <= TURN;
                end
                TURN : if (counter + 1 >= TURN_CYCLES) begin
                    state <= IDLE;
                    cmd_done <= 1;
                    counter <= 0;
                end else counter <= counter + 1'b1;
                default : state <= IDLE;
            endcase
        end
    end
endmodule
