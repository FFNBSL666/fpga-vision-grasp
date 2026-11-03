// 多舵机包编码：单个关节使用单条语法，多个关节按原组包规则添加包头和包尾。
// 最低位槽首先发送；保存数据后逐槽移位，不在这里计算关节角度。

`timescale 1ns / 1ps
// 组包编码器：第0号数据槽位于打包数值的最低位。
module servo_group_encoder #(
    parameter integer MAX_SERVOS = 6,
    parameter integer INCLUDE_GROUP_TAG = 1
) (
    input wire clk,
    input wire rst_n,
    input wire start,
    input wire [3:0] servo_count,
    input wire [MAX_SERVOS * 8 - 1:0] ids,
    input wire [MAX_SERVOS * 16 - 1:0] positions,
    input wire [MAX_SERVOS * 16 - 1:0] times_ms,
    input wire [15:0] group_id,
    output reg [7:0] byte_data,
    output wire byte_valid,
    input wire byte_ready,
    output reg done
);
    localparam [2:0] IDLE = 0, HEADER = 1, START_JOINT = 2, JOINT = 3, TAIL = 4;
    reg [2:0] state;
    reg [2:0] header_index;
    reg [3:0] count_saved, joint_index;
    reg [15:0] group_saved;
    reg [MAX_SERVOS * 8 - 1:0] ids_saved;
    reg [MAX_SERVOS * 16 - 1:0] positions_saved, times_saved;
    wire [7:0] joint_byte;
    wire joint_valid, joint_done;
    wire joint_start = (state == START_JOINT);
    wire joint_ready = byte_ready && (state == JOINT);
    assign byte_valid = (state == HEADER) || (state == TAIL) || ((state == JOINT) && joint_valid);
    servo_ascii_encoder enc (
        .clk(clk),
        .rst_n(rst_n),
        .start(joint_start),
        .id(ids_saved[7:0]),
        .position(positions_saved[15:0]),
        .time_ms(times_saved[15:0]),
        .byte_data(joint_byte),
        .byte_valid(joint_valid),
        .byte_ready(joint_ready),
        .done(joint_done)
    );
    always @* begin
        byte_data = joint_byte;
        if (state == HEADER) begin
            case (header_index)
                0 : byte_data = "{";
                1 : byte_data = "G";
                2 : byte_data = "0" + (group_saved / 1000) % 10;
                3 : byte_data = "0" + (group_saved / 100) % 10;
                4 : byte_data = "0" + (group_saved / 10) % 10;
                5 : byte_data = "0" + group_saved % 10;
                default : byte_data = "{";
            endcase
        end else if (state == TAIL) byte_data = "}";
    end
    always @(posedge clk) begin
        if (!rst_n) begin
            state <= IDLE;
            header_index <= 0;
            count_saved <= 0;
            joint_index <= 0;
            group_saved <= 0;
            ids_saved <= 0;
            positions_saved <= 0;
            times_saved <= 0;
            done <= 0;
        end
        else begin
            done <= 0;
            case (state)
                IDLE : if (start) begin
                    count_saved <= servo_count;
                    joint_index <= 0;
                    header_index <= 0;
                    group_saved <= group_id;
                    ids_saved <= ids;
                    positions_saved <= positions;
                    times_saved <= times_ms;
                    state <= (servo_count > 1) ? HEADER : START_JOINT;
                end
                HEADER : if (byte_ready) begin
                    if (header_index == 5) state <= START_JOINT;
                    else header_index <= header_index + 1'b1;
                end
                START_JOINT : state <= JOINT;
                JOINT : if (joint_done) begin
                    if (joint_index + 1 >= count_saved) begin
                        if (count_saved > 1) state <= TAIL;
                        else begin
                            state <= IDLE;
                            done <= 1;
                        end
                    end
                    else begin
                        joint_index <= joint_index + 1'b1;
                        ids_saved <= ids_saved >> 8;
                        positions_saved <= positions_saved >> 16;
                        times_saved <= times_saved >> 16;
                        state <= START_JOINT;
                    end
                end
                TAIL : if (byte_ready) begin
                    state <= IDLE;
                    done <= 1;
                end
                default : state <= IDLE;
            endcase
        end
    end
endmodule
