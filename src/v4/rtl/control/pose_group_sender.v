// 整组姿态提交：六个 16 位位置组成一个动作请求，最低位槽为首个关节。
// 发送完成表示请求已传输，不代表机械臂已经到位。

`timescale 1ns / 1ps
// 在一个数据包中提交六个关节；第0号数据槽位于打包数值的最低位。
module pose_group_sender #(
    parameter integer ID_BASE = 0
)(
    input wire clk,
    input wire rst_n,
    input wire start,
    input wire [95:0] positions,
    input wire [15:0] duration_ms,
    output wire cmd_valid,
    input wire cmd_ready,
    output wire [47:0] cmd_ids,
    output wire [95:0] cmd_positions,
    output wire [95:0] cmd_times,
    input wire cmd_done,
    output wire busy,
    output reg done
);
    localparam [1:0] IDLE = 0, SEND = 1, WAIT_DONE = 2;
    reg [1:0] state;
    reg [95:0] saved_positions;
    reg [15:0] saved_time;
    assign cmd_valid = (state == SEND);
    assign cmd_positions = saved_positions;
    assign cmd_times = {6 {saved_time}};
    assign busy = (state != IDLE);
    genvar j;
    generate
        for (j = 0; j < 6; j = j + 1) begin : ids
            localparam [7:0] JOINT_ID = ID_BASE + j;
            assign cmd_ids[j * 8 +: 8] = JOINT_ID;
        end
    endgenerate
    always @(posedge clk) begin
        if (!rst_n) begin
            state <= IDLE;
            saved_positions <= 0;
            saved_time <= 0;
            done <= 0;
        end
        else begin
            done <= 0;
            case (state)
                IDLE : if (start) begin
                    saved_positions <= positions;
                    saved_time <= duration_ms;
                    state <= SEND;
                end
                SEND : if (cmd_ready) state <= WAIT_DONE;
                WAIT_DONE : if (cmd_done) begin
                    state <= IDLE;
                    done <= 1;
                end
                default : state <= IDLE;
            endcase
        end
    end
endmodule
