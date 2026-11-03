// 保留的双动作控制器：每次新请求执行两个动作帧，并等待各自运动时间。
// 每帧支持 1～6 个关节，协议范围检查不等同于机械限位标定。

`timescale 1ns / 1ps
// 每次新请求在 FPGA 内执行一次包含两个动作帧的流程。
// 每个动作帧包含1至6个关节，先发送第0号数据槽。
// 下方检查仅验证协议范围，不表示机械限位或角度与 P 值的标定。
// 此模块不内置姿态数值、回零、行程搜索或上电自动动作。
module arm_action_pair_controller #(
    parameter integer CLK_HZ = 50000000,
    parameter integer BAUD = 115200,
    parameter integer MIN_DURATION_MS = 2000,
    parameter integer SETTLE_MARGIN_MS = 200
)(
    input wire clk, rst_n, enable, estop, parameters_valid, request_toggle,
    input wire [7:0] requested_counts,
    input wire [95:0] requested_ids,
    input wire [191:0] requested_positions,
    input wire [191:0] requested_times_ms,
    output wire tx, ready, busy,
    output reg stop_latched, fault_latched, active_step, pair_done,
    output reg [7:0] error_code,
    output reg [31:0] accepted_pairs, sent_frames, completed_pairs
);
    localparam [1:0] IDLE = 0, SEND = 1, WAIT_TX = 2, SETTLE = 3;
    localparam integer MS_DIV = CLK_HZ / 1000;
    reg [1:0] state;
    reg seen_toggle;
    reg [7:0] saved_counts;
    reg [95:0] saved_ids;
    reg [191:0] saved_positions, saved_times;
    reg [15:0] saved_wait_first, saved_wait_second;
    reg [31:0] ms_divider;
    reg [15:0] elapsed_ms;
    wire live = rst_n && enable && parameters_valid && !estop &&
    !stop_latched && !fault_latched;
    assign ready = live && state == IDLE;
    assign busy = state != IDLE;

    function frame_valid;
        input [3:0] count;
        input [47:0] ids;
        input [95:0] positions, times_ms;
        integer i, j;
        begin
            frame_valid = (count >= 1 && count <= 6);
            for (i = 0; i < 6; i = i + 1) if (i < count) begin
                if (ids[i * 8 +: 8] > 5 || positions[i * 16 +: 16] < 500 ||
                    positions[i * 16 +: 16] > 2500 || times_ms[i * 16 +: 16] < MIN_DURATION_MS ||
                    times_ms[i * 16 +: 16] > 9999) frame_valid = 0;
                for (j = 0; j < 6; j = j + 1)
                if (j < i && ids[i * 8 +: 8] == ids[j * 8 +: 8]) frame_valid = 0;
            end
        end
    endfunction
    function [15:0] max_duration;
        input [15:0] a, b;
        begin
            max_duration = a > b ? a : b;
        end
    endfunction
    function [15:0] frame_wait_ms;
        input [3:0] count;
        input [95:0] times_ms;
        reg [15:0] ab, cd, ef;
        begin
            // 使用平衡比较树求最大值；比较前先屏蔽未使用的关节数据槽。
            ab = max_duration(count > 0 ? times_ms[15:0] : 16'd0,
                count > 1 ? times_ms[31:16] : 16'd0);
            cd = max_duration(count > 2 ? times_ms[47:32] : 16'd0,
                count > 3 ? times_ms[63:48] : 16'd0);
            ef = max_duration(count > 4 ? times_ms[79:64] : 16'd0,
                count > 5 ? times_ms[95:80] : 16'd0);
            frame_wait_ms = max_duration(max_duration(ab, cd), ef) + SETTLE_MARGIN_MS;
        end
    endfunction
    wire all_valid = frame_valid(requested_counts[3:0], requested_ids[47:0],
        requested_positions[95:0], requested_times_ms[95:0]) &&
    frame_valid(requested_counts[7:4], requested_ids[95:48],
        requested_positions[191:96], requested_times_ms[191:96]);
    wire [3:0] count = active_step ? saved_counts[7:4] : saved_counts[3:0];
    wire [47:0] ids = active_step ? saved_ids[95:48] : saved_ids[47:0];
    wire [95:0] positions = active_step ? saved_positions[191:96] : saved_positions[95:0];
    wire [95:0] times_ms = active_step ? saved_times[191:96] : saved_times[95:0];
    wire [15:0] wait_ms = active_step ? saved_wait_second : saved_wait_first;
    wire master_ready, master_done, master_error, unused_oe;
    servo_command_master #(
        .CLK_HZ(CLK_HZ),
        .BAUD(BAUD),
        .MAX_SERVOS(6)
    ) sender (
        .clk(clk),
        .rst_n(rst_n),
        .enable(live),
        .cmd_valid(state == SEND && live),
        .cmd_ready(master_ready),
        .servo_count(count),
        .ids(ids),
        .positions(positions),
        .times_ms(times_ms),
        .group_id(16'd0),
        .cmd_done(master_done),
        .cmd_error(master_error),
        .tx(tx),
        .oe(unused_oe)
    );

    always @(posedge clk) begin
        if (!rst_n) begin
            state <= IDLE;
            seen_toggle <= 0;
            stop_latched <= 0;
            fault_latched <= 0;
            error_code <= 0;
            active_step <= 0;
            pair_done <= 0;
            accepted_pairs <= 0;
            sent_frames <= 0;
            completed_pairs <= 0;
            saved_counts <= 0;
            saved_ids <= 0;
            saved_positions <= 0;
            saved_times <= 0;
            saved_wait_first <= 0;
            saved_wait_second <= 0;
            ms_divider <= 0;
            elapsed_ms <= 0;
        end else begin
            pair_done <= 0;
            if (estop) stop_latched <= 1;
            else if (!enable) stop_latched <= 0;
            if (!enable) begin
                fault_latched <= 0;
                error_code <= 0;
            end
            if (!live) begin
                // 禁用期间出现的请求直接消耗，恢复后不重放。
                state <= IDLE;
                seen_toggle <= request_toggle;
                active_step <= 0;
                ms_divider <= 0;
                elapsed_ms <= 0;
            end else begin
                if (request_toggle != seen_toggle) begin
                    seen_toggle <= request_toggle;
                    if (state != IDLE) error_code <= 8'd5;  // 丢弃该请求，不排队等待。
                    else if (!all_valid) error_code <= 8'd1;  // 首次发送前一并拒绝两个无效动作帧。
                    else begin
                        saved_counts <= requested_counts;
                        saved_ids <= requested_ids;
                        saved_positions <= requested_positions;
                        saved_times <= requested_times_ms;
                        // 与指令数据在同一采样时刻计算并保存一次。
                        // 毫秒计时完成通路只选择已保存的寄存器数值。
                        saved_wait_first <= frame_wait_ms(requested_counts[3:0], requested_times_ms[95:0]);
                        saved_wait_second <= frame_wait_ms(requested_counts[7:4], requested_times_ms[191:96]);
                        active_step <= 0;
                        error_code <= 0;
                        state <= SEND;
                        accepted_pairs <= accepted_pairs + 1'b1;
                    end
                end
                case (state)
                    SEND : if (master_ready) state <= WAIT_TX;
                    WAIT_TX : begin
                        if (master_error) begin
                            fault_latched <= 1;
                            error_code <= 8'd6;
                            state <= IDLE;
                        end else if (master_done) begin
                            sent_frames <= sent_frames + 1'b1;
                            state <= SETTLE;
                            ms_divider <= 0;
                            elapsed_ms <= 0;
                        end
                    end
                    SETTLE : begin
                        if (ms_divider + 1 >= MS_DIV) begin
                            ms_divider <= 0;
                            if (elapsed_ms + 1 >= wait_ms) begin
                                elapsed_ms <= 0;
                                if (!active_step) begin
                                    active_step <= 1;
                                    state <= SEND;
                                end
                                else begin
                                    completed_pairs <= completed_pairs + 1'b1;
                                    pair_done <= 1;
                                    state <= IDLE;
                                end
                            end else elapsed_ms <= elapsed_ms + 1'b1;
                        end else ms_divider <= ms_divider + 1'b1;
                    end
                    default : begin
                    end
                endcase
            end
        end
    end
endmodule
