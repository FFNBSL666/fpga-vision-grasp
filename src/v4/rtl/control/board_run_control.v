// 按键去抖与运行锁存：低电平按下，复位、急停或故障清除运行允许。
// 接入方式以 fpga_board_top 对应分支为准，本次不改变任何按键功能。

`timescale 1ns / 1ps
// 启动按键去抖后锁存运行允许；复位、急停或故障清除允许，不自动重新启动。
module board_run_control #(
    parameter integer DEBOUNCE_CYCLES = 500000)(
    input wire clk, rst_n, start_n, estop_n, fault,
    output reg run_enable
);
    (* ASYNC_REG = "TRUE" *) reg [1:0] start_ff, stop_ff;
    reg [31:0] count;
    reg pressed, released_once, last_start;
    always @(posedge clk) begin
        if (!rst_n) begin
            start_ff <= 2'b11;
            stop_ff <= 2'b11;
        end
        else begin
            start_ff <= {start_ff[0], start_n};
            stop_ff <= {stop_ff[0], estop_n};
        end
    end
    always @(posedge clk) begin
        if (!rst_n) begin
            run_enable <= 0;
            count <= 0;
            pressed <= 0;
            released_once <= 0;
            last_start <= 1;
        end else if (!estop_n || !stop_ff[1] || fault) begin
            run_enable <= 0;
            count <= 0;
            pressed <= 0;
            released_once <= 0;
            last_start <= 1;
        end else if (start_ff[1] != last_start) begin
            count <= 0;
            last_start <= start_ff[1];
        end else if (count < DEBOUNCE_CYCLES - 1) begin
            count <= count + 1'b1;
        end else if (start_ff[1]) begin
            pressed <= 0;
            released_once <= 1;
        end else if (!pressed && released_once) begin
            pressed <= 1;
            run_enable <= 1;
        end
    end
endmodule
