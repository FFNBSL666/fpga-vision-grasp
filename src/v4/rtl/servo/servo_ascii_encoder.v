// 单舵机 ASCII 编码：# 后跟三位编号、P 四位位置、T 四位时间及 !。
// 例如 #001P1500T0600!；接收方允许一个字节时，编码索引才前进。

`timescale 1ns / 1ps
// 单个执行器指令的 ASCII 编码示例：#001P1500T0600!。
module servo_ascii_encoder(
    input wire clk,
    input wire rst_n,
    input wire start,
    input wire [7:0] id,
    input wire [15:0] position,
    input wire [15:0] time_ms,
    output reg [7:0] byte_data,
    output wire byte_valid,
    input wire byte_ready,
    output reg done
);
    reg active;
    reg [3:0] index;
    reg [7:0] saved_id;
    reg [15:0] saved_position, saved_time;
    assign byte_valid = active;

    // 按字符索引输出 #、编号、P、位置、T、时间和 !；十进制位数保持不变。
    always @* begin
        case (index)
            0 : byte_data = 8'h23;
            1 : byte_data = 8'h30 + (saved_id / 100) % 10;
            2 : byte_data = 8'h30 + (saved_id / 10) % 10;
            3 : byte_data = 8'h30 + saved_id % 10;
            4 : byte_data = 8'h50;
            5 : byte_data = 8'h30 + (saved_position / 1000) % 10;
            6 : byte_data = 8'h30 + (saved_position / 100) % 10;
            7 : byte_data = 8'h30 + (saved_position / 10) % 10;
            8 : byte_data = 8'h30 + saved_position % 10;
            9 : byte_data = 8'h54;
            10 : byte_data = 8'h30 + (saved_time / 1000) % 10;
            11 : byte_data = 8'h30 + (saved_time / 100) % 10;
            12 : byte_data = 8'h30 + (saved_time / 10) % 10;
            13 : byte_data = 8'h30 + saved_time % 10;
            14 : byte_data = 8'h21;
            default : byte_data = 8'h21;
        endcase
    end

    // 启动时锁存字段；只有对方接收一个字符后才推进到下一个索引。
    always @(posedge clk) begin
        if (!rst_n) begin
            active <= 0;
            index <= 0;
            done <= 0;
            saved_id <= 0;
            saved_position <= 0;
            saved_time <= 0;
        end
        else begin
            done <= 0;
            if (start && !active) begin
                saved_id <= id;
                saved_position <= position;
                saved_time <= time_ms;
                index <= 0;
                active <= 1;
            end
            else if (active && byte_ready) begin
                if (index == 14) begin
                    active <= 0;
                    done <= 1;
                end
                else index <= index + 1'b1;
            end
        end
    end
endmodule
