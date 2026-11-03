// RGB565 包解析：核对帧头、版本、尺寸和格式，接收像素后检查 CRC16。
// 像素按低字节在前传输；CRC 覆盖版本到像素负载，初值 FFFF、多项式 1021。
// pixel_valid 是暂存写入信号；只有 frame_good 才允许后续使用整帧。

`timescale 1ns / 1ps
// 第2版数据包：A5 5A 02、帧号、宽、高、01；随后是小端 RGB565 像素和 CRC16 高低字节。
// CRC16-CCITT-FALSE 校验覆盖版本字段至像素负载，初值为 FFFF，多项式为 1021。
// 像素先暂存到 RAM；只有 frame_good 有效时，才允许后续使用这帧数据。
module camera_rgb565_rx #(
    parameter integer IMAGE_W = 160, IMAGE_H = 120,
    parameter integer CLK_HZ = 50000000, GAP_MS = 100
)(
    input wire clk, rst_n,
    input wire [7:0] byte_data,
    input wire byte_valid, byte_error,
    output reg frame_start, pixel_valid,
    output reg [15:0] pixel, pixel_addr,
    output reg frame_good, frame_bad,
    output reg [7:0] frame_id
);
    localparam S_MAGIC0 = 0, S_MAGIC1 = 1, S_VER = 2, S_SEQ = 3, S_W = 4, S_H = 5, S_FMT = 6,
    S_LOW = 7, S_HIGH = 8, S_CRC_H = 9, S_CRC_L = 10;
    reg [3:0] state;
    reg [15:0] crc;
    reg [7:0] low_byte, crc_high;
    reg [15:0] index;
    reg [31:0] gap_count;

    // 每收到一个受保护字节，按原 CRC16 算法更新校验值。
    function [15:0] crc_byte;
        input [15:0] old_crc;
        input [7:0] value;
        reg [15:0] c;
        integer k;
        begin
            c = old_crc ^ {value, 8'b0};
            for (k = 0; k < 8; k = k + 1) c = c[15] ? (c << 1) ^ 16'h1021 : c << 1;
            crc_byte = c;
        end
    endfunction
    always @(posedge clk) begin
        if (!rst_n) begin
            state <= S_MAGIC0;
            crc <= 16'hffff;
            index <= 0;
            gap_count <= 0;
            frame_start <= 0;
            pixel_valid <= 0;
            pixel <= 0;
            pixel_addr <= 0;
            frame_good <= 0;
            frame_bad <= 0;
            frame_id <= 0;
            low_byte <= 0;
            crc_high <= 0;
        end else begin
            frame_start <= 0;
            pixel_valid <= 0;
            frame_good <= 0;
            frame_bad <= 0;
            if (byte_error || (state != S_MAGIC0 && gap_count >= CLK_HZ / 1000 * GAP_MS - 1)) begin
                frame_bad <= 1;
                state <= S_MAGIC0;
                gap_count <= 0;
            end else if (byte_valid) begin
                gap_count <= 0;
                if (state >= S_VER && state <= S_HIGH) crc <= crc_byte(crc, byte_data);

                // 包解析状态依次处理魔数、版本、帧号、尺寸、格式、像素和校验字节。
                case (state)
                    S_MAGIC0 : if (byte_data == 8'ha5) state <= S_MAGIC1;
                    S_MAGIC1 : if (byte_data == 8'h5a) begin
                        state <= S_VER;
                        crc <= 16'hffff;
                    end
                    else if (byte_data != 8'ha5) state <= S_MAGIC0;
                    S_VER : if (byte_data == 2) state <= S_SEQ;
                    else begin
                        state <= S_MAGIC0;
                        frame_bad <= 1;
                    end
                    S_SEQ : begin
                        frame_id <= byte_data;
                        state <= S_W;
                    end
                    S_W : if (byte_data == IMAGE_W) state <= S_H;
                    else begin
                        state <= S_MAGIC0;
                        frame_bad <= 1;
                    end
                    S_H : if (byte_data == IMAGE_H) state <= S_FMT;
                    else begin
                        state <= S_MAGIC0;
                        frame_bad <= 1;
                    end
                    S_FMT : if (byte_data == 1) begin
                        state <= S_LOW;
                        index <= 0;
                        frame_start <= 1;
                    end
                    else begin
                        state <= S_MAGIC0;
                        frame_bad <= 1;
                    end
                    S_LOW : begin
                        low_byte <= byte_data;
                        state <= S_HIGH;
                    end

                    // 合并小端 RGB565 的两个字节并给出缓存写地址。
                    S_HIGH : begin
                        pixel <= {byte_data, low_byte};
                        pixel_addr <= index;
                        pixel_valid <= 1;
                        if (index == IMAGE_W * IMAGE_H - 1) state <= S_CRC_H;
                        else begin
                            index <= index + 1'b1;
                            state <= S_LOW;
                        end
                    end
                    S_CRC_H : begin
                        crc_high <= byte_data;
                        state <= S_CRC_L;
                    end

                    // 只有最终 CRC 一致才报告好帧，否则报告坏帧并重新寻找帧头。
                    S_CRC_L : begin
                        if ({crc_high, byte_data} == crc) frame_good <= 1;
                        else frame_bad <= 1;
                        state <= S_MAGIC0;
                    end
                    default : state <= S_MAGIC0;
                endcase
            end else if (state != S_MAGIC0) gap_count <= gap_count + 1'b1;
        end
    end
endmodule
