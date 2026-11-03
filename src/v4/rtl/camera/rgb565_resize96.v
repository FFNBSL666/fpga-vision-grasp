// 图像预处理：缓存有效 RGB565 帧，再以中心对齐的双线性插值缩放。
// 输出 RGB888 像素流；仅在 rgb_valid 与 rgb_ready 同时为真时推进像素。
// 处理期间到达的新包整帧丢弃，避免覆盖正在读取的图像。

`timescale 1ns / 1ps
// 在 PL 中完成单帧缓存接收，以及中心对齐的 Q8 双线性缩放。
// 按行输出 RGB888，使用 ready/valid 握手和末像素标志；本模块不执行模型推理。
// 处理期间到达的数据包整包丢弃，避免混合不同帧。
module rgb565_resize96 #(
    parameter integer SRC_W = 160, SRC_H = 120, DST_W = 96, DST_H = 96
)(
    input wire clk, rst_n, frame_start, frame_good,
    input wire pixel_valid,
    input wire [15:0] pixel, pixel_addr,
    input wire [7:0] frame_id,
    output reg buffer_frame_start, buffer_frame_good,
    output reg [7:0] buffer_frame_id,
    output reg [23:0] rgb_data,
    output reg rgb_valid,
    input wire rgb_ready,
    output reg rgb_last,
    output wire processing
);
    reg [15:0] rd_addr, base_addr, p00, p10, p01, p11;
    wire [15:0] rd_word;
    reg [7:0] fx, fy, ox, oy;
    reg [3:0] state;
    reg capture_ok;
    reg [7:0] x_index[0:DST_W - 1], y_index[0:DST_H - 1];
    reg [7:0] x_frac[0:DST_W - 1], y_frac[0:DST_H - 1];
    integer i, q;

    // 预计算中心对齐的源像素索引与 Q8 小数权重，保留原缩放公式。
    initial begin
        for (i = 0; i < DST_W; i = i + 1) begin
            q = (((2 * i + 1) * SRC_W - DST_W) * 256) / (2 * DST_W);
            x_index[i] = q >> 8;
            x_frac[i] = q & 255;
        end
        for (i = 0; i < DST_H; i = i + 1) begin
            q = (((2 * i + 1) * SRC_H - DST_H) * 256) / (2 * DST_H);
            y_index[i] = q >> 8;
            y_frac[i] = q & 255;
        end
    end
    assign processing = (state != 0);
    camera_frame_store storage (
        .clk(clk),
        .we(pixel_valid && capture_ok && !processing),
        .waddr(pixel_addr[14:0]),
        .raddr(rd_addr[14:0]),
        .wdata(pixel),
        .rdata(rd_word)
    );

    // 把 RGB565 位段扩展为 RGB888 分量，供双线性插值使用。
    function [23:0] expand;
        input [15:0] v;
        begin
            expand = {{v[15:11], v[15:13]}, {v[10:5], v[10:9]}, {v[4:0], v[4:2]}};
        end
    endfunction

    // 按四个邻近像素及横纵权重插值，保留原有舍入常数 32768。
    function [7:0] blend;
        input [7:0] a, b, c, d, u, v;
        reg [31:0] top, bottom, result;
        begin
            top = a * (256 - u) + b * u;
            bottom = c * (256 - u) + d * u;
            result = top * (256 - v) + bottom * v + 32768;
            blend = result >> 16;
        end
    endfunction
    wire [23:0] c00 = expand(p00), c10 = expand(p10), c01 = expand(p01), c11 = expand(p11);
    always @(posedge clk) begin
        if (!rst_n) begin
            state <= 0;
            capture_ok <= 0;
            rd_addr <= 0;
            base_addr <= 0;
            ox <= 0;
            oy <= 0;
            fx <= 0;
            fy <= 0;
            p00 <= 0;
            p10 <= 0;
            p01 <= 0;
            p11 <= 0;
            rgb_data <= 0;
            rgb_valid <= 0;
            rgb_last <= 0;
            buffer_frame_start <= 0;
            buffer_frame_good <= 0;
            buffer_frame_id <= 0;
        end else begin
            buffer_frame_start <= 0;
            buffer_frame_good <= 0;
            if (frame_start) begin
                capture_ok <= !processing;
                if (!processing) begin
                    buffer_frame_start <= 1;
                    buffer_frame_id <= frame_id;
                end
            end

            // 状态 0 等整帧有效；1～10 分步读取四个邻点；11 计算；12 等下游接收。
            case (state)
                0 : if (frame_good && capture_ok) begin
                    ox <= 0;
                    oy <= 0;
                    state <= 1;
                    capture_ok <= 0;
                    buffer_frame_good <= 1;
                end
                1 : begin
                    base_addr <= y_index[oy] * SRC_W + x_index[ox];
                    fx <= x_frac[ox];
                    fy <= y_frac[oy];
                    state <= 2;
                end
                2 : begin
                    rd_addr <= base_addr;
                    state <= 3;
                end
                3 : state <= 4;
                4 : begin
                    p00 <= rd_word;
                    rd_addr <= base_addr + 1;
                    state <= 5;
                end
                5 : state <= 6;
                6 : begin
                    p10 <= rd_word;
                    rd_addr <= base_addr + SRC_W;
                    state <= 7;
                end
                7 : state <= 8;
                8 : begin
                    p01 <= rd_word;
                    rd_addr <= base_addr + SRC_W + 1;
                    state <= 9;
                end
                9 : state <= 10;
                10 : begin
                    p11 <= rd_word;
                    state <= 11;
                end
                11 : begin
                    rgb_data <= {blend(c00[23:16], c10[23:16], c01[23:16], c11[23:16], fx, fy),
                    blend(c00[15:8], c10[15:8], c01[15:8], c11[15:8], fx, fy),
                    blend(c00[7:0], c10[7:0], c01[7:0], c11[7:0], fx, fy)};
                    rgb_valid <= 1;
                    rgb_last <= (ox == DST_W - 1 && oy == DST_H - 1);
                    state <= 12;
                end
                12 : if (rgb_ready) begin
                    rgb_valid <= 0;
                    rgb_last <= 0;
                    if (ox == DST_W - 1 && oy == DST_H - 1) state <= 0;
                    else begin
                        if (ox == DST_W - 1) begin
                            ox <= 0;
                            oy <= oy + 1'b1;
                        end
                        else ox <= ox + 1'b1;
                        state <= 1;
                    end
                end
                default : begin
                    state <= 0;
                    rgb_valid <= 0;
                    rgb_last <= 0;
                end
            endcase
        end
    end
endmodule
