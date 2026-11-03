// 稳定坐标捕获接口：对照初始锚点累计稳定时间，移动或失效时重新累计。
// 坐标单位是 0～95 的图像像素；此处不进行桌面厘米距离判断。

`timescale 1ns / 1ps
// 检测器接口库：原设计要求模型 IP 验证通过后再接入。
// 坐标使用0至95的图像像素值，不采用0至255的归一化坐标。
// 稳定性始终相对最初锚点检查，缓慢漂移也会重新开始稳定计时。
module point_hold_capture #(
    parameter integer HOLD_MS = 2000, FRAME_TIMEOUT_MS = 1500, STABLE_TOL = 4,
    parameter integer CONF_THRESHOLD = 128, IMAGE_W = 96, IMAGE_H = 96
)(
    input wire clk, rst_n, tick_ms, frame_valid, object_valid, finger_valid,
    input wire [7:0] object_confidence, finger_confidence, object_x, object_y, finger_x, finger_y,
    input wire clear_capture,
    output reg object_latched,
    output reg [7:0] object_center_x, object_center_y,
    output reg finger_tracking,
    output reg [15:0] stable_ms,
    output reg target_valid,
    output reg [7:0] target_x, target_y,
    output reg place_request
);
    reg [7:0] anchor_x, anchor_y;
    reg [31:0] frame_age_ms;
    wire [7:0] dx = (finger_x > anchor_x) ? finger_x - anchor_x : anchor_x - finger_x;
    wire [7:0] dy = (finger_y > anchor_y) ? finger_y - anchor_y : anchor_y - finger_y;
    wire interval_fresh = (FRAME_TIMEOUT_MS > 0 && frame_age_ms < FRAME_TIMEOUT_MS &&
        !(tick_ms && frame_age_ms >= FRAME_TIMEOUT_MS - 1));
    wire finger_ok = finger_valid && finger_confidence >= CONF_THRESHOLD &&
    finger_x < IMAGE_W && finger_y < IMAGE_H;
    wire hold_elapsed = (HOLD_MS == 0 || stable_ms >= HOLD_MS ||
        (tick_ms && stable_ms >= HOLD_MS - 1));
    always @(posedge clk) begin
        if (!rst_n || clear_capture) begin
            object_latched <= 0;
            object_center_x <= 0;
            object_center_y <= 0;
            finger_tracking <= 0;
            stable_ms <= 0;
            target_valid <= 0;
            target_x <= 0;
            target_y <= 0;
            place_request <= 0;
            anchor_x <= 0;
            anchor_y <= 0;
            frame_age_ms <= FRAME_TIMEOUT_MS;
        end else begin
            place_request <= 0;
            if (tick_ms) begin
                if (frame_age_ms < FRAME_TIMEOUT_MS) frame_age_ms <= frame_age_ms + 1'b1;
                if (!interval_fresh) begin
                    finger_tracking <= 0;
                    stable_ms <= 0;
                end
                else if (finger_tracking && stable_ms < HOLD_MS) stable_ms <= stable_ms + 1'b1;
            end
            if (frame_valid) begin
                frame_age_ms <= 0;
                if (object_valid && object_confidence >= CONF_THRESHOLD && !object_latched &&
                    object_x < IMAGE_W && object_y < IMAGE_H) begin
                    object_latched <= 1;
                    object_center_x <= object_x;
                    object_center_y <= object_y;
                end
                if (!target_valid) begin
                    if (!finger_ok) begin
                        finger_tracking <= 0;
                        stable_ms <= 0;
                    end
                    else if (!finger_tracking || !interval_fresh || dx > STABLE_TOL || dy > STABLE_TOL) begin
                        finger_tracking <= 1;
                        anchor_x <= finger_x;
                        anchor_y <= finger_y;
                        stable_ms <= 0;
                    end else if (object_latched && hold_elapsed) begin
                        target_x <= finger_x;
                        target_y <= finger_y;
                        target_valid <= 1;
                        place_request <= 1;
                    end
                end
            end
        end
    end
endmodule
