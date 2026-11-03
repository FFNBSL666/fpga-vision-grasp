// 保留的单点选择器：物体接近手指且结果稳定后，等待手移开再提交坐标。
// V4 的 3 cm 两点选择由 pick_place_target_mapper 实现。

`timescale 1ns/1ps
// 选择手指附近检出的物体，抓取使用物体中心坐标。
// 先要求三个新的稳定选点结果，再要求两个新的无手结果。
module pointing_target_selector #(
    parameter integer POINT_RADIUS = 20, STABLE_RADIUS = 6, POINT_FRAMES = 3, CLEAR_FRAMES = 2,
    parameter integer SELECT_FINGER = 0
)(
    input wire clk, rst_n, enable, result_valid,
    input wire [49:0] detections,
    input wire map_response, map_valid,
    output reg request,
    output reg [7:0] selected_x, selected_y
);
    wire fv = detections[0], ov = detections[25];
    wire [7:0] fc = detections[8:1], fx = detections[16:9], fy = detections[24:17];
    wire [7:0] oc = detections[33:26], ox = detections[41:34], oy = detections[49:42];
    wire finger = fv && fc >= 128, object_ok = ov && oc >= 128 && ox < 96 && oy < 96;
    wire [7:0] px = fx > ox ? fx - ox : ox - fx, py = fy > oy ? fy - oy : oy - fy;
    wire [7:0] sx = SELECT_FINGER != 0 ? fx : ox, sy = SELECT_FINGER != 0 ? fy : oy;
    wire [7:0] dx = sx > selected_x ? sx - selected_x : selected_x - sx;
    wire [7:0] dy = sy > selected_y ? sy - selected_y : selected_y - sy;
    wire near_object = SELECT_FINGER != 0 ? (finger && fx < 96 && fy < 96) :
    (finger && object_ok && px <= POINT_RADIUS && py <= POINT_RADIUS);
    wire stable_object = object_ok && dx <= STABLE_RADIUS && dy <= STABLE_RADIUS;
    reg [2:0] state;
    reg [7:0] point_count, clear_count;
    always @(posedge clk) begin
        if (!rst_n || !enable) begin
            state <= 0;
            point_count <= 0;
            clear_count <= 0;
            request <= 0;
            selected_x <= 0;
            selected_y <= 0;
        end else begin
            request <= 0;
            if (state == 3 && map_response) state <= map_valid ? 4 : 0;
            if (result_valid) case (state)
                0 : if (near_object) begin
                    selected_x <= sx;
                    selected_y <= sy;
                    point_count <= 1;
                    state <= 1;
                end
                1 : begin
                    if (!near_object) begin
                        state <= 0;
                        point_count <= 0;
                    end
                    else if (dx > STABLE_RADIUS || dy > STABLE_RADIUS) begin
                        selected_x <= sx;
                        selected_y <= sy;
                        point_count <= 1;
                    end
                    else if (point_count + 1 >= POINT_FRAMES) begin
                        clear_count <= 0;
                        state <= 2;
                    end
                    else point_count <= point_count + 1'b1;
                end
                2 : begin
                    if (SELECT_FINGER != 0) begin
                        if (finger) begin
                            clear_count <= 0;
                            if (dx > STABLE_RADIUS || dy > STABLE_RADIUS) begin
                                selected_x <= sx;
                                selected_y <= sy;
                                point_count <= 1;
                                state <= 1;
                            end
                        end else if (clear_count + 1 >= CLEAR_FRAMES) begin
                            request <= 1;
                            state <= 3;
                        end
                        else clear_count <= clear_count + 1'b1;
                    end else if (!stable_object) begin
                        state <= 0;
                        clear_count <= 0;
                    end
                    else if (finger) clear_count <= 0;
                    else if (clear_count + 1 >= CLEAR_FRAMES) begin
                        request <= 1;
                        state <= 3;
                    end
                    else clear_count <= clear_count + 1'b1;
                end
                default : begin
                end
            endcase
        end
    end
endmodule
