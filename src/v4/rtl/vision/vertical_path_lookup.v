// 垂直路径查表：先由像素找到径向分组，再按高度读出 1～3 号关节目标。
// 高度单位为毫米；两个 ROM 均同步读取，主控需等待相应读取周期。

`timescale 1ns/1ps
// 离线根据 CAD 进行逆运动学计算，同一2毫米径向分组内的像素共用路径。
// 运行期间的像素和路径查表均由 FPGA 完成，不需要电脑干预。
module vertical_path_lookup(
    input wire clk,
    input wire [15:0] pixel,
    input wire [6:0] height_mm,
    output reg [35:0] pose,
    output wire valid
);
    (* rom_style = "block" *) reg [6:0] radius_memory[0:9215];
    (* rom_style = "block" *) reg [35:0] path_memory[0:8553];
    reg [6:0] radius_id;
    initial begin
        $readmemh("pixel_radius_lut.mem", radius_memory);
        $readmemh("vertical_path_lut.mem", path_memory);
    end

    // 与平面姿态查表使用同一镜像约定，维持原有坐标方向。
    wire [7:0] robot_pixel_x = 8'd95 - pixel[7:0];
    wire [13:0] pixel_address = pixel[15:8] * 14'd96 + robot_pixel_x;

    // 每个径向分组包含 0～90 mm 共 91 个高度条目。
    wire [13:0] path_address = radius_id * 14'd91 + height_mm;
    always @(posedge clk) begin
        if (pixel[15:8] < 96 && pixel[7:0] < 96) radius_id <= radius_memory[pixel_address];
        if (radius_id < 94 && height_mm <= 90) pose <= path_memory[path_address];
    end
    assign valid = pixel[15:8] < 96 && pixel[7:0] < 96 && radius_id < 94 && height_mm <= 90;
endmodule
