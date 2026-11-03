// 像素到姿态查表：把 96×96 图像坐标映射为接近、下降、合爪、抬升姿态。
// 镜像仅在原有查表地址处执行 X 反向；ROM 有效位决定该点能否使用。
// 姿态最低 16 位是 0 号底座 P 值，最高 16 位是 5 号夹爪 P 值。

`timescale 1ns/1ps
// 每个像素有独立有效位，未标定或不可达的像素保持无效。
// 离线生成过程完成平面标定和逆运动学计算；运行时由 FPGA 独立查表。
module pixel_pose_lookup #(
    parameter MEM_FILE = "workspace_pose_lut.mem"
)(
    input wire clk, rst_n, request,
    input wire [7:0] pixel_x, pixel_y,
    output reg response, valid,
    output wire [95:0] approach_pose, descent_pose, grip_pose, lift_pose
);
    (* rom_style = "block" *) reg [128:0] memory[0:9215];
    reg [128:0] word;
    reg pending, in_range;
    initial $readmemh(MEM_FILE, memory);
    // 选定的工作区坐标关于机械臂 Y 轴镜像：X 取反，Y 保持不变。
    // 站在机械臂后方朝工作区看，右侧为机械臂 X 正方向。
    // 保持原始图像检测结果及全部舵机姿态 ROM 数据不变。

    // 沿用原有 Y 轴镜像：X 像素取 95-x；不要在其他层重复反转。
    wire [7:0] robot_pixel_x = 8'd95 - pixel_x;
    wire [13:0] address = pixel_y * 14'd96 + robot_pixel_x;
    wire [15:0] a0 = {4'd0, word[11:0]}, a1 = {4'd0, word[23:12]};
    wire [15:0] a2 = {4'd0, word[35:24]}, a3 = {4'd0, word[47:36]};
    wire [15:0] d0 = {4'd0, word[59:48]}, d1 = {4'd0, word[71:60]};
    wire [15:0] d2 = {4'd0, word[83:72]}, d3 = {4'd0, word[95:84]};

    // 六个 P 值按 5→0 号打包；夹爪张开为 1300、合拢为 1700，均沿用原值。
    assign approach_pose = {16'd1300, 16'd1500, a3, a2, a1, a0};
    assign descent_pose = {16'd1300, 16'd1500, d3, d2, d1, d0};
    assign grip_pose = {16'd1700, 16'd1500, d3, d2, d1, d0};
    assign lift_pose = {16'd1700, 16'd1500, a3, a2, a1, a0};
    // 同步 ROM 读取逻辑与可复位寄存器分开实现。
    always @(posedge clk) if (request && pixel_x < 96 && pixel_y < 96) word <= memory[address];

    // 用 pending 对齐同步 ROM 延迟，response 与有效位在对应周期输出。
    always @(posedge clk) begin
        if (!rst_n) begin
            pending <= 0;
            response <= 0;
            valid <= 0;
            in_range <= 0;
        end
        else begin
            pending <= request;
            if (request) in_range <= pixel_x < 96 && pixel_y < 96;
            response <= pending;
            valid <= pending && in_range && word[128];
        end
    end
endmodule
