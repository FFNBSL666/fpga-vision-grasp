// 检测结果解码：在 6×6 网格中分别找出手指和物体的最佳候选。
// 按原有整数阈值及 Q24 仿射/Sigmoid 查表还原置信度和 0～95 像素坐标。
// result_valid 表示完整结果可用，具体是否选点由上层选择器决定。

`timescale 1ns / 1ps
// 解码已训练的第3版网格相对坐标检测头；整数置信度阈值保持精确，
// 通过 Q24 仿射变换与 Sigmoid 查表，生成明确的8位输出值。
module qnn_result_decode #(
    parameter AFFINE_FILE = "qnn_affine.mem", SIGMOID_FILE = "qnn_sigmoid.mem"
)(
    input wire clk, rst_n, head_valid, head_last,
    input wire [8:0] head_index,
    input wire signed [31:0] head_acc,
    output wire busy,
    output reg result_valid, error,
    output reg finger_valid, object_valid,
    output reg [7:0] finger_confidence, finger_x, finger_y, object_confidence, object_x, object_y
);
    localparam IDLE = 0, T0_ADDR = 1, T0_WAIT = 2, T0_READ = 3, T1_WAIT = 4, T1_READ = 5,
    READ_ADDR = 6, READ_WAIT = 7, READ_VALUE = 8, BIAS_WAIT = 9, BIAS_READ = 10,
    MULTIPLY = 11, LUT_ADDR = 12, LUT_WAIT = 13, LUT_READ = 14;
    reg [3:0] state;
    (* ram_style = "block" *) reg signed [31:0] head_ram[0:511];
    (* rom_style = "distributed" *) reg [63:0] affine_rom[0:31];
    (* rom_style = "block" *) reg [7:0] sigmoid_rom[0:511];
    initial begin
        $readmemh(AFFINE_FILE, affine_rom);
        $readmemh(SIGMOID_FILE, sigmoid_rom);
    end
    reg [8:0] read_addr;
    reg signed [31:0] read_data, best_finger, best_object, selected_acc, multiplier;
    reg [5:0] best_finger_cell, best_object_cell;
    reg [4:0] affine_addr;
    reg signed [63:0] affine_data, bias, product;
    reg [8:0] sigmoid_addr;
    reg [7:0] sigmoid_data;
    reg [2:0] item;
    reg [8:0] expected_index;
    integer channel, grid_cell;
    reg signed [63:0] scaled_index;
    assign busy = (state != IDLE);

    // 缓存检测头与同步读取 ROM，控制状态机另行等待读取延迟。
    always @(posedge clk) begin
        if (head_valid) head_ram[head_index] <= head_acc;
        read_data <= head_ram[read_addr];
        affine_data <= affine_rom[affine_addr];
        sigmoid_data <= sigmoid_rom[sigmoid_addr];
    end

    // 依次解码手指置信度/X/Y、物体置信度/X/Y，使用各自最佳网格。
    always @* begin
        case (item)
            0 : channel = 0;
            1 : channel = 1;
            2 : channel = 2;
            3 : channel = 5;
            4 : channel = 6;
            default : channel = 7;
        endcase
        grid_cell = (item < 3) ? best_finger_cell : best_object_cell;
        scaled_index = ((product + bias + 64'sd8388608) >>> 24) + 64'sd256;
    end
    always @(posedge clk) begin
        if (!rst_n) begin
            state <= IDLE;
            read_addr <= 0;
            affine_addr <= 0;
            sigmoid_addr <= 0;
            best_finger <= 0;
            best_object <= 0;
            best_finger_cell <= 0;
            best_object_cell <= 0;
            selected_acc <= 0;
            multiplier <= 0;
            bias <= 0;
            product <= 0;
            item <= 0;
            expected_index <= 0;
            result_valid <= 0;
            error <= 0;
            finger_valid <= 0;
            object_valid <= 0;
            finger_confidence <= 0;
            finger_x <= 0;
            finger_y <= 0;
            object_confidence <= 0;
            object_x <= 0;
            object_y <= 0;
        end else begin
            result_valid <= 0;

            // 检查检测头索引连续性，同时保存两类目标的最大置信度累加值。
            if (head_valid) begin
                if (head_index == 0) begin
                    expected_index <= 1;
                    error <= 0;
                end
                else if (head_index != expected_index) error <= 1;
                else expected_index <= expected_index + 1'b1;
                if (head_index < 36 && (head_index == 0 || head_acc > best_finger)) begin
                    best_finger <= head_acc;
                    best_finger_cell <= head_index;
                end
                if (head_index >= 180 && head_index < 216 && (head_index == 180 || head_acc > best_object)) begin
                    best_object <= head_acc;
                    best_object_cell <= head_index - 180;
                end
            end
            case (state)
                IDLE : if (head_valid && head_last) begin
                    if (head_index != 359) error <= 1;
                    else begin
                        affine_addr <= 20;
                        state <= T0_WAIT;
                    end
                end
                T0_WAIT : state <= T0_READ;
                T0_READ : begin
                    finger_valid <= best_finger >= $signed(affine_data[31:0]);
                    affine_addr <= 21;
                    state <= T1_WAIT;
                end
                T1_WAIT : state <= T1_READ;
                T1_READ : begin
                    object_valid <= best_object >= $signed(affine_data[31:0]);
                    item <= 0;
                    state <= READ_ADDR;
                end
                READ_ADDR : begin
                    read_addr <= channel * 36 + grid_cell;
                    affine_addr <= channel;
                    state <= READ_WAIT;
                end
                READ_WAIT : state <= READ_VALUE;
                READ_VALUE : begin
                    selected_acc <= read_data;
                    multiplier <= affine_data[31:0];
                    affine_addr <= 10 + channel;
                    state <= BIAS_WAIT;
                end
                BIAS_WAIT : state <= BIAS_READ;
                BIAS_READ : begin
                    bias <= affine_data;
                    state <= MULTIPLY;
                end
                MULTIPLY : begin
                    product <= selected_acc * multiplier;
                    state <= LUT_ADDR;
                end
                LUT_ADDR : begin
                    if (scaled_index < 0) sigmoid_addr <= 0;
                    else if (scaled_index > 511) sigmoid_addr <= 511;
                    else sigmoid_addr <= scaled_index[8:0];
                    state <= LUT_WAIT;
                end
                LUT_WAIT : state <= LUT_READ;

                // 结合 6×6 网格位置和单格内偏移，形成 96×96 图像坐标。
                LUT_READ : begin
                    case (item)
                        0 : finger_confidence <= sigmoid_data;
                        1 : finger_x <= (grid_cell % 6) * 16 + sigmoid_data[7:4];
                        2 : finger_y <= (grid_cell / 6) * 16 + sigmoid_data[7:4];
                        3 : object_confidence <= sigmoid_data;
                        4 : object_x <= (grid_cell % 6) * 16 + sigmoid_data[7:4];
                        5 : object_y <= (grid_cell / 6) * 16 + sigmoid_data[7:4];
                    endcase
                    if (item == 5) begin
                        result_valid <= !error;
                        state <= IDLE;
                    end
                    else begin
                        item <= item + 1'b1;
                        state <= READ_ADDR;
                    end
                end
                default : begin
                    state <= IDLE;
                    error <= 1;
                end
            endcase
        end
    end
endmodule
