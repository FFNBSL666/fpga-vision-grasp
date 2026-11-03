// 量化网络计算：输入 3×96×96，依次执行卷积、池化与 10 通道检测头。
// 激活值使用 CHW 排列的 RAM，权重为 INT8，累加值为 INT32。
// 状态机复用乘加和存储资源；量化阈值、权重和输入映射来自原有 ROM。

`timescale 1ns / 1ps
// 为原有已训练的第3版 QONNX 网络提供原生 Verilog 计算后端。
// 网络顺序：3乘96乘96输入、16通道步长2卷积、池化、32通道步长2卷积、池化、64通道卷积、10通道检测头。
// 激活值按 CHW 排列存入 RAM，权重使用 INT8，累加器使用 INT32，
// 量化采用原样导出的整数阈值；本模块不实现机械运动。
module qnn_sequential_core #(
    parameter WEIGHTS_FILE = "qnn_weights.mem", THRESHOLDS_FILE = "qnn_thresholds.mem",
    parameter INPUT_FILE = "qnn_input.mem"
)(
    input wire clk, rst_n, start,
    input wire [23:0] rgb_data,
    input wire rgb_valid, rgb_last,
    output wire rgb_ready, busy,
    output reg error, done,
    output reg head_valid, head_last,
    output reg [8:0] head_index,
    output reg signed [31:0] head_acc
);

    // 计算阶段：装入 RGB → 初始化网络层 → 卷积乘加 → 量化 → 池化/检测头输出。
    localparam IDLE = 0, LOAD_WAIT = 1, LOAD_R = 2, LOAD_G = 3, LOAD_B = 4, INIT_LAYER = 5,
    PIXEL_BEGIN = 6, READ_ADDR = 7, READ_WAIT = 8, MAC = 9, Q_INIT = 10, Q_ADDR = 11,
    Q_WAIT = 12, Q_COMPARE = 13, ACT_STORE = 14, POOL_ADDR = 15, POOL_WAIT = 16,
    POOL_MAX = 17, POOL_STORE = 18, HEAD_STORE = 19;
    reg [4:0] state;
    reg [1:0] layer, kx, ky, pool_index;
    reg [5:0] oc, ic, ox, oy;
    reg [7:0] pool_max, qvalue, qlo, qhi;
    reg [13:0] input_pixel;
    reg [23:0] saved_rgb;
    reg saved_last, padding;
    reg signed [31:0] accumulator;
    reg [15:0] read_addr;
    reg [14:0] weight_addr;
    reg [9:0] threshold_addr;
    wire signed [7:0] weight_data;
    wire [31:0] threshold_data;
    wire [7:0] a_data, b_data;
    reg a_we, b_we;
    reg [15:0] a_waddr, b_waddr;
    reg [7:0] a_wdata, b_wdata, input_byte;
    (* rom_style = "distributed" *) reg [7:0] input_lut[0:255];
    initial $readmemh(INPUT_FILE, input_lut);
    integer in_channels, out_channels, out_dim, pool_dim, in_dim, stride, pad;
    integer sx, sy, computed_read, computed_weight, computed_output, computed_pool_read, computed_pool_write;
    wire [8:0] qsum = {1'b0, qlo} + {1'b0, qhi};
    wire [7:0] qmid = qsum[8:1];
    wire [7:0] selected_pixel = (layer == 3) ? b_data : a_data;

    // 激活值按无符号扩展，权重按有符号扩展，保持原有整数乘加语义。
    wire signed [31:0] product = $signed({1'b0, selected_pixel}) * $signed(weight_data);
    assign rgb_ready = (state == LOAD_WAIT);
    assign busy = (state != IDLE);
    qnn_ram8 #(
        .ADDR_BITS(15)
    ) ram_a (
        .clk(clk),
        .we(a_we),
        .waddr(a_waddr[14:0]),
        .raddr(read_addr[14:0]),
        .wdata(a_wdata),
        .rdata(a_data)
    );
    qnn_ram8 #(
        .ADDR_BITS(16)
    ) ram_b (
        .clk(clk),
        .we(b_we),
        .waddr(b_waddr),
        .raddr(read_addr),
        .wdata(b_wdata),
        .rdata(b_data)
    );
    qnn_parameter_rom #(
        .WEIGHTS_FILE(WEIGHTS_FILE),
        .THRESHOLDS_FILE(THRESHOLDS_FILE)
    ) params (
        .clk(clk),
        .weight_addr(weight_addr),
        .threshold_addr(threshold_addr),
        .weight_data(weight_data),
        .threshold_data(threshold_data)
    );

    // 组合地址计算：按层选择特征尺寸、卷积参数及原权重/激活 RAM 偏移。
    always @* begin
        case (layer)
            0 : begin
                in_channels = 3;
                out_channels = 16;
                in_dim = 96;
                out_dim = 48;
                pool_dim = 24;
                stride = 2;
                pad = 1;
            end
            1 : begin
                in_channels = 16;
                out_channels = 32;
                in_dim = 24;
                out_dim = 12;
                pool_dim = 6;
                stride = 2;
                pad = 1;
            end
            2 : begin
                in_channels = 32;
                out_channels = 64;
                in_dim = 6;
                out_dim = 6;
                pool_dim = 0;
                stride = 1;
                pad = 1;
            end
            default : begin
                in_channels = 64;
                out_channels = 10;
                in_dim = 6;
                out_dim = 6;
                pool_dim = 0;
                stride = 1;
                pad = 0;
            end
        endcase
        sx = ox * stride + kx - pad;
        sy = oy * stride + ky - pad;
        case (layer)
            0 : begin
                computed_read = ic * 9216 + sy * 96 + sx;
                computed_weight = oc * 27 + ic * 9 + ky * 3 + kx;
                computed_output = oc * 2304 + oy * 48 + ox;
            end
            1 : begin
                computed_read = ic * 576 + sy * 24 + sx;
                computed_weight = 432 + oc * 144 + ic * 9 + ky * 3 + kx;
                computed_output = oc * 144 + oy * 12 + ox;
            end
            2 : begin
                computed_read = ic * 36 + sy * 6 + sx;
                computed_weight = 5040 + oc * 288 + ic * 9 + ky * 3 + kx;
                computed_output = oc * 36 + oy * 6 + ox;
            end
            default : begin
                computed_read = ic * 36 + oy * 6 + ox;
                computed_weight = 23472 + oc * 64 + ic;
                computed_output = oc * 36 + oy * 6 + ox;
            end
        endcase
        if (layer == 0) begin
            computed_pool_read = oc * 2304 + (oy * 2 + pool_index[1]) * 48 + ox * 2 + pool_index[0];
            computed_pool_write = oc * 576 + oy * 24 + ox;
        end else begin
            computed_pool_read = oc * 144 + (oy * 2 + pool_index[1]) * 12 + ox * 2 + pool_index[0];
            computed_pool_write = oc * 36 + oy * 6 + ox;
        end
        a_we = 0;
        b_we = 0;
        a_waddr = 0;
        b_waddr = computed_output;
        a_wdata = 0;
        b_wdata = qvalue;
        input_byte = (state == LOAD_R) ? saved_rgb[23:16] : (state == LOAD_G) ? saved_rgb[15:8] : saved_rgb[7:0];
        if (state == LOAD_R || state == LOAD_G || state == LOAD_B) begin
            a_we = rst_n;
            a_waddr = input_pixel + ((state == LOAD_R) ? 0 : (state == LOAD_G) ? 9216 : 18432);
            a_wdata = input_lut[input_byte];
        end
        if (state == ACT_STORE) b_we = rst_n;
        if (state == POOL_STORE) begin
            a_we = rst_n;
            a_waddr = computed_pool_write;
            a_wdata = pool_max;
        end
    end

    // 完成当前输出后依次推进 X、Y、输出通道，不改变 CHW 存储顺序。
    task advance_output;
        begin
            if (ox == out_dim - 1) begin
                ox <= 0;
                if (oy == out_dim - 1) begin
                    oy <= 0;
                    oc <= oc + 1'b1;
                end
                else oy <= oy + 1'b1;
            end else ox <= ox + 1'b1;
            state <= PIXEL_BEGIN;
        end
    endtask
    always @(posedge clk) begin
        if (!rst_n) begin
            state <= IDLE;
            layer <= 0;
            oc <= 0;
            ic <= 0;
            ox <= 0;
            oy <= 0;
            kx <= 0;
            ky <= 0;
            pool_index <= 0;
            pool_max <= 0;
            qvalue <= 0;
            qlo <= 0;
            qhi <= 0;
            input_pixel <= 0;
            saved_rgb <= 0;
            saved_last <= 0;
            padding <= 0;
            accumulator <= 0;
            read_addr <= 0;
            weight_addr <= 0;
            threshold_addr <= 0;
            error <= 0;
            done <= 0;
            head_valid <= 0;
            head_last <= 0;
            head_index <= 0;
            head_acc <= 0;
        end else begin
            done <= 0;
            head_valid <= 0;
            head_last <= 0;
            case (state)
                IDLE : if (start) begin
                    state <= LOAD_WAIT;
                    input_pixel <= 0;
                    error <= 0;
                    layer <= 0;
                end
                LOAD_WAIT : if (rgb_valid) begin
                    saved_rgb <= rgb_data;
                    saved_last <= rgb_last;
                    state <= LOAD_R;
                end
                LOAD_R : state <= LOAD_G;
                LOAD_G : state <= LOAD_B;
                LOAD_B : begin
                    if (saved_last != (input_pixel == 9215)) begin
                        error <= 1;
                        state <= IDLE;
                    end
                    else if (input_pixel == 9215) state <= INIT_LAYER;
                    else begin
                        input_pixel <= input_pixel + 1'b1;
                        state <= LOAD_WAIT;
                    end
                end
                INIT_LAYER : begin
                    oc <= 0;
                    ox <= 0;
                    oy <= 0;
                    state <= PIXEL_BEGIN;
                end
                PIXEL_BEGIN : begin
                    accumulator <= 0;
                    ic <= 0;
                    kx <= 0;
                    ky <= 0;
                    state <= READ_ADDR;
                end

                // 同步存储器先提交地址，再等待读出，之后才进行一次乘加。
                READ_ADDR : begin
                    padding <= (sx < 0 || sy < 0 || sx >= in_dim || sy >= in_dim);
                    read_addr <= (sx < 0 || sy < 0 || sx >= in_dim || sy >= in_dim) ? 16'd0 : computed_read;
                    weight_addr <= computed_weight;
                    state <= READ_WAIT;
                end
                READ_WAIT : state <= MAC;
                MAC : begin
                    if (!padding) accumulator <= accumulator + product;
                    if ((layer == 3 || (kx == 2 && ky == 2)) && ic == in_channels - 1)
                    state <= (layer == 3) ? HEAD_STORE : Q_INIT;
                    else begin
                        if (layer == 3) ic <= ic + 1'b1;
                        else if (kx == 2) begin
                            kx <= 0;
                            if (ky == 2) begin
                                ky <= 0;
                                ic <= ic + 1'b1;
                            end else ky <= ky + 1'b1;
                        end
                        else kx <= kx + 1'b1;
                        state <= READ_ADDR;
                    end
                end

                // 使用导出阈值二分查找量化激活值，保持原训练参数对应的整数计算。
                Q_INIT : begin
                    qlo <= 0;
                    qhi <= 255;
                    state <= Q_ADDR;
                end
                Q_ADDR : begin
                    if (qlo == qhi) begin
                        qvalue <= qlo;
                        state <= ACT_STORE;
                    end
                    else begin
                        threshold_addr <= {layer, qmid};
                        state <= Q_WAIT;
                    end
                end
                Q_WAIT : state <= Q_COMPARE;
                Q_COMPARE : begin
                    if (accumulator >= $signed(threshold_data)) qlo <= qmid + 1'b1;
                    else qhi <= qmid;
                    state <= Q_ADDR;
                end
                ACT_STORE : begin
                    if (oc == out_channels - 1 && oy == out_dim - 1 && ox == out_dim - 1) begin
                        oc <= 0;
                        ox <= 0;
                        oy <= 0;
                        if (layer < 2) begin
                            pool_index <= 0;
                            pool_max <= 0;
                            state <= POOL_ADDR;
                        end
                        else begin
                            layer <= 3;
                            state <= INIT_LAYER;
                        end
                    end else advance_output;
                end

                // 前两层卷积后执行 2×2 最大池化，再送入下一层。
                POOL_ADDR : begin
                    read_addr <= computed_pool_read;
                    state <= POOL_WAIT;
                end
                POOL_WAIT : state <= POOL_MAX;
                POOL_MAX : begin
                    if (pool_index == 0 || b_data > pool_max) pool_max <= b_data;
                    if (pool_index == 3) state <= POOL_STORE;
                    else begin
                        pool_index <= pool_index + 1'b1;
                        state <= POOL_ADDR;
                    end
                end
                POOL_STORE : begin
                    pool_index <= 0;
                    pool_max <= 0;
                    if (oc == out_channels - 1 && oy == pool_dim - 1 && ox == pool_dim - 1) begin
                        layer <= layer + 1'b1;
                        state <= INIT_LAYER;
                    end
                    else begin
                        if (ox == pool_dim - 1) begin
                            ox <= 0;
                            if (oy == pool_dim - 1) begin
                                oy <= 0;
                                oc <= oc + 1'b1;
                            end else oy <= oy + 1'b1;
                        end
                        else ox <= ox + 1'b1;
                        state <= POOL_ADDR;
                    end
                end

                // 检测头输出原始累加值与索引；最后一个值同时产生 head_last。
                HEAD_STORE : begin
                    head_valid <= 1;
                    head_index <= computed_output;
                    head_acc <= accumulator;
                    if (oc == 9 && oy == 5 && ox == 5) begin
                        head_last <= 1;
                        done <= 1;
                        state <= IDLE;
                    end
                    else advance_output;
                end
                default : begin
                    state <= IDLE;
                    error <= 1;
                end
            endcase
        end
    end
endmodule
