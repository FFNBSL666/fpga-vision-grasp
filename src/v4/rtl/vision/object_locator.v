// 模块：物体中心定位（object_locator）
// 作用：累计画面下半幅的亮像素，再用坐标和除以像素数求中心。
// 输入：像素、坐标、帧开始、有效帧脉冲。
// 输出：是否找到物体、中心坐标、亮像素数量。
// 注意：这是灰度阈值识别，多个物体会合并成一个区域。
// 阅读：每行右侧说明这行代码在做什么。

`timescale 1ns / 1ps // 仿真时间单位1ns，最小精度1ps；不产生硬件时钟。
module object_locator #( // 定义这个模块；括号中写参数或输入输出接口。
    parameter integer IMAGE_H = 60, // 设置图像高度；这些常量在综合前确定。
    parameter integer THRESHOLD = 200, // 设置亮像素阈值；这些常量在综合前确定。
    parameter integer MIN_PIXELS = 4 // 设置最少亮像素数量；这些常量在综合前确定。
) ( // 结束参数列表，开始写输入输出接口。
    input wire clk, // 声明接口：时钟。
    input wire rst_n, // 声明接口：内部低电平复位。
    input wire frame_start, // 声明接口：开始一帧的脉冲。
    input wire pixel_valid, // 声明接口：像素有效标志。
    input wire [7:0] pixel, // 声明接口：灰度像素。
    input wire [7:0] x, // 声明接口：当前像素横坐标。
    input wire [7:0] y, // 声明接口：当前像素纵坐标。
    input wire frame_good, // 声明接口：完整好帧脉冲。
    output reg result_valid, // 声明接口：结果有效脉冲。
    output reg found, // 声明接口：找到物体标志。
    output reg [7:0] center_x, // 声明接口：物体中心横坐标。
    output reg [7:0] center_y, // 声明接口：物体中心纵坐标。
    output reg [23:0] area // 声明接口：亮像素数量。
); // 结束输入输出接口定义。
    localparam [2:0] IDLE = 0; // 给这些步骤编号：IDLE为等待状态。
    localparam [2:0] START_X = 1; // 给这些步骤编号：START_X为开始横坐标除法。
    localparam [2:0] WAIT_X = 2; // 给这些步骤编号：WAIT_X为等待横坐标结果。
    localparam [2:0] START_Y = 3; // 给这些步骤编号：START_Y为开始纵坐标除法。
    localparam [2:0] WAIT_Y = 4; // 给这些步骤编号：WAIT_Y为等待纵坐标结果。
    reg [2:0] state; // 声明3位寄存器：当前状态。
    reg [23:0] count; // 声明24位寄存器：物体亮像素总数。
    reg [23:0] sum_x; // 声明24位寄存器：横坐标总和。
    reg [23:0] sum_y; // 声明24位寄存器：纵坐标总和。
    reg [23:0] count_hold; // 声明24位寄存器：保存的亮像素数。
    reg [23:0] sx_hold; // 声明24位寄存器：保存的横坐标总和。
    reg [23:0] sy_hold; // 声明24位寄存器：保存的纵坐标总和。
    reg div_start; // 声明寄存器：除法启动脉冲。
    reg [23:0] div_num; // 声明24位寄存器：送入除法器的被除数。
    wire [23:0] div_result; // 声明24位连接信号：除法结果。
    wire div_busy; // 声明连接信号：除法忙标志。
    wire div_done; // 声明连接信号：除法完成脉冲。
    unsigned_divider #( // 创建分步除法模块，下面配置参数。
        .WIDTH(24) // 除法数据位数接24。
    ) divider ( // 结束参数设置，为这个实例起名并开始连接接口。
        .clk(clk), // 时钟接时钟。
        .rst_n(rst_n), // 内部低电平复位接内部低电平复位。
        .start(div_start), // 启动脉冲接除法启动脉冲。
        .numerator(div_num), // 被除数接送入除法器的被除数。
        .denominator(count_hold), // 除数接保存的亮像素数。
        .quotient(div_result), // 除法结果接除法结果。
        .busy(div_busy), // 忙标志接除法忙标志。
        .done(div_done) // 完成脉冲接除法完成脉冲。
    ); // 结束输入输出接口定义。
    always @(posedge clk) begin // 每个时钟上升沿更新下面的数据。
        if (!rst_n) begin // 判断内部低电平复位为0；满足条件才进入后面的步骤。
            count <= 0; // 物体亮像素总数设为0。
            sum_x <= 0; // 横坐标总和设为0。
            sum_y <= 0; // 纵坐标总和设为0。
            count_hold <= 0; // 保存的亮像素数设为0。
            sx_hold <= 0; // 保存的横坐标总和设为0。
            sy_hold <= 0; // 保存的纵坐标总和设为0。
            state <= IDLE; // 当前状态设为等待状态。
            div_start <= 0; // 除法启动脉冲设为0。
            div_num <= 0; // 送入除法器的被除数设为0。
            result_valid <= 0; // 结果有效脉冲设为0。
            found <= 0; // 找到物体标志设为0。
            center_x <= 0; // 物体中心横坐标设为0。
            center_y <= 0; // 物体中心纵坐标设为0。
            area <= 0; // 亮像素数量设为0。
        end // 结束当前代码块。
        else begin // 前一个条件不满足时，执行下面这一块。
            result_valid <= 0; // 结果有效脉冲设为0。
            div_start <= 0; // 除法启动脉冲设为0。
            if (frame_start) begin // 判断开始一帧的脉冲；满足条件才进入后面的步骤。
                count <= 0; // 物体亮像素总数设为0。
                sum_x <= 0; // 横坐标总和设为0。
                sum_y <= 0; // 纵坐标总和设为0。
            end // 结束当前代码块。
            else if (pixel_valid && y >= IMAGE_H / 2 && pixel >= THRESHOLD) begin // 判断像素有效标志 && 当前像素纵坐标 >= 图像高度 / 2 && 灰度像素 >= 亮像素阈值；满足条件才进入后面的步骤。
                count <= count + 1'b1; // 物体亮像素总数设为物体亮像素总数加1。
                sum_x <= sum_x + x; // 横坐标总和设为横坐标总和 + 当前像素横坐标。
                sum_y <= sum_y + y; // 纵坐标总和设为纵坐标总和 + 当前像素纵坐标。
            end // 结束当前代码块。
            case (state) // 根据当前状态，选择下面的分支。
                IDLE: // 进入等待状态分支。
                if (frame_good) begin // 判断完整好帧脉冲；满足条件才进入后面的步骤。
                    area <= count; // 亮像素数量设为物体亮像素总数。
                    if (count >= MIN_PIXELS) begin // 判断物体亮像素总数 >= 最少亮像素数量；满足条件才进入后面的步骤。
                        count_hold <= count; // 保存的亮像素数设为物体亮像素总数。
                        sx_hold <= sum_x; // 保存的横坐标总和设为横坐标总和。
                        sy_hold <= sum_y; // 保存的纵坐标总和设为纵坐标总和。
                        state <= START_X; // 当前状态设为开始横坐标除法。
                    end // 结束当前代码块。
                    else begin // 前一个条件不满足时，执行下面这一块。
                        found <= 0; // 找到物体标志设为0。
                        center_x <= 0; // 物体中心横坐标设为0。
                        center_y <= 0; // 物体中心纵坐标设为0。
                        result_valid <= 1; // 结果有效脉冲设为1。
                    end // 结束当前代码块。
                end // 结束当前代码块。
                START_X: begin // 处于开始横坐标除法时执行；开始该步骤。
                    div_num <= sx_hold; // 送入除法器的被除数设为保存的横坐标总和。
                    div_start <= 1; // 除法启动脉冲设为1。
                    state <= WAIT_X; // 当前状态设为等待横坐标结果。
                end // 结束当前代码块。
                WAIT_X: // 进入等待横坐标结果分支。
                if (div_done) begin // 判断除法完成脉冲；满足条件才进入后面的步骤。
                    center_x <= div_result[7:0]; // 物体中心横坐标设为除法结果[7:0]。
                    state <= START_Y; // 当前状态设为开始纵坐标除法。
                end // 结束当前代码块。
                START_Y: begin // 处于开始纵坐标除法时执行；开始该步骤。
                    div_num <= sy_hold; // 送入除法器的被除数设为保存的纵坐标总和。
                    div_start <= 1; // 除法启动脉冲设为1。
                    state <= WAIT_Y; // 当前状态设为等待纵坐标结果。
                end // 结束当前代码块。
                WAIT_Y: // 进入等待纵坐标结果分支。
                if (div_done) begin // 判断除法完成脉冲；满足条件才进入后面的步骤。
                    center_y <= div_result[7:0]; // 物体中心纵坐标设为除法结果[7:0]。
                    found <= 1; // 找到物体标志设为1。
                    result_valid <= 1; // 结果有效脉冲设为1。
                    state <= IDLE; // 当前状态设为等待状态。
                end // 结束当前代码块。
                default: state <= IDLE; // 未匹配任何分支时，回到预设状态。
            endcase // 结束状态或编号分支。
        end // 结束当前代码块。
    end // 结束当前代码块。
endmodule // 本模块结束。
