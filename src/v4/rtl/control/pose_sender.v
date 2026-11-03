// 模块：六关节发送顺序（pose_sender）
// 作用：保存动作数据，然后依次交给舵机总线发送。
// 输入：动作启动、6个位置、时间、总线握手。
// 输出：当前关节ID、位置、时间、忙与完成。
// 注意：完成仅表示发完6条指令，不表示关节已经到位。
// 阅读：每行右侧说明这行代码在做什么。

`timescale 1ns / 1ps // 仿真时间单位1ns，最小精度1ps；不产生硬件时钟。
module pose_sender #( // 定义这个模块；括号中写参数或输入输出接口。
    parameter integer ID_BASE = 0 // 设置第一个舵机ID；这些常量在综合前确定。
) ( // 结束参数列表，开始写输入输出接口。
    input wire clk, // 声明接口：时钟。
    input wire rst_n, // 声明接口：内部低电平复位。
    input wire start, // 声明接口：启动脉冲。
    input wire [95:0] positions, // 声明接口：6个关节打包位置。
    input wire [15:0] duration_ms, // 声明接口：运动时间。
    output wire cmd_valid, // 声明接口：指令有效标志。
    input wire cmd_ready, // 声明接口：总线可接收标志。
    output wire [7:0] cmd_id, // 声明接口：目标舵机ID。
    output wire [15:0] cmd_position, // 声明接口：目标舵机位置。
    output wire [15:0] cmd_time, // 声明接口：目标运动时间。
    input wire cmd_done, // 声明接口：报文发完脉冲。
    output reg busy, // 声明接口：忙标志。
    output reg done // 声明接口：完成脉冲。
); // 结束输入输出接口定义。
    localparam [1:0] IDLE = 0; // 给这些步骤编号：IDLE为等待状态。
    localparam [1:0] SEND = 1; // 给这些步骤编号：SEND为发送状态。
    localparam [1:0] WAIT_DONE = 2; // 给这些步骤编号：WAIT_DONE为等待单关节发完。
    reg [ 1:0] state; // 声明寄存器：当前状态。
    reg [ 2:0] joint; // 声明寄存器：当前关节序号。
    reg [95:0] saved_positions; // 声明96位寄存器：保存的6个关节位置。
    reg [15:0] saved_time; // 声明16位寄存器：保存的运动时间。
    assign cmd_valid = (state == SEND); // 进入发送状态时，告诉总线这里有一条指令。
    assign cmd_id = ID_BASE + joint; // 基准ID加当前关节序号，得到目标舵机ID。
    assign cmd_position = saved_positions[15:0]; // 取最低16位，作为当前关节的位置值。
    assign cmd_time = saved_time; // 所有关节使用保存的运动时间。
    always @(posedge clk) begin // 每个时钟上升沿更新下面的数据。
        if (!rst_n) begin // 判断内部低电平复位为0；满足条件才进入后面的步骤。
            state <= IDLE; // 当前状态设为等待状态。
            joint <= 0; // 当前关节序号设为0。
            saved_positions <= 0; // 保存的6个关节位置设为0。
            saved_time <= 0; // 保存的运动时间设为0。
            busy <= 0; // 忙标志设为0。
            done <= 0; // 完成脉冲设为0。
        end // 结束当前代码块。
        else begin // 前一个条件不满足时，执行下面这一块。
            done <= 0; // 完成脉冲设为0。
            case (state) // 根据当前状态，选择下面的分支。
                IDLE: // 进入等待状态分支。
                if (start) begin // 判断启动脉冲；满足条件才进入后面的步骤。
                    saved_positions <= positions; // 保存的6个关节位置设为6个关节打包位置。
                    saved_time <= duration_ms; // 保存的运动时间设为运动时间。
                    joint <= 0; // 当前关节序号设为0。
                    busy <= 1; // 忙标志设为1。
                    state <= SEND; // 当前状态设为发送状态。
                end // 结束当前代码块。
                SEND: if (cmd_ready) state <= WAIT_DONE; // 判断总线可接收标志；当前状态设为等待单关节发完。
                WAIT_DONE: // 进入等待单关节发完分支。
                if (cmd_done) begin // 判断报文发完脉冲；满足条件才进入后面的步骤。
                    if (joint == 5) begin // 判断当前关节序号等于5；满足条件才进入后面的步骤。
                        done  <= 1; // 完成脉冲设为1。
                        busy  <= 0; // 忙标志设为0。
                        state <= IDLE; // 当前状态设为等待状态。
                    end // 结束当前代码块。
                    else begin // 前一个条件不满足时，执行下面这一块。
                        joint <= joint + 1'b1; // 当前关节序号设为当前关节序号加1。
                        saved_positions <= saved_positions >> 16; // 保存的6个关节位置设为保存的6个关节位置 >> 16。
                        state <= SEND; // 当前状态设为发送状态。
                    end // 结束当前代码块。
                end // 结束当前代码块。
                default: state <= IDLE; // 未匹配任何分支时，回到预设状态。
            endcase // 结束状态或编号分支。
        end // 结束当前代码块。
    end // 结束当前代码块。
endmodule // 本模块结束。
