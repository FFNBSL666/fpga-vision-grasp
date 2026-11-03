// 模块：抓取主流程（grab_fsm）
// 作用：按接近、合爪、抬升、观察、放置、松爪、回零推进。
// 输入：目标请求、动作发送完成、计时、故障和观察结果。
// 输出：动作编号、动作启动、忙、观察使能和完成。
// 注意：到位暂按等待时间判断；故障后保持，复位才重新开始。
// 阅读：每行右侧说明这行代码在做什么。

`timescale 1ns / 1ps // 仿真时间单位1ns，最小精度1ps；不产生硬件时钟。
module grab_fsm #( // 定义这个模块；括号中写参数或输入输出接口。
    parameter integer MONITOR_MS  = 2000, // 设置观察毫秒数；这些常量在综合前确定。
    parameter integer COOLDOWN_MS = 1000 // 设置冷却等待毫秒数；这些常量在综合前确定。
) ( // 结束参数列表，开始写输入输出接口。
    input wire clk, // 声明接口：时钟。
    input wire rst_n, // 声明接口：内部低电平复位。
    input wire tick, // 声明接口：毫秒脉冲。
    input wire arm_enable, // 声明接口：流程允许开关。
    input wire fault, // 声明接口：故障锁。
    input wire request_valid, // 声明接口：抓取请求。
    input wire [1:0] target_region, // 声明接口：物体所在区域。
    output reg pose_start, // 声明接口：动作启动脉冲。
    output reg [2:0] pose_id, // 声明接口：动作编号。
    output reg [1:0] region, // 声明接口：目标区域。
    input wire pose_done, // 声明接口：六关节发送完成脉冲。
    input wire [15:0] pose_time, // 声明接口：动作等待时间。
    input wire monitor_checked, // 声明接口：已观察两帧的标志。
    output wire monitor_enable, // 声明接口：观察模块使能。
    output wire busy, // 声明接口：忙标志。
    output reg cycle_done, // 声明接口：取放流程完成脉冲。
    output reg monitor_timeout, // 声明接口：观察不足超时脉冲。
    output reg [3:0] state_debug // 声明接口：当前流程状态。
); // 结束输入输出接口定义。
    localparam [3:0] IDLE = 0; // 给这些步骤编号：IDLE为等待状态。
    localparam [3:0] ISSUE = 1; // 给这些步骤编号：ISSUE为启动动作状态。
    localparam [3:0] WAIT_TX = 2; // 给这些步骤编号：WAIT_TX为等待六关节发完。
    localparam [3:0] WAIT_MOVE = 3; // 给这些步骤编号：WAIT_MOVE为等待运动时间。
    localparam [3:0] MONITOR = 4; // 给这些步骤编号：MONITOR为观察状态。
    localparam [3:0] COOLDOWN = 5; // 给这些步骤编号：COOLDOWN为冷却等待状态。
    localparam [3:0] HOLD = 6; // 给这些步骤编号：HOLD为故障保持状态。
    reg [31:0] elapsed; // 声明32位寄存器：已等待毫秒数。
    reg [15:0] latched_pose_time; // 声明16位寄存器：启动时保存的动作时间。
    assign monitor_enable = (state_debug == MONITOR); // 只有观察状态才打开监测接口。
    assign busy = (state_debug != IDLE); // 不是等待状态就显示忙，故障保持也算忙。
    always @(posedge clk) begin // 每个时钟上升沿更新下面的数据。
        if (!rst_n) begin // 判断内部低电平复位为0；满足条件才进入后面的步骤。
            state_debug <= IDLE; // 当前流程状态设为等待状态。
            elapsed <= 0; // 已等待毫秒数设为0。
            latched_pose_time <= 0; // 启动时保存的动作时间设为0。
            pose_start <= 0; // 动作启动脉冲设为0。
            pose_id <= 0; // 动作编号设为0。
            region <= 1; // 目标区域设为1。
            cycle_done <= 0; // 取放流程完成脉冲设为0。
            monitor_timeout <= 0; // 观察不足超时脉冲设为0。
        end // 结束当前代码块。
        else begin // 前一个条件不满足时，执行下面这一块。
            pose_start <= 0; // 动作启动脉冲设为0。
            cycle_done <= 0; // 取放流程完成脉冲设为0。
            monitor_timeout <= 0; // 观察不足超时脉冲设为0。
            if (fault || (!arm_enable && state_debug != IDLE)) begin // 判断故障锁 || (!流程允许开关 && 当前流程状态 != 等待状态)；满足条件才进入后面的步骤。
                state_debug <= HOLD; // 当前流程状态设为故障保持状态。
            end // 结束当前代码块。
            else // 前一个条件不满足时，执行下一行或下一块。
                case (state_debug) // 根据抓取流程状态，选择下一步。
                    IDLE: // 进入等待状态分支。
                    if (arm_enable && request_valid) begin // 判断流程允许开关 && 抓取请求；满足条件才进入后面的步骤。
                        region <= target_region; // 目标区域设为物体所在区域。
                        pose_id <= 1; // 动作编号设为1。
                        state_debug <= ISSUE; // 当前流程状态设为启动动作状态。
                    end // 结束当前代码块。
                    ISSUE: begin // 处于启动动作状态时执行；开始该步骤。
                        latched_pose_time <= pose_time; // 启动时保存的动作时间设为动作等待时间。
                        pose_start <= 1; // 动作启动脉冲设为1。
                        state_debug <= WAIT_TX; // 当前流程状态设为等待六关节发完。
                    end // 结束当前代码块。
                    WAIT_TX: // 进入等待六关节发完分支。
                    if (pose_done) begin // 判断六关节发送完成脉冲；满足条件才进入后面的步骤。
                        elapsed <= 0; // 已等待毫秒数设为0。
                        state_debug <= WAIT_MOVE; // 当前流程状态设为等待运动时间。
                    end // 结束当前代码块。
                    WAIT_MOVE: // 进入等待运动时间分支。
                    if (tick) begin // 判断毫秒脉冲；满足条件才进入后面的步骤。
                        if ((latched_pose_time==0 || elapsed+1>=latched_pose_time)) begin // 判断(启动时保存的动作时间==0 || 已等待毫秒数+1>=启动时保存的动作时间)；满足条件才进入后面的步骤。
                            elapsed <= 0; // 已等待毫秒数设为0。
                            if (pose_id == 3) state_debug <= MONITOR; // 判断动作编号等于3；当前流程状态设为观察状态。
                            else if (pose_id == 0) begin // 判断动作编号等于0；满足条件才进入后面的步骤。
                                cycle_done  <= 1; // 取放流程完成脉冲设为1。
                                state_debug <= COOLDOWN; // 当前流程状态设为冷却等待状态。
                            end // 结束当前代码块。
                            else begin // 前一个条件不满足时，执行下面这一块。
                                pose_id <= pose_id == 5 ? 3'd0 : pose_id + 1'b1; // 动作编号设为动作编号 == 5 ? 3'd0 : 动作编号 + 1'b1。
                                state_debug <= ISSUE; // 当前流程状态设为启动动作状态。
                            end // 结束当前代码块。
                        end // 结束当前代码块。
                        else elapsed <= elapsed + 1'b1; // 否则，已等待毫秒数设为已等待毫秒数加1。
                    end // 结束当前代码块。
                    MONITOR: // 进入观察状态分支。
                    if (tick) begin // 判断毫秒脉冲；满足条件才进入后面的步骤。
                        if ((MONITOR_MS == 0 || elapsed + 1 >= MONITOR_MS)) begin // 判断(观察毫秒数 == 0 || 已等待毫秒数 + 1 >= 观察毫秒数)；满足条件才进入后面的步骤。
                            if (monitor_checked) begin // 判断已观察两帧的标志；满足条件才进入后面的步骤。
                                pose_id <= 4; // 动作编号设为4。
                                state_debug <= ISSUE; // 当前流程状态设为启动动作状态。
                            end // 结束当前代码块。
                            else begin // 前一个条件不满足时，执行下面这一块。
                                monitor_timeout <= 1; // 观察不足超时脉冲设为1。
                                state_debug <= HOLD; // 当前流程状态设为故障保持状态。
                            end // 结束当前代码块。
                            elapsed <= 0; // 已等待毫秒数设为0。
                        end // 结束当前代码块。
                        else elapsed <= elapsed + 1'b1; // 否则，已等待毫秒数设为已等待毫秒数加1。
                    end // 结束当前代码块。
                    COOLDOWN: // 进入冷却等待状态分支。
                    if (tick) begin // 判断毫秒脉冲；满足条件才进入后面的步骤。
                        if ((COOLDOWN_MS == 0 || elapsed + 1 >= COOLDOWN_MS)) begin // 判断(冷却等待毫秒数 == 0 || 已等待毫秒数 + 1 >= 冷却等待毫秒数)；满足条件才进入后面的步骤。
                            elapsed <= 0; // 已等待毫秒数设为0。
                            state_debug <= IDLE; // 当前流程状态设为等待状态。
                        end // 结束当前代码块。
                        else elapsed <= elapsed + 1'b1; // 否则，已等待毫秒数设为已等待毫秒数加1。
                    end // 结束当前代码块。
                    HOLD: state_debug <= HOLD; // 当前流程状态设为故障保持状态。
                    default: state_debug <= HOLD; // 未匹配任何分支时，回到预设状态。
                endcase // 结束状态或编号分支。
        end // 结束当前代码块。
    end // 结束当前代码块。
endmodule // 本模块结束。
