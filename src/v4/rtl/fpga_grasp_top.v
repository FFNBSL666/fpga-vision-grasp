// 阅读提示：此模块属于保留的旧流程，是否接入由板级 generate 参数决定。
// 当前 V4 抓放主链路请从 confirmed_grasp_core 阅读。

// 模块：整体连接（fpga_grasp_top）
// 作用：把相机接收、识别接口、抓取流程和舵机串口连接起来。
// 输入：时钟、复位、相机RX、启动开关、急停、舵机RX。
// 输出：舵机TX、工作状态、故障与观察信号；机械臂RX由servo_rx输入。
// 注意：默认锁住物理舵机输出；5V电平转换在FPGA外部。CNN、LK和回包协议仍是演示接口。
// 阅读：每行右侧说明这行代码在做什么。

`timescale 1ns / 1ps // 仿真时间单位1ns，最小精度1ps；不产生硬件时钟。
module fpga_grasp_top #(  // 定义这个模块；括号中写参数或输入输出接口。
    parameter integer CLK_HZ = 50000000,  // 设置时钟频率；这些常量在综合前确定。
    parameter integer CAMERA_BAUD = 115200,  // 设置相机串口波特率；这些常量在综合前确定。
    parameter integer SERVO_ID_BASE = 0,  // 机械臂关节ID按文档使用0..5。
    parameter integer EXTERNAL_BUS_DRIVER = 0,  // 默认连接四针中转板，无须外部OE。
    parameter integer SERVO_BAUD = 115200,  // 设置舵机串口波特率；这些常量在综合前确定。
    parameter integer IMAGE_W = 80,  // 设置图像宽度；这些常量在综合前确定。
    parameter integer IMAGE_H = 60,  // 设置图像高度；这些常量在综合前确定。
    parameter integer DEMO_MODE = 1,  // 设置演示模式开关；这些常量在综合前确定。
    parameter integer MODEL_BACKEND = 0,  // 0为原算法回退，1为FINN适配接口。
    parameter integer MODEL_CONFIDENCE_THRESHOLD = 128,  // 模型结果最低置信度。
    parameter integer ACTUATOR_OUTPUT_ENABLE = 0,  // 设置物理舵机输出开关；这些常量在综合前确定。
    parameter integer MOVE_MS = 800,  // 设置动作等待毫秒数；这些常量在综合前确定。
    parameter integer MONITOR_MS = 2000,  // 设置观察毫秒数；这些常量在综合前确定。
    parameter integer COOLDOWN_MS = 1000,  // 设置冷却等待毫秒数；这些常量在综合前确定。
    parameter integer FRAME_TIMEOUT_MS = 5000  // 设置图像超时毫秒数；这些常量在综合前确定。
)(  // 结束参数列表，开始写输入输出接口。
    input wire clk,  // 声明接口：时钟。
    input wire rst_n_in,  // 声明接口：外部低电平复位。
    input wire camera_rx,  // 声明接口：相机串口输入。
    input wire arm_enable_in,  // 声明接口：外部流程开关。
    input wire estop_in,  // 声明接口：外部急停。
    input wire servo_rx,  // 声明接口：舵机串口输入。
    output wire servo_tx,  // 声明接口：半双工总线逻辑侧发送数据。
    output wire servo_oe,  // 声明接口：外部三态电平转换器发送使能。
    output wire busy,  // 声明接口：忙标志。
    output wire fault,  // 声明接口：故障锁。
    output wire [3:0] state_debug,  // 声明接口：当前流程状态。
    output wire [3:0] fault_code,  // 声明接口：故障原因编号。
    output wire [1:0] gesture_debug,  // 声明接口：手势观察值。
    output wire [1:0] region_debug,  // 声明接口：区域观察值。
    output wire [7:0] object_x_debug,  // 声明接口：物体横坐标观察值。
    output wire [7:0] object_y_debug,  // 声明接口：物体纵坐标观察值。
    output wire object_found_debug,  // 声明接口：物体存在观察值。
    output wire frame_good_debug,  // 声明接口：好帧观察脉冲。
    output wire cycle_done,  // 声明接口：取放流程完成脉冲。
    output wire [7:0] feedback_id_debug,  // 声明接口：回包ID观察值。
    output wire [15:0] feedback_position_debug,  // 声明接口：回包位置观察值。
    output wire feedback_valid_debug,  // 声明接口：回包有效观察脉冲。
    output wire actuator_locked  // 声明接口：物理输出锁标志。
);  // 结束输入输出接口定义。
    wire rst_n;  // 声明连接信号：内部低电平复位。
    wire tick;  // 声明连接信号：毫秒脉冲。
    wire arm_enable;  // 声明连接信号：流程允许开关。
    wire estop;  // 声明连接信号：急停信号。
    reset_sync resetter(  // 创建复位同步模块，并开始连接接口。
    .clk(clk),  // 时钟接时钟。
    .rst_n_in(rst_n_in),  // 外部低电平复位接外部低电平复位。
    .rst_n(rst_n)  // 内部低电平复位接内部低电平复位。
);  // 结束输入输出接口定义。
    input_sync arm_sync(  // 创建外部信号同步模块，并开始连接接口。
    .clk(clk),  // 时钟接时钟。
    .rst_n(rst_n),  // 内部低电平复位接内部低电平复位。
    .signal_in(arm_enable_in),  // 外部开关接外部流程开关。
    .signal_out(arm_enable)  // 同步后的开关接流程允许开关。
);  // 结束输入输出接口定义。
    input_sync stop_sync(  // 创建外部信号同步模块，并开始连接接口。
    .clk(clk),  // 时钟接时钟。
    .rst_n(rst_n),  // 内部低电平复位接内部低电平复位。
    .signal_in(estop_in),  // 外部开关接外部急停。
    .signal_out(estop)  // 同步后的开关接急停信号。
);  // 结束输入输出接口定义。
    tick_ms #(  // 创建毫秒计时模块，下面配置参数。
    .CLK_HZ(CLK_HZ)  // 时钟频率接时钟频率。
) timer(  // 结束参数设置，为这个实例起名并开始连接接口。
    .clk(clk),  // 。
    .rst_n(rst_n),  // 内部低电平复位接内部低电平复位。
    .tick(tick)  // 。
);  // 结束输入输出接口定义。
    wire [7:0] cam_byte;  // 声明8位连接信号：相机接收字节。
    wire [7:0] pixel;  // 声明8位连接信号：灰度像素。
    wire [7:0] x;  // 声明8位连接信号：当前像素横坐标。
    wire [7:0] y;  // 声明8位连接信号：当前像素纵坐标。
    wire [7:0] frame_id;  // 声明8位连接信号：帧号。
    wire cam_valid;  // 声明连接信号：相机字节有效标志。
    wire cam_error;  // 声明连接信号：相机串口错误。
    wire frame_start;  // 声明连接信号：开始一帧的脉冲。
    wire pixel_valid;  // 声明连接信号：像素有效标志。
    wire frame_good;  // 声明连接信号：完整好帧脉冲。
    wire frame_bad;  // 声明连接信号：坏帧脉冲。
    uart_rx #(  // 创建串口接收模块，下面配置参数。
    .CLK_HZ(CLK_HZ),  // 时钟频率接时钟频率。
    .BAUD(CAMERA_BAUD)  // 。
) camera_uart(  // 结束参数设置，为这个实例起名并开始连接接口。
    .clk(clk),  // 时钟接时钟。
    .rst_n(rst_n),  // 内部低电平复位接内部低电平复位。
    .rx(camera_rx),  // 串口输入接相机串口输入。
    .data(cam_byte),  // 字节数据接相机接收字节。
    .valid(cam_valid),  // 数据有效标志接相机字节有效标志。
    .frame_error(cam_error)  // 停止位错误脉冲接相机串口错误。
);  // 结束输入输出接口定义。
    camera_frame_rx #(  // 创建相机数据包解析模块，下面配置参数。
    .IMAGE_W(IMAGE_W),  // 图像宽度接图像宽度。
    .IMAGE_H(IMAGE_H),  // 图像高度接图像高度。
    .CLK_HZ(CLK_HZ)  // 。
) camera_packet(  // 结束参数设置，为这个实例起名并开始连接接口。
    .clk(clk),  // 时钟接时钟。
    .rst_n(rst_n),  // 内部低电平复位接内部低电平复位。
    .byte_data(cam_byte),  // 字节内容接相机接收字节。
    .byte_valid(cam_valid),  // 字节有效标志接相机字节有效标志。
    .byte_error(cam_error),  // 字节错误接相机串口错误。
    .frame_start(frame_start),  // 开始一帧的脉冲接开始一帧的脉冲。
    .pixel_valid(pixel_valid),  // 像素有效标志接像素有效标志。
    .pixel(pixel),  // 灰度像素接灰度像素。
    .x(x),  // 当前像素横坐标接当前像素横坐标。
    .y(y),  // 当前像素纵坐标接当前像素纵坐标。
    .frame_good(frame_good),  // 完整好帧脉冲接完整好帧脉冲。
    .frame_bad(frame_bad),  // 坏帧脉冲接坏帧脉冲。
    .frame_id(frame_id)  // 帧号接帧号。
);  // 结束输入输出接口定义。
    assign frame_good_debug = frame_good;  // 好帧观察脉冲设为完整好帧脉冲。
    wire cnn_enable;  // 声明连接信号：手势接口使能。
    wire lk_enable;  // 声明连接信号：光流接口使能。
    wire monitor_enable;  // 声明连接信号：观察模块使能。
    wire gesture_sample_valid;  // 声明连接信号：本帧手势有效脉冲。
    wire [7:0] model_confidence;  // 模型输出置信度。
    // 尚未接入真实FINN模型，先用无效常量禁止误动作。
    wire fin_result_valid = 1'b0;
    wire [1:0] fin_class = 2'd3;
    wire [7:0] fin_confidence = 8'd0;
    wire stable_valid;  // 声明连接信号：连续帧确认标志。
    wire frame_allowed;  // 声明连接信号：当前有效新帧允许标志。
    wire [1:0] gesture_sample;  // 声明2位连接信号：本帧手势类别。
    wire [1:0] stable_gesture;  // 声明2位连接信号：确认后的手势。
    wire [1:0] target_region;  // 声明2位连接信号：物体所在区域。
    wire object_result_valid;  // 声明连接信号：物体结果有效脉冲。
    wire object_found;  // 声明连接信号：找到物体标志。
    wire request_valid;  // 声明连接信号：抓取请求。
    wire monitor_checked;  // 声明连接信号：已观察两帧的标志。
    wire slip_alarm;  // 声明连接信号：疑似滑移报警。
    wire [7:0] center_x;  // 声明8位连接信号：物体中心横坐标。
    wire [7:0] center_y;  // 声明8位连接信号：物体中心纵坐标。
    wire [23:0] object_area;  // 声明24位连接信号：物体亮像素数。
    resource_scheduler scheduler(  // 创建阶段切换模块，并开始连接接口。
    .robot_busy(busy),  // 机械臂流程忙标志接忙标志。
    .monitor_phase(monitor_enable),  // 观察阶段标志接观察模块使能。
    .cnn_enable(cnn_enable),  // 手势接口使能接手势接口使能。
    .lk_enable(lk_enable)  // 光流接口使能接光流接口使能。
);  // 结束输入输出接口定义。
    vision_model_adapter #(  // 创建手势接口演示模块，下面配置参数。
    .IMAGE_W(IMAGE_W),  // 图像宽度。
    .IMAGE_H(IMAGE_H),  // 图像高度。
    .MODEL_BACKEND(MODEL_BACKEND),  // 选择后备处理通路或 FINN 通路。
    .CONFIDENCE_THRESHOLD(MODEL_CONFIDENCE_THRESHOLD)  // 演示模式开关接演示模式开关。
) gesture_core(  // 结束参数设置，为这个实例起名并开始连接接口。
    .clk(clk),  // 时钟接时钟。
    .rst_n(rst_n),  // 内部低电平复位接内部低电平复位。
    .enable(cnn_enable),  // 模块使能接手势接口使能。
    .frame_start(frame_start),  // 开始一帧的脉冲接开始一帧的脉冲。
    .pixel_valid(pixel_valid),  // 像素有效标志接像素有效标志。
    .pixel(pixel),  // 灰度像素接灰度像素。
    .x(x),  // 当前像素横坐标接当前像素横坐标。
    .y(y),  // 当前像素纵坐标接当前像素纵坐标。
    .frame_good(frame_good),
    .fin_result_valid(fin_result_valid),  // FINN结果有效标志。
    .fin_class(fin_class),  // FINN输出类别。
    .fin_confidence(fin_confidence),  // FINN输出置信度。 // 完整好帧脉冲接完整好帧脉冲。
    .result_valid(gesture_sample_valid),  // 结果有效脉冲接本帧手势有效脉冲。
    .gesture(gesture_sample),
    .confidence(model_confidence)  // 模型输出置信度。 // 手势类别接本帧手势类别。
);  // 结束输入输出接口定义。
    gesture_vote voter(  // 创建连续帧确认模块，并开始连接接口。
    .clk(clk),  // 时钟接时钟。
    .rst_n(rst_n),  // 内部低电平复位接内部低电平复位。
    .sample_valid(gesture_sample_valid),  // 新样本有效标志接本帧手势有效脉冲。
    .sample_gesture(gesture_sample),  // 这帧手势接本帧手势类别。
    .stable_valid(stable_valid),  // 连续帧确认标志接连续帧确认标志。
    .stable_gesture(stable_gesture)  // 确认后的手势接确认后的手势。
);  // 结束输入输出接口定义。
    object_locator #(  // 创建物体中心定位模块，下面配置参数。
    .IMAGE_H(IMAGE_H)  // 图像高度接图像高度。
) object_core(  // 结束参数设置，为这个实例起名并开始连接接口。
    .clk(clk),  // 时钟接时钟。
    .rst_n(rst_n),  // 内部低电平复位接内部低电平复位。
    .frame_start(frame_start),  // 开始一帧的脉冲接开始一帧的脉冲。
    .pixel_valid(pixel_valid),  // 像素有效标志接像素有效标志。
    .pixel(pixel),  // 灰度像素接灰度像素。
    .x(x),  // 当前像素横坐标接当前像素横坐标。
    .y(y),  // 当前像素纵坐标接当前像素纵坐标。
    .frame_good(frame_good),  // 完整好帧脉冲接完整好帧脉冲。
    .result_valid(object_result_valid),  // 结果有效脉冲接物体结果有效脉冲。
    .found(object_found),  // 找到物体标志接找到物体标志。
    .center_x(center_x),  // 物体中心横坐标接物体中心横坐标。
    .center_y(center_y),  // 物体中心纵坐标接物体中心纵坐标。
    .area(object_area)  // 亮像素数量接物体亮像素数。
);  // 结束输入输出接口定义。
    frame_fresh_gate freshness(  // 创建新帧有效门模块，并开始连接接口。
    .clk(clk),  // 时钟接时钟。
    .rst_n(rst_n),  // 内部低电平复位接内部低电平复位。
    .frame_start(frame_start),  // 开始一帧的脉冲接开始一帧的脉冲。
    .frame_good(frame_good),  // 完整好帧脉冲接完整好帧脉冲。
    .robot_busy(busy),  // 机械臂流程忙标志接忙标志。
    .fault(fault),  // 故障锁接故障锁。
    .frame_allowed(frame_allowed)  // 当前有效新帧允许标志接当前有效新帧允许标志。
);  // 结束输入输出接口定义。
    target_mapper #(  // 创建区域匹配模块，下面配置参数。
    .IMAGE_W(IMAGE_W)  // 图像宽度接图像宽度。
) mapper(  // 结束参数设置，为这个实例起名并开始连接接口。
    .object_valid(object_result_valid),  // 物体结果有效标志接物体结果有效脉冲。
    .object_found(object_found),  // 找到物体标志接找到物体标志。
    .center_x(center_x),  // 物体中心横坐标接物体中心横坐标。
    .gesture_valid(stable_valid && frame_allowed),  // 手势确认标志接连续帧确认标志 && 当前有效新帧允许标志。
    .gesture(stable_gesture),  // 手势类别接确认后的手势。
    .request_valid(request_valid),  // 抓取请求接抓取请求。
    .target_region(target_region)  // 物体所在区域接物体所在区域。
);  // 结束输入输出接口定义。
    lk_monitor_demo monitor(  // 创建滑移接口演示模块，并开始连接接口。
    .clk(clk),  // 时钟接时钟。
    .rst_n(rst_n),  // 内部低电平复位接内部低电平复位。
    .enable(lk_enable),  // 模块使能接光流接口使能。
    .sample_valid(object_result_valid),  // 新样本有效标志接物体结果有效脉冲。
    .object_found(object_found),  // 找到物体标志接找到物体标志。
    .center_x(center_x),  // 物体中心横坐标接物体中心横坐标。
    .center_y(center_y),  // 物体中心纵坐标接物体中心纵坐标。
    .checked(monitor_checked),  // 已经比较过两帧的标志接已观察两帧的标志。
    .slip_alarm(slip_alarm)  // 疑似滑移报警接疑似滑移报警。
);  // 结束输入输出接口定义。
    assign gesture_debug = stable_gesture;  // 手势观察值设为确认后的手势。
    assign object_x_debug = center_x;  // 物体横坐标观察值设为物体中心横坐标。
    assign object_y_debug = center_y;  // 物体纵坐标观察值设为物体中心纵坐标。
    assign object_found_debug = object_found;  // 物体存在观察值设为找到物体标志。
    wire pose_start;  // 声明连接信号：动作启动脉冲。
    wire pose_busy;  // 声明连接信号：六关节发送忙标志。
    wire pose_done;  // 声明连接信号：六关节发送完成脉冲。
    wire monitor_timeout;  // 声明连接信号：观察不足超时脉冲。
    wire cmd_valid;  // 声明连接信号：指令有效标志。
    wire cmd_ready;  // 声明连接信号：总线可接收标志。
    wire cmd_done;  // 声明连接信号：报文发完脉冲。
    wire cmd_error;  // 声明连接信号：指令范围错误脉冲。
    wire [2:0] pose_id;  // 声明3位连接信号：动作编号。
    wire [1:0] selected_region;  // 声明2位连接信号：锁定的抓取区域。
    wire [95:0] positions;  // 声明96位连接信号：6个关节打包位置。
    wire [15:0] pose_time;  // 声明16位连接信号：动作等待时间。
    wire [95:0] cmd_positions;  // 声明16位连接信号：目标舵机位置。
    wire [95:0] cmd_times;  // 声明16位连接信号：目标运动时间。
    wire [47:0] cmd_ids;  // 声明8位连接信号：目标舵机ID。
    wire logic_enable = arm_enable && !fault && !estop && !estop_in;  // 启动开关打开，并且没有故障或急停，才允许发内部指令。
    grab_fsm #(  // 创建抓取主流程模块，下面配置参数。
    .MONITOR_MS(MONITOR_MS),  // 。
    .COOLDOWN_MS(COOLDOWN_MS)  // 冷却等待毫秒数接冷却等待毫秒数。
) controller(  // 结束参数设置，为这个实例起名并开始连接接口。
    .clk(clk),  // 时钟接时钟。
    .rst_n(rst_n),  // 内部低电平复位接内部低电平复位。
    .tick(tick),  // 毫秒脉冲接毫秒脉冲。
    .arm_enable(arm_enable),  // 流程允许开关接流程允许开关。
    .fault(fault),  // 故障锁接故障锁。
    .request_valid(request_valid),  // 抓取请求接抓取请求。
    .target_region(target_region),  // 物体所在区域接物体所在区域。
    .pose_start(pose_start),  // 动作启动脉冲接动作启动脉冲。
    .pose_id(pose_id),  // 动作编号接动作编号。
    .region(selected_region),  // 目标区域接锁定的抓取区域。
    .pose_done(pose_done),  // 六关节发送完成脉冲接六关节发送完成脉冲。
    .pose_time(pose_time),  // 动作等待时间接动作等待时间。
    .monitor_checked(monitor_checked),  // 已观察两帧的标志接已观察两帧的标志。
    .monitor_enable(monitor_enable),  // 观察模块使能接观察模块使能。
    .busy(busy),  // 忙标志接忙标志。
    .cycle_done(cycle_done),  // 取放流程完成脉冲接取放流程完成脉冲。
    .monitor_timeout(monitor_timeout),  // 观察不足超时脉冲接观察不足超时脉冲。
    .state_debug(state_debug)  // 当前流程状态接当前流程状态。
);  // 结束输入输出接口定义。
    pose_table #(  // 创建动作数据表模块，下面配置参数。
    .MOVE_MS(MOVE_MS)  // 动作等待毫秒数接动作等待毫秒数。
) poses(  // 结束参数设置，为这个实例起名并开始连接接口。
    .pose_id(pose_id),  // 动作编号接动作编号。
    .region(selected_region),  // 目标区域接锁定的抓取区域。
    .positions(positions),  // 6个关节打包位置接6个关节打包位置。
    .duration_ms(pose_time)  // 运动时间接动作等待时间。
);  // 结束输入输出接口定义。
    pose_group_sender #(
    .ID_BASE(SERVO_ID_BASE)
) sender (
    .clk(clk),
    .rst_n(rst_n && logic_enable),
    .start(pose_start),
    .positions(positions),
    .duration_ms(pose_time),
    .cmd_valid(cmd_valid),
    .cmd_ready(cmd_ready),
    .cmd_ids(cmd_ids),
    .cmd_positions(cmd_positions),
    .cmd_times(cmd_times),
    .cmd_done(cmd_done),
    .busy(pose_busy),
    .done(pose_done)
);
    assign region_debug = selected_region;  // 区域观察值设为锁定的抓取区域。
    wire servo_tx_internal;  // 声明连接信号：内部串口发送信号。
    wire servo_oe_internal;  // 声明连接信号：内部半双工发送使能。
    servo_command_master #(  // 创建舵机总线发送模块，下面配置参数。
        .CLK_HZ(CLK_HZ),  // 时钟频率接时钟频率。
        .BAUD(SERVO_BAUD)  // 。
    ) servo_master(  // 结束参数设置，为这个实例起名并开始连接接口。
        .clk(clk),  // 时钟接时钟。
        .rst_n(rst_n),  // 内部低电平复位接内部低电平复位。
        .enable(logic_enable),  // 模块使能接内部指令发送允许条件。
        .cmd_valid(cmd_valid),  // 指令有效标志接指令有效标志。
        .cmd_ready(cmd_ready),  // 总线可接收标志接总线可接收标志。
        .servo_count(4'd6),  // 一包发送6个关节。
        .ids(cmd_ids),  // 6个关节ID。
        .positions(cmd_positions),  // 6个位置值。
        .times_ms(cmd_times),  // 6个运动时间值。
        .group_id(16'd0),  // 按商家示例发送G0000。
        .cmd_done(cmd_done),  // 完整组包发完。
        .cmd_error(cmd_error),  // 指令范围错误脉冲接指令范围错误脉冲。
        .tx(servo_tx_internal),  // 串口输出接内部串口发送信号。
        .oe(servo_oe_internal)  // 半双工总线发送使能接内部半双工发送使能。
    );  // 结束输入输出接口定义。
    assign actuator_locked = (ACTUATOR_OUTPUT_ENABLE == 0);  // 输出参数为0时，告知物理舵机输出被锁住。
    assign servo_oe = (EXTERNAL_BUS_DRIVER != 0 && ACTUATOR_OUTPUT_ENABLE != 0 && logic_enable) ? servo_oe_internal : 1'b0;  // 物理输出锁定或故障时释放半双工总线。
    assign servo_tx = (ACTUATOR_OUTPUT_ENABLE != 0 && logic_enable) ? servo_tx_internal : 1'b1;  // 发送期间输出数据，释放期间保持空闲高电平。
    wire [7:0] feedback_byte;  // 声明8位连接信号：回包字节。
    wire feedback_byte_valid;  // 声明连接信号：回包字节有效标志。
    wire feedback_byte_error;  // 声明连接信号：回包串口错误。
    wire feedback_parse_error;  // 声明连接信号：回包格式错误。
    wire feedback_rx_rst = rst_n && !servo_oe_internal;  // 半双工发送时暂停接收，释放总线后再接收回包。
    uart_rx #(  // 创建串口接收模块，下面配置参数。
        .CLK_HZ(CLK_HZ),  // 时钟频率接时钟频率。
        .BAUD(SERVO_BAUD)  // 。
    ) feedback_uart(  // 结束参数设置，为这个实例起名并开始连接接口。
        .clk(clk),  // 时钟接时钟。
        .rst_n(feedback_rx_rst),  // 内部低电平复位接回包接收器复位条件。
        .rx(servo_rx),  // 串口输入接舵机串口输入。
        .data(feedback_byte),  // 字节数据接回包字节。
        .valid(feedback_byte_valid),  // 数据有效标志接回包字节有效标志。
        .frame_error(feedback_byte_error)  // 停止位错误脉冲接回包串口错误。
    );  // 结束输入输出接口定义。
    assign feedback_id_debug = 8'd0;  // 未提供回复协议，不伪造反馈ID。
    assign feedback_position_debug = 16'd0;  // 未实现真实到位反馈。
    assign feedback_valid_debug = 1'b0;  // 旧演示回包通路停用。
    safety_guard #(  // 创建故障保护模块，下面配置参数。
        .FRAME_TIMEOUT_MS(FRAME_TIMEOUT_MS)  // 图像超时毫秒数接图像超时毫秒数。
    ) guard(  // 结束参数设置，为这个实例起名并开始连接接口。
        .clk(clk),  // 时钟接时钟。
        .rst_n(rst_n),  // 内部低电平复位接内部低电平复位。
        .tick(tick),  // 毫秒脉冲接毫秒脉冲。
        .armed(arm_enable),  // 内部流程允许标志接流程允许开关。
        .estop(estop || estop_in),  // 急停信号接急停信号 || 外部急停。
        .frame_good(frame_good),  // 完整好帧脉冲接完整好帧脉冲。
        .frame_bad(frame_bad),  // 坏帧脉冲接坏帧脉冲。
        .command_error(cmd_error),  // 指令错误接指令范围错误脉冲。
        .slip_alarm(slip_alarm),  // 疑似滑移报警接疑似滑移报警。
        .monitor_timeout(monitor_timeout),  // 观察不足超时脉冲接观察不足超时脉冲。
        .pose_busy(pose_busy),  // 六关节发送忙标志接六关节发送忙标志。
        .fault(fault),  // 故障锁接故障锁。
        .fault_code(fault_code)  // 故障原因编号接故障原因编号。
    );  // 结束输入输出接口定义。
endmodule  // 本模块结束。
