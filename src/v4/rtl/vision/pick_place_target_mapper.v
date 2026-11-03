// V4 选点核心：第一次同时检出手指和物体，且两者的桌面距离不超过 30 mm。
// 连续三个稳定结果后锁定物体中心；移开手后才能进行第二次选点。
// 第二次锁定手指坐标，阶段标志变为 2；再次移开手后才输出有效抓放请求。
// 使用现有模型给出的手指坐标；这里没有增加指向射线或新的识别模型。

`timescale 1ns/1ps
// V4 邻近选物使用原模型输出的手指坐标，
// 不推断指向射线；只在第一次锁定物体时增加30毫米圆形范围。
// 邻近条件稳定后锁定物体中心，再等待手移开。
// 第二次同时检出手指和物体时锁定手指坐标，第二次移手后才执行动作。
module pick_place_target_mapper(
    input wire clk, rst_n, enable, result_valid,
    input wire [49:0] detections,
    output reg response, valid,
    output reg [1:0] phase,
    output reg [15:0] source_pixel, destination_pixel,
    output reg [95:0] source_approach, source_descent, source_grip, source_lift,
    output reg [95:0] destination_approach, destination_descent, destination_release, destination_retreat
);

    // 阶段顺序：选物体 → 物体查表 → 第一次移手 → 选放置点 → 查表 → 第二次移手。
    localparam [2:0] SOURCE = 0, SOURCE_MAP = 1, FIRST_CLEAR = 2,
    DESTINATION = 3, DESTINATION_MAP = 4, SECOND_CLEAR = 5, DONE = 6;
    reg [2:0] state;
    reg [1:0] point_count, clear_count;
    reg [7:0] candidate_x, candidate_y;
    reg sr, dr;
    wire sresponse, svalid, dresponse, dvalid;

    // 检测字段拆包：fx/fy 为手指坐标，ox/oy 为物体中心，坐标范围 0～95。
    wire [7:0] fx = detections[16:9], fy = detections[24:17];
    wire [7:0] ox = detections[41:34], oy = detections[49:42];

    // 有效条件沿用原置信度阈值 128；手指和物体都有效才参与选点。
    wire finger = detections[0] && detections[8:1] >= 128 && fx < 96 && fy < 96;
    wire object_ok = detections[25] && detections[33:26] >= 128 && ox < 96 && oy < 96;
    wire both_present = finger && object_ok;
    // 沿用 V1 工作平面：96乘96图像的端点横向跨度为220毫米，
    // 纵向跨度为150毫米，X 为 -11 至11厘米，Y 为10至25厘米。
    // 直接比较实际距离平方，避免舍入：(220*dx)^2+(150*dy)^2 <= (30*95)^2。
    // 原有统一 X 镜像处理不改变这段距离。

    // 实际 3 cm 半径：X 跨度 220 mm、Y 跨度 150 mm，各对应 95 个像素间隔。
    // 用 (220×dx)^2+(150×dy)^2 ≤ (30×95)^2 判断，包含圆周，避免整数除法损失。
    localparam [31:0] SOURCE_RADIUS_SCALED_SQUARED = 32'd8122500;
    wire [7:0] finger_object_dx = fx > ox ? fx - ox : ox - fx;
    wire [7:0] finger_object_dy = fy > oy ? fy - oy : oy - fy;
    wire [15:0] source_dx_scaled = finger_object_dx * 16'd220;
    wire [15:0] source_dy_scaled = finger_object_dy * 16'd150;
    wire [31:0] source_dx_squared = source_dx_scaled * source_dx_scaled;
    wire [31:0] source_dy_squared = source_dy_scaled * source_dy_scaled;
    wire [31:0] source_distance_squared = source_dx_squared + source_dy_squared;
    wire source_eligible = both_present &&
    source_distance_squared <= SOURCE_RADIUS_SCALED_SQUARED;

    // 第二次要求原物体仍在已锁定中心附近；这两个差值用于检查同一物体。
    wire [7:0] source_dx = ox > source_pixel[7:0] ? ox - source_pixel[7:0] : source_pixel[7:0] - ox;
    wire [7:0] source_dy = oy > source_pixel[15:8] ? oy - source_pixel[15:8] : source_pixel[15:8] - oy;
    wire same_object = object_ok && source_dx <= 6 && source_dy <= 6;

    // 第一次稳定的是物体中心，第二次稳定的是手指位置；允许原有 6 像素抖动。
    wire [7:0] selected_x = state == SOURCE ? ox : fx;
    wire [7:0] selected_y = state == SOURCE ? oy : fy;
    wire [7:0] stable_dx = selected_x > candidate_x ? selected_x - candidate_x : candidate_x - selected_x;
    wire [7:0] stable_dy = selected_y > candidate_y ? selected_y - candidate_y : candidate_y - selected_y;
    wire [95:0] sa, sd, sg, sl, da, dd, dg, dl;

    // 抓取点使用抓取 ROM；放置点使用独立的原有放置 ROM。
    pixel_pose_lookup source_lookup (
        .clk(clk),
        .rst_n(rst_n),
        .request(sr),
        .pixel_x(source_pixel[7:0]),
        .pixel_y(source_pixel[15:8]),
        .response(sresponse),
        .valid(svalid),
        .approach_pose(sa),
        .descent_pose(sd),
        .grip_pose(sg),
        .lift_pose(sl)
    );
    pixel_pose_lookup #(
        .MEM_FILE("placement_pose_lut.mem")
    ) destination_lookup (
        .clk(clk),
        .rst_n(rst_n),
        .request(dr),
        .pixel_x(destination_pixel[7:0]),
        .pixel_y(destination_pixel[15:8]),
        .response(dresponse),
        .valid(dvalid),
        .approach_pose(da),
        .descent_pose(dd),
        .grip_pose(dg),
        .lift_pose(dl)
    );
    always @(posedge clk) begin
        if (!rst_n || !enable) begin
            state <= SOURCE;
            phase <= 0;
            response <= 0;
            valid <= 0;
            sr <= 0;
            dr <= 0;
            point_count <= 0;
            clear_count <= 0;
            candidate_x <= 0;
            candidate_y <= 0;
            source_pixel <= 0;
            destination_pixel <= 0;
            source_approach <= 0;
            source_descent <= 0;
            source_grip <= 0;
            source_lift <= 0;
            destination_approach <= 0;
            destination_descent <= 0;
            destination_release <= 0;
            destination_retreat <= 0;
        end else begin
            response <= 0;
            valid <= 0;
            sr <= 0;
            dr <= 0;

            // 抓取点查表成功后 phase=1，板级 PL_LED0 开始闪烁。
            if (state == SOURCE_MAP && sresponse) begin
                if (svalid) begin
                    source_approach <= sa;
                    source_descent <= sd;
                    source_grip <= sg;
                    source_lift <= sl;
                    phase <= 1;
                    state <= FIRST_CLEAR;
                    clear_count <= 0;
                end else begin
                    state <= SOURCE;
                    point_count <= 0;
                    response <= 1;
                end
            end

            // 放置点查表成功后 phase=2，板级灯先停止闪烁；仍等待第二次移手。
            if (state == DESTINATION_MAP && dresponse) begin
                if (dvalid) begin
                    destination_approach <= dl;
                    destination_descent <= dg;
                    destination_release <= dd;
                    destination_retreat <= da;
                    phase <= 2;
                    state <= SECOND_CLEAR;
                    clear_count <= 0;
                end else begin
                    state <= DESTINATION;
                    point_count <= 0;
                    response <= 1;
                end
            end

            // 只按新识别结果推进；连续三个稳定结果才锁点，缺失或偏离时重新累计。
            if (result_valid) case (state)
                SOURCE, DESTINATION : begin
                    if (!both_present || (state == SOURCE && !source_eligible) ||
                        (state == DESTINATION && !same_object)) point_count <= 0;
                    else if (point_count == 0 || stable_dx > 6 || stable_dy > 6) begin
                        candidate_x <= selected_x;
                        candidate_y <= selected_y;
                        point_count <= 1;
                    end else if (point_count == 2) begin
                        point_count <= 0;
                        if (state == SOURCE) begin
                            source_pixel <= {oy, ox};
                            sr <= 1;
                            state <= SOURCE_MAP;
                        end else begin
                            destination_pixel <= {fy, fx};
                            dr <= 1;
                            state <= DESTINATION_MAP;
                        end
                    end else point_count <= point_count + 1'b1;
                end

                // 第一次移手：连续两个新结果没有手指，才允许进入第二次选点。
                FIRST_CLEAR : begin
                    if (finger) clear_count <= 0;
                    else if (clear_count == 1) begin
                        state <= DESTINATION;
                        clear_count <= 0;
                        point_count <= 0;
                    end else clear_count <= clear_count + 1'b1;
                end

                // 第二次移手：连续两个结果无手且原物体仍在，才提交一次有效抓放请求。
                SECOND_CLEAR : begin
                    if (finger || !same_object) clear_count <= 0;
                    else if (clear_count == 1) begin
                        response <= 1;
                        valid <= 1;
                        state <= DONE;
                        clear_count <= 0;
                    end else clear_count <= clear_count + 1'b1;
                end
                default : begin
                end
            endcase
        end
    end
endmodule
