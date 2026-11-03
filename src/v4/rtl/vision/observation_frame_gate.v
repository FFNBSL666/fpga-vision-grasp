// 新帧接纳门：只在观察条件稳定后，提交从头到尾完整接收的图像。
// 使用任务代次 epoch 和帧号匹配结果，避免旧图像结果触发下一轮。

`timescale 1ns / 1ps
// 原腕部相机方案要求在标定姿态下定位桌面；此处仍沿用对应的观察门控。
// 完整图像帧的开始和结束都必须发生在观察姿态稳定之后。
// 模型封装必须随任务保存 submit_epoch 和 submit_frame_id。
// 之前姿态或图像帧的延迟结果不能触发新的坐标捕获。
module observation_frame_gate #(
    parameter integer SETTLE_MS = 500,
    parameter integer RESULT_TIMEOUT_MS = 5000
)(
    input wire clk, rst_n, tick_ms,
    input wire enable, fault, estop, robot_busy, at_observation_pose, cancel_capture,
    input wire frame_start, frame_good, frame_bad,
    input wire [7:0] frame_id,
    input wire model_ready, model_result_valid,
    input wire [31:0] result_epoch,
    input wire [7:0] result_frame_id,
    output wire observe_ready,
    output reg frame_submit,
    output reg [31:0] submit_epoch,
    output reg [7:0] submit_frame_id,
    output wire result_accept,
    output wire clear_targets,
    output reg model_timeout,
    output reg in_flight
);
    wire pose_ok = rst_n && enable && !fault && !estop && !robot_busy && at_observation_pose;
    reg pose_was_ok, capturing;
    reg [31:0] settled_ms, age_ms, epoch;
    reg [7:0] capture_frame_id;
    assign observe_ready = pose_ok && pose_was_ok && (SETTLE_MS == 0 || settled_ms >= SETTLE_MS);

    // 结果必须同时匹配当前任务代次和提交帧号，才向选点模块放行。
    assign result_accept = model_result_valid && observe_ready && !cancel_capture && in_flight &&
    result_epoch == submit_epoch && result_epoch == epoch &&
    result_frame_id == submit_frame_id;
    assign clear_targets = !observe_ready || model_timeout || cancel_capture;
    always @(posedge clk) begin
        if (!rst_n) begin
            pose_was_ok <= 0;
            capturing <= 0;
            settled_ms <= 0;
            age_ms <= 0;
            epoch <= 0;
            capture_frame_id <= 0;
            frame_submit <= 0;
            submit_epoch <= 0;
            submit_frame_id <= 0;
            model_timeout <= 0;
            in_flight <= 0;
        end else begin
            pose_was_ok <= pose_ok;
            frame_submit <= 0;
            model_timeout <= 0;
            if (!pose_ok) begin
                settled_ms <= 0;
                capturing <= 0;
                in_flight <= 0;
                age_ms <= 0;
                if (pose_was_ok) epoch <= epoch + 1'b1;
            end else if (cancel_capture) begin
                capturing <= 0;
                in_flight <= 0;
                age_ms <= 0;
                epoch <= epoch + 1'b1;
            end else begin
                if (tick_ms && settled_ms < SETTLE_MS) settled_ms <= settled_ms + 1'b1;

                // 必须从当前稳定观察阶段的帧头开始接收，不能接纳半帧。
                if (frame_start) begin
                    capturing <= observe_ready && model_ready && !in_flight;
                    capture_frame_id <= frame_id;
                end
                if (frame_bad) capturing <= 0;
                else if (frame_good) begin
                    capturing <= 0;
                    if (capturing && observe_ready && model_ready && !in_flight) begin
                        // 独立的任务代次同时避免8位相机帧号回绕后，
                        // 错误接纳旧的重复输出。
                        epoch <= epoch + 1'b1;
                        frame_submit <= 1;
                        submit_epoch <= epoch + 1'b1;
                        submit_frame_id <= capture_frame_id;
                        in_flight <= 1;
                        age_ms <= 0;
                    end
                end

                // 接收结果后释放任务；等待超时则清除任务并产生原有超时脉冲。
                if (result_accept) begin
                    in_flight <= 0;
                    age_ms <= 0;
                end
                else if (in_flight && tick_ms) begin
                    if (RESULT_TIMEOUT_MS == 0 || age_ms >= RESULT_TIMEOUT_MS - 1) begin
                        in_flight <= 0;
                        capturing <= 0;
                        age_ms <= 0;
                        epoch <= epoch + 1'b1;
                        model_timeout <= 1;
                    end else age_ms <= age_ms + 1'b1;
                end
            end
        end
    end
endmodule
