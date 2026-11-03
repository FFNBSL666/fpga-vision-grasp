// 固定姿态参数：每个姿态打包六个 16 位 P 值，最低 16 位对应 0 号舵机。
// 本文件的旧 XY 说明是历史记录；当前 V4 平面范围为 X ±11 cm、Y 10～25 cm。
// 实际映射使用现有像素姿态 ROM；本次仅增加注释，所有常量保持原值。

// 历史 XY 范围记录：X 为 -120 至120毫米，Y 为110至270毫米；当时未修改 Z 和初始姿态。
localparam [95:0] CONFIRMED_OBSERVE = 96'h051405dc03a008c107ff05dc;
localparam [95:0] CONFIRMED_APPROACH = 96'h051405dc0472085b05dd05dc;
localparam [95:0] CONFIRMED_DESCENT = 96'h051405dc0623098c055d05dc;
localparam [95:0] CONFIRMED_GRIP = 96'h06a405dc0623098c055d05dc;
localparam [95:0] CONFIRMED_LIFT = 96'h06a405dc0472085b05dd05dc;
