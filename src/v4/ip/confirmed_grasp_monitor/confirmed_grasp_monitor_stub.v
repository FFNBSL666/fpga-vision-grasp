// Copyright 1986-2018 Xilinx, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2018.3 (win64) Build 2405991 Thu Dec  6 23:38:27 MST 2018
// Date        : Sat Oct 31 08:45:37 2026
// Host        : FF running 64-bit major release  (build 9200)
// Command     : write_verilog -force -mode synth_stub
//               f:/jinsai/vision_versions_20261029/v4/ip/confirmed_grasp_monitor/confirmed_grasp_monitor_stub.v
// Design      : confirmed_grasp_monitor
// Purpose     : Stub declaration of top-level module interface
// Device      : xc7z020clg400-2
// --------------------------------------------------------------------------------

// This empty module with port declaration file causes synthesis tools to infer a black box for IP.
// The synthesis directives are for Synopsys Synplify support to prevent IO buffer insertion.
// Please paste the declaration into a Verilog source file or add the file as an additional source.
(* X_CORE_INFO = "vio,Vivado 2018.3" *)
module confirmed_grasp_monitor(clk, probe_in0, probe_in1, probe_in2, probe_in3, 
  probe_in4, probe_in5, probe_in6, probe_in7, probe_in8, probe_in9, probe_in10, probe_in11, 
  probe_in12, probe_in13, probe_in14, probe_in15, probe_in16, probe_in17, probe_in18, probe_in19, 
  probe_out0)
/* synthesis syn_black_box black_box_pad_pin="clk,probe_in0[3:0],probe_in1[0:0],probe_in2[0:0],probe_in3[7:0],probe_in4[31:0],probe_in5[31:0],probe_in6[49:0],probe_in7[0:0],probe_in8[7:0],probe_in9[15:0],probe_in10[0:0],probe_in11[0:0],probe_in12[1:0],probe_in13[15:0],probe_in14[15:0],probe_in15[31:0],probe_in16[15:0],probe_in17[15:0],probe_in18[2:0],probe_in19[31:0],probe_out0[0:0]" */;
  input clk;
  input [3:0]probe_in0;
  input [0:0]probe_in1;
  input [0:0]probe_in2;
  input [7:0]probe_in3;
  input [31:0]probe_in4;
  input [31:0]probe_in5;
  input [49:0]probe_in6;
  input [0:0]probe_in7;
  input [7:0]probe_in8;
  input [15:0]probe_in9;
  input [0:0]probe_in10;
  input [0:0]probe_in11;
  input [1:0]probe_in12;
  input [15:0]probe_in13;
  input [15:0]probe_in14;
  input [31:0]probe_in15;
  input [15:0]probe_in16;
  input [15:0]probe_in17;
  input [2:0]probe_in18;
  input [31:0]probe_in19;
  output [0:0]probe_out0;
endmodule
