-- Copyright 1986-2018 Xilinx, Inc. All Rights Reserved.
-- --------------------------------------------------------------------------------
-- Tool Version: Vivado v.2018.3 (win64) Build 2405991 Thu Dec  6 23:38:27 MST 2018
-- Date        : Mon Nov  2 21:06:35 2026
-- Host        : FF running 64-bit major release  (build 9200)
-- Command     : write_vhdl -force -mode synth_stub
--               c:/Users/admin/AppData/Local/Temp/v4_submit_78ad61428357457684d45c9bab321028/v4/ip/confirmed_grasp_monitor_1/confirmed_grasp_monitor_stub.vhdl
-- Design      : confirmed_grasp_monitor
-- Purpose     : Stub declaration of top-level module interface
-- Device      : xc7z020clg400-2
-- --------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity confirmed_grasp_monitor is
  Port ( 
    clk : in STD_LOGIC;
    probe_in0 : in STD_LOGIC_VECTOR ( 3 downto 0 );
    probe_in1 : in STD_LOGIC_VECTOR ( 0 to 0 );
    probe_in2 : in STD_LOGIC_VECTOR ( 0 to 0 );
    probe_in3 : in STD_LOGIC_VECTOR ( 7 downto 0 );
    probe_in4 : in STD_LOGIC_VECTOR ( 31 downto 0 );
    probe_in5 : in STD_LOGIC_VECTOR ( 31 downto 0 );
    probe_in6 : in STD_LOGIC_VECTOR ( 49 downto 0 );
    probe_in7 : in STD_LOGIC_VECTOR ( 0 to 0 );
    probe_in8 : in STD_LOGIC_VECTOR ( 7 downto 0 );
    probe_in9 : in STD_LOGIC_VECTOR ( 15 downto 0 );
    probe_in10 : in STD_LOGIC_VECTOR ( 0 to 0 );
    probe_in11 : in STD_LOGIC_VECTOR ( 0 to 0 );
    probe_in12 : in STD_LOGIC_VECTOR ( 1 downto 0 );
    probe_in13 : in STD_LOGIC_VECTOR ( 15 downto 0 );
    probe_in14 : in STD_LOGIC_VECTOR ( 15 downto 0 );
    probe_in15 : in STD_LOGIC_VECTOR ( 31 downto 0 );
    probe_in16 : in STD_LOGIC_VECTOR ( 15 downto 0 );
    probe_in17 : in STD_LOGIC_VECTOR ( 15 downto 0 );
    probe_in18 : in STD_LOGIC_VECTOR ( 2 downto 0 );
    probe_in19 : in STD_LOGIC_VECTOR ( 31 downto 0 );
    probe_out0 : out STD_LOGIC_VECTOR ( 0 to 0 )
  );

end confirmed_grasp_monitor;

architecture stub of confirmed_grasp_monitor is
attribute syn_black_box : boolean;
attribute black_box_pad_pin : string;
attribute syn_black_box of stub : architecture is true;
attribute black_box_pad_pin of stub : architecture is "clk,probe_in0[3:0],probe_in1[0:0],probe_in2[0:0],probe_in3[7:0],probe_in4[31:0],probe_in5[31:0],probe_in6[49:0],probe_in7[0:0],probe_in8[7:0],probe_in9[15:0],probe_in10[0:0],probe_in11[0:0],probe_in12[1:0],probe_in13[15:0],probe_in14[15:0],probe_in15[31:0],probe_in16[15:0],probe_in17[15:0],probe_in18[2:0],probe_in19[31:0],probe_out0[0:0]";
attribute X_CORE_INFO : string;
attribute X_CORE_INFO of stub : architecture is "vio,Vivado 2018.3";
begin
end;
