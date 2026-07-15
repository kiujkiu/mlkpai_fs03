#!/bin/bash
# 集成 xsim: pov_dual_top + panel_engine_2047 真身 (不含 stub) + tb_pov_dual
cd "$(dirname "$0")"
cmd.exe /c "cd /d D:\claude_workspace\pov3d\mlkpai_fs03\sim && call C:\Xilinx\Vivado\2024.2\settings64.bat && xvlog ..\vivado\hdl\angle_tracker.v ..\vivado\hdl\icnd2047_panel_core.v ..\vivado\hdl\row_drv_icnd1028.v ..\vivado\hdl\panel_engine_2047.v ..\vivado\hdl\pov_dual_top.v tb_pov_dual.v %XILINX_VIVADO%\data\verilog\src\glbl.v && xelab -debug typical -L unisims_ver tb_pov_dual glbl -s tb_dual_int && xsim tb_dual_int -runall" 2>&1 | tee run_tb_dual_int.log
