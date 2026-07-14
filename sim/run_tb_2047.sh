#!/bin/bash
# xsim 批跑: icnd2047_panel_core + row_drv_icnd1028 + tb_2047_core
# 套路: Windows Vivado, cmd.exe + settings64.bat (硬规矩); ODDR 用 unisims_ver + glbl
cd "$(dirname "$0")"
cmd.exe /c "cd /d D:\claude_workspace\pov3d\mlkpai_fs03\sim && call C:\Xilinx\Vivado\2024.2\settings64.bat && xvlog ..\vivado\hdl\icnd2047_panel_core.v ..\vivado\hdl\row_drv_icnd1028.v tb_2047_core.v %XILINX_VIVADO%\data\verilog\src\glbl.v && xelab -debug typical -L unisims_ver tb_2047_core glbl -s tb_2047 && xsim tb_2047 -runall" 2>&1 | tee run_tb_2047.log
