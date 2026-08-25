@echo off
setlocal
call C:\Xilinx\Vitis\2024.2\settings64.bat >nul 2>&1
cd /d D:\claude_workspace\pov3d\mlkpai_fs03\stream\boot\plbin
bootgen -arch zynq -image pl_lz4x3.bif -process_bitstream bin -w on
bootgen -arch zynq -image pl_prelz4.bif -process_bitstream bin -w on
dir /b *.bin
