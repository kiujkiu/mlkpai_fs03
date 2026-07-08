@echo off
rem build_boot.bat — reproducible BOOT.BIN build for MLKPAI-FS03 (Zynq-7020)
rem Steps: 1) FSBL from mlkpai_panel.xsa via XSCT  2) bootgen BOOT.BIN
rem Prereqs: u-boot.bin already extracted (python3 extract_uboot.py factory_BOOT.bin u-boot.bin)
rem Run from anywhere:  cmd /c D:\claude_workspace\pov3d\mlkpai_fs03\stream\boot\build_boot.bat
setlocal
set BOOT=D:\claude_workspace\pov3d\mlkpai_fs03\stream\boot

call C:\Xilinx\Vitis\2024.2\settings64.bat

echo === [1/2] FSBL build (xsct) ===
cd /d %BOOT%
call xsct build_fsbl.tcl > %BOOT%\logs\fsbl_build.log 2>&1
if not exist %BOOT%\fsbl.elf (
  echo FSBL BUILD FAILED - see logs\fsbl_build.log
  exit /b 1
)

echo === [2/2] bootgen ===
bootgen -arch zynq -image boot.bif -o BOOT.BIN -w on > %BOOT%\logs\bootgen.log 2>&1
if not exist %BOOT%\BOOT.BIN (
  echo BOOTGEN FAILED - see logs\bootgen.log
  exit /b 1
)
echo === DONE: %BOOT%\BOOT.BIN ===
dir %BOOT%\BOOT.BIN
