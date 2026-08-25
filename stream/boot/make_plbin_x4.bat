@echo off
setlocal
cd /d D:\claude_workspace\pov3d\mlkpai_fs03\stream\boot\plbin
REM 4-engine PL bitstream only. pl_lz4x3.* is the live 3-engine fallback -- do not touch.
REM Do NOT replace bootgen with bit_to_bin: the PCAP/fpga-manager path rejects that
REM format (memory note: feedback_dr1_load_bit_without_jtag).
REM Full path to bootgen.bat on purpose: calling Vitis settings64.bat did not put
REM bootgen on PATH in a non-interactive cmd /c here (2026-08-25).
C:\Xilinx\Vitis\2024.2\bin\bootgen.bat -arch zynq -image pl_lz4x4.bif -process_bitstream bin -w on
dir /b pl_lz4x4.*
