@echo off
REM build_arm.bat - cross-compile pov_rxd (static ARM) from WSL via cmd.exe
REM Toolchain is Vitis-bundled Windows .exe and is NOT on PATH, so the
REM Makefile "arm" target cannot run here.
REM WARNING: this file MUST be CRLF and ASCII-only.
REM   LF endings break cmd set/continuation parsing;
REM   non-ASCII comments get mangled by the GBK console and split REM lines.
setlocal
set "TC=C:\Xilinx\Vitis\2024.2\gnu\aarch32\nt\gcc-arm-linux-gnueabi\bin"
cd /d "%~dp0"
"%TC%\arm-linux-gnueabihf-gcc.exe" -O2 -Wall -Wextra -pthread -march=armv7-a -mfpu=neon -mfloat-abi=hard -static -Ideps\arm -o pov_rxd pov_rxd.c deps\arm\libz.a deps\arm\liblz4.a || exit /b 1
"%TC%\arm-linux-gnueabihf-strip.exe" pov_rxd || exit /b 1
echo BUILD_OK
