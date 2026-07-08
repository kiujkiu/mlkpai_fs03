@echo off
rem MLK-cable-only hw_server on port 3122 (skip wedged Digilent SMT2)
start /b "" "C:\Xilinx\Vitis\2024.2\bin\unwrapped\win64.o\hw_server.exe" -d -s tcp::3122 -e "set jtag-port-filter 2515BCEF4DEA"
