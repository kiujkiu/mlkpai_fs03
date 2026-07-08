# 增量 rebuild: 只改了 RTL/XDC 时用 (BD 没动), ~5 分钟
# cmd.exe /c "cd /d D:\...\mlkpai_fs03 && call C:\Xilinx\Vivado\2024.2\settings64.bat && vivado -mode batch -source vivado\rebuild_panel.tcl"
set DIR [file normalize [file dirname [info script]]/..]
open_project $DIR/build_panel/mlkpai_panel.xpr
set_property STEPS.SYNTH_DESIGN.ARGS.DIRECTIVE RuntimeOptimized [get_runs synth_1]
set_property STEPS.PLACE_DESIGN.ARGS.DIRECTIVE RuntimeOptimized [get_runs impl_1]
set_property STEPS.ROUTE_DESIGN.ARGS.DIRECTIVE RuntimeOptimized [get_runs impl_1]
# ⚠ module_ref IP 有独立 OOC 综合 run, 只 reset synth_1 会链接旧网表 (2026-07-08 实坑)
# run 名 = <BD名>_<cell名>_0_synth_1 (system_panel_0_0_synth_1), 由 BD/cell 名决定,
# 与 Verilog 模块名无关 → v4 icnd2049_panel_fb 和 v5 icnd2049_panel_pov 同名。
# 通配 reset 所有 OOC synth run, 以后加 IP / 改名都不用回来改这行:
foreach r [get_runs *_synth_1] { catch { reset_run $r } }
catch { reset_run system_panel_0_0_synth_1 }  ;# 显式兜底 (通配万一不中)
reset_run synth_1
launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] ne "100%"} { puts "BUILD_FAILED"; exit 1 }
open_run impl_1
puts "TIMING_WNS: [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]"
puts "BUILD_OK"
