# 增量 rebuild: 只改了 RTL/XDC 时用 (BD 没动), ~5 分钟
# cmd.exe /c "cd /d D:\...\mlkpai_fs03 && call C:\Xilinx\Vivado\2024.2\settings64.bat && vivado -mode batch -source vivado\rebuild_panel.tcl"
set DIR [file normalize [file dirname [info script]]/..]
open_project $DIR/build_panel/mlkpai_panel.xpr
set_property STEPS.SYNTH_DESIGN.ARGS.DIRECTIVE RuntimeOptimized [get_runs synth_1]
set_property STEPS.PLACE_DESIGN.ARGS.DIRECTIVE RuntimeOptimized [get_runs impl_1]
set_property STEPS.ROUTE_DESIGN.ARGS.DIRECTIVE RuntimeOptimized [get_runs impl_1]
reset_run synth_1
launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] ne "100%"} { puts "BUILD_FAILED"; exit 1 }
open_run impl_1
puts "TIMING_WNS: [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]"
puts "BUILD_OK"
