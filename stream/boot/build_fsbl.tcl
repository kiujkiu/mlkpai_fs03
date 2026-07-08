# build_fsbl.tcl — XSCT (Vitis 2024.2) : generate + build Zynq FSBL from mlkpai_panel.xsa
# The v5 XSA's ps7_init carries the HP0 32-bit AFI writes
# (mask_write 0xF8008000[0]=1 / 0xF8008014[0]=1) that the FSBL must apply.
# Run:  xsct build_fsbl.tcl
set repo  {D:/claude_workspace/pov3d/mlkpai_fs03}
set boot  $repo/stream/boot
set ws    $boot/fsbl_ws
set xsa   $repo/mlkpai_panel.xsa

if {![file exists $xsa]} { error "XSA not found: $xsa" }
file delete -force $ws
setws $ws

puts ">>> platform create from $xsa"
platform create -name fs03_plat -hw $xsa -os standalone -proc ps7_cortexa9_0
platform generate

# platform generate auto-creates + builds the zynq_fsbl boot domain;
# the resulting fsbl.elf embeds ps7_init from OUR XSA (verified: 3x 0xF8008000
# EMIT_MASKWRITE entries, one per silicon-rev init variant).
set elf $ws/fs03_plat/zynq_fsbl/fsbl.elf
if {![file exists $elf]} { error "FSBL build failed: $elf missing" }
file copy -force $elf $boot/fsbl.elf
puts ">>> OK: copied to $boot/fsbl.elf"
