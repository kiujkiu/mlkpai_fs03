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
# ---- 注入开机画面 (2026-07-28) ----
# platform generate 每次重建工作区, 所以源码存在 stream/boot/fsbl_src/ 并在此覆盖回去,
# 然后用工程自带 Makefile 重编 (不跟 Vitis 构建系统较劲)。
set fsbldir $ws/fs03_plat/zynq_fsbl
set src     $boot/fsbl_src
if {[file exists $src/panel_splash.c]} {
    puts ">>> inject panel splash"
    file copy -force $src/panel_splash.c $fsbldir/panel_splash.c
    file copy -force $src/splash_fb.h    $fsbldir/splash_fb.h
    # 在 FsblHookAfterBitstreamDload 里调用 PanelSplashShow()
    set hk $fsbldir/fsbl_hooks.c
    set fh [open $hk r]; set txt [read $fh]; close $fh
    if {![string match "*PanelSplashShow*" $txt]} {
        # ⚠ 必须挂 BeforeHandoff 而非 AfterBitstreamDload:
        # 后者调用时 PL 的 AXI 复位 (FCLK_RESET0_N) 可能尚未释放, 写进去的配置会被丢弃/清掉
        # (2026-07-28 实测: 挂 AfterBitstreamDload 上电无画面)。
        set anchor "\tfsbl_printf(DEBUG_INFO,\"In FsblHookBeforeHandoff function \\r\\n\");"
        set inject "$anchor\n\n\t/* boot splash: light the panel; engine free-runs until Linux takes over */\n\t(void)PanelSplashShow();"
        if {[string first $anchor $txt] < 0} { error "fsbl_hooks.c anchor not found" }
        set txt [string map [list $anchor $inject] $txt]
        set txt "extern unsigned int PanelSplashShow(void);\n$txt"
        set fh [open $hk w]; puts -nonewline $fh $txt; close $fh
    }
    # 重编 (Makefile 自带 wildcard 收集 .c, 新文件会被带上)
    puts ">>> rebuild fsbl with splash"
    cd $fsbldir
    if {[catch {exec make all 2>@1} out]} { puts $out; error "fsbl rebuild failed" }
    puts $out
}

set elf $ws/fs03_plat/zynq_fsbl/fsbl.elf
if {![file exists $elf]} { error "FSBL build failed: $elf missing" }
file copy -force $elf $boot/fsbl.elf
puts ">>> OK: copied to $boot/fsbl.elf"
