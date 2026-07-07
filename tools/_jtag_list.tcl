# 列出所有 JTAG cable 和目标, 只读不动任何板子
if {[catch {
    connect
    puts "==== jtag targets ===="
    puts [jtag targets]
    puts "==== debug targets ===="
    puts [targets]
} err]} {
    puts "ERR: $err"
}
exit
