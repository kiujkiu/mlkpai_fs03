open_project [file normalize [file dirname [info script]]/../build_panel/mlkpai_panel.xpr]
write_hw_platform -fixed -include_bit -force [file normalize [file dirname [info script]]/../mlkpai_panel.xsa]
puts "XSA_OK"
