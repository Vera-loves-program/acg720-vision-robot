# Optional manual Windows build. This file is not run by Codex.
set project_dir [file dirname [info script]]
cd $project_dir
open_project vision_robot_netfix.gprj
set_option -top_module vision_robot_top
set_option -verilog_std v2001
set_option -output_base_name vision_robot_netfix
set_option -timing_driven 1
set_option -use_sspi_as_gpio 1
set_option -use_cpu_as_gpio 1
run all
exit
