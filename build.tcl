# Run manually with Gowin gw_sh on Windows if preferred over the GUI.
# These two options release the board's multiplexed LCD/camera/LED pins.
set project_dir [file dirname [info script]]
cd $project_dir
open_project vision_robot_ui.gprj
set_option -top_module vision_robot_ui_top
set_option -verilog_std v2001
set_option -output_base_name vision_robot_ui
set_option -timing_driven 1
set_option -use_sspi_as_gpio 1
set_option -use_cpu_as_gpio 1
run all
exit
