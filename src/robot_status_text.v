// Compact atomic text: six fields, one20-bit serial decimal datapath.
module robot_status_text #(parameter integer CLK_HZ=50000000,parameter integer PARALLEL_COMPAT=1)(
    input wire clk,reset_n,
    input wire [2:0] page,subpage,
    input wire [15:0] follow_selected,follow_candidate,
    input wire [7:0] follow_kind,follow_confidence,follow_state,
    input wire signed [15:0] follow_speed,
    input wire [7:0] follow_gain,planner_error,follow_diagnostic,follow_dog_score,
    input wire waiting_map,
    input wire waiting_path,
    input wire [15:0] follow_cap,follow_pan_us,follow_tilt_us,
    input wire signed [15:0] visual_x,visual_y,
    input wire armed,geometry_valid,gyro_calibrated,calibrating,lidar_ready,scan_enable,safety_stop,map_ready,gains_ready,
    input wire map_local,map_world_ready,
    input wire motor_armed,motor_running,motor_selected_wheel,motor_mode_position,
    input wire encoder_invert_left,encoder_invert_right,drive_invert_left,drive_invert_right,
    input wire calibration_enable,manual_active,manual_turn_active,manual_linked,gyro_invert,
    input wire [3:0] nav_status,ui_notice,
    input wire [4:0] selected_field,
    input wire virtual_contact,
    input wire [2:0] recovery_state,
    input wire [15:0] body_self_points,
    input wire signed [15:0] blocking_x_mm,blocking_y_mm,
    input wire [7:0] nav_fault,lidar_error,motor_fault,
    input wire signed [31:0] selected_value,x_mm,y_mm,heading_mdeg,goal_x_mm,goal_y_mm,
    input wire signed [31:0] speed_left,speed_right,position_left,position_right,
    input wire signed [31:0] set_x_mm,set_y_mm,set_heading_mdeg,
    input wire signed [15:0] duty_left,duty_right,gyro_raw,lidar_offset_x_mm,lidar_offset_y_mm,
    input wire signed [15:0] motor_target_left,motor_target_right,
    input wire signed [31:0] motor_position_target_left,motor_position_target_right,
    input wire [15:0] speed_kp_left,speed_kp_right,speed_ki_left,speed_ki_right,speed_kd_left,speed_kd_right,
    input wire [15:0] position_kp_left,position_kp_right,position_ki_left,position_ki_right,position_kd_left,position_kd_right,
    input wire [31:0] selected_step,lidar_points,
    input wire [31:0] rx_bytes,node_count,uart_errors,packet_errors,restart_count,map_points,map_scans,
    input wire [31:0] map_dropped,
    input wire [31:0] lidar_debug_status,lidar_last_rx_word,
    input wire [127:0] lidar_reply_capture,
    input wire [31:0] lidar_last_tx_word,
    input wire [15:0] front_mm,left_mm,right_mm,rear_mm,
    input wire [15:0] manual_speed_left,manual_speed_right,wheel_mm_per_rev,encoder_counts_per_rev,track_mm,
    input wire [15:0] cruise_counts,turn_counts,
    input wire [8:0] lidar_zero_deg,
    input wire [7:0] match_score,match_baseline,match_margin,
    output wire [2047:0] text_rows,
    output reg [31:0] flags,
    output reg text_valid,
    output wire text_byte_valid,
    output wire [7:0] text_byte_data,
    output wire [7:0] text_byte_index,
    input wire text_byte_ready
);
    (* syn_romstyle="block_rom" *) reg [7:0] labels[0:2047];
    integer init_index;
    initial begin
        for(init_index=0;init_index<1024;init_index=init_index+1)begin labels[init_index]=8'h20;labels[init_index+1024]=8'h20;end
        // 1: WAIT USB CAMERA
        labels[16]=8'h57;
        labels[17]=8'h41;
        labels[18]=8'h49;
        labels[19]=8'h54;
        labels[21]=8'h55;
        labels[22]=8'h53;
        labels[23]=8'h42;
        labels[25]=8'h43;
        labels[26]=8'h41;
        labels[27]=8'h4d;
        labels[28]=8'h45;
        labels[29]=8'h52;
        labels[30]=8'h41;
        // 2: CAMERA ERROR
        labels[32]=8'h43;
        labels[33]=8'h41;
        labels[34]=8'h4d;
        labels[35]=8'h45;
        labels[36]=8'h52;
        labels[37]=8'h41;
        labels[39]=8'h45;
        labels[40]=8'h52;
        labels[41]=8'h52;
        labels[42]=8'h4f;
        labels[43]=8'h52;
        // 3: NO DOG IN VIEW
        labels[48]=8'h4e;
        labels[49]=8'h4f;
        labels[51]=8'h44;
        labels[52]=8'h4f;
        labels[53]=8'h47;
        labels[55]=8'h49;
        labels[56]=8'h4e;
        labels[58]=8'h56;
        labels[59]=8'h49;
        labels[60]=8'h45;
        labels[61]=8'h57;
        // 4: DOG SCORE LOW
        labels[64]=8'h44;
        labels[65]=8'h4f;
        labels[66]=8'h47;
        labels[68]=8'h53;
        labels[69]=8'h43;
        labels[70]=8'h4f;
        labels[71]=8'h52;
        labels[72]=8'h45;
        labels[74]=8'h4c;
        labels[75]=8'h4f;
        labels[76]=8'h57;
        // 5: WAIT DOG STABLE
        labels[80]=8'h57;
        labels[81]=8'h41;
        labels[82]=8'h49;
        labels[83]=8'h54;
        labels[85]=8'h44;
        labels[86]=8'h4f;
        labels[87]=8'h47;
        labels[89]=8'h53;
        labels[90]=8'h54;
        labels[91]=8'h41;
        labels[92]=8'h42;
        labels[93]=8'h4c;
        labels[94]=8'h45;
        // 6: TARGET EXPIRED
        labels[96]=8'h54;
        labels[97]=8'h41;
        labels[98]=8'h52;
        labels[99]=8'h47;
        labels[100]=8'h45;
        labels[101]=8'h54;
        labels[103]=8'h45;
        labels[104]=8'h58;
        labels[105]=8'h50;
        labels[106]=8'h49;
        labels[107]=8'h52;
        labels[108]=8'h45;
        labels[109]=8'h44;
        // 7: CALIBRATE GIMBAL
        labels[112]=8'h43;
        labels[113]=8'h41;
        labels[114]=8'h4c;
        labels[115]=8'h49;
        labels[116]=8'h42;
        labels[117]=8'h52;
        labels[118]=8'h41;
        labels[119]=8'h54;
        labels[120]=8'h45;
        labels[122]=8'h47;
        labels[123]=8'h49;
        labels[124]=8'h4d;
        labels[125]=8'h42;
        labels[126]=8'h41;
        labels[127]=8'h4c;
        // 8: CENTERING DOG
        labels[128]=8'h43;
        labels[129]=8'h45;
        labels[130]=8'h4e;
        labels[131]=8'h54;
        labels[132]=8'h45;
        labels[133]=8'h52;
        labels[134]=8'h49;
        labels[135]=8'h4e;
        labels[136]=8'h47;
        labels[138]=8'h44;
        labels[139]=8'h4f;
        labels[140]=8'h47;
        // 9: CLICK TRACK DOG
        labels[144]=8'h43;
        labels[145]=8'h4c;
        labels[146]=8'h49;
        labels[147]=8'h43;
        labels[148]=8'h4b;
        labels[150]=8'h54;
        labels[151]=8'h52;
        labels[152]=8'h41;
        labels[153]=8'h43;
        labels[154]=8'h4b;
        labels[156]=8'h44;
        labels[157]=8'h4f;
        labels[158]=8'h47;
        // 10: MOVE DOG FORWARD
        labels[160]=8'h4d;
        labels[161]=8'h4f;
        labels[162]=8'h56;
        labels[163]=8'h45;
        labels[165]=8'h44;
        labels[166]=8'h4f;
        labels[167]=8'h47;
        labels[169]=8'h46;
        labels[170]=8'h4f;
        labels[171]=8'h52;
        labels[172]=8'h57;
        labels[173]=8'h41;
        labels[174]=8'h52;
        labels[175]=8'h44;
        // 11: PI COMMAND ERROR
        labels[176]=8'h50;
        labels[177]=8'h49;
        labels[179]=8'h43;
        labels[180]=8'h4f;
        labels[181]=8'h4d;
        labels[182]=8'h4d;
        labels[183]=8'h41;
        labels[184]=8'h4e;
        labels[185]=8'h44;
        labels[187]=8'h45;
        labels[188]=8'h52;
        labels[189]=8'h52;
        labels[190]=8'h4f;
        labels[191]=8'h52;
        // 12: INSTALL POWEROFF
        labels[192]=8'h49;
        labels[193]=8'h4e;
        labels[194]=8'h53;
        labels[195]=8'h54;
        labels[196]=8'h41;
        labels[197]=8'h4c;
        labels[198]=8'h4c;
        labels[200]=8'h50;
        labels[201]=8'h4f;
        labels[202]=8'h57;
        labels[203]=8'h45;
        labels[204]=8'h52;
        labels[205]=8'h4f;
        labels[206]=8'h46;
        labels[207]=8'h46;
        // 13: POWER OFF FAILED
        labels[208]=8'h50;
        labels[209]=8'h4f;
        labels[210]=8'h57;
        labels[211]=8'h45;
        labels[212]=8'h52;
        labels[214]=8'h4f;
        labels[215]=8'h46;
        labels[216]=8'h46;
        labels[218]=8'h46;
        labels[219]=8'h41;
        labels[220]=8'h49;
        labels[221]=8'h4c;
        labels[222]=8'h45;
        labels[223]=8'h44;
        // 14: POWER OFF SENT
        labels[224]=8'h50;
        labels[225]=8'h4f;
        labels[226]=8'h57;
        labels[227]=8'h45;
        labels[228]=8'h52;
        labels[230]=8'h4f;
        labels[231]=8'h46;
        labels[232]=8'h46;
        labels[234]=8'h53;
        labels[235]=8'h45;
        labels[236]=8'h4e;
        labels[237]=8'h54;
        // 15: UPDATE PI V22
        labels[240]=8'h55;
        labels[241]=8'h50;
        labels[242]=8'h44;
        labels[243]=8'h41;
        labels[244]=8'h54;
        labels[245]=8'h45;
        labels[247]=8'h50;
        labels[248]=8'h49;
        labels[250]=8'h56;
        labels[251]=8'h32;
        labels[252]=8'h32;
        // 16: READY TO FOLLOW
        labels[256]=8'h52;
        labels[257]=8'h45;
        labels[258]=8'h41;
        labels[259]=8'h44;
        labels[260]=8'h59;
        labels[262]=8'h54;
        labels[263]=8'h4f;
        labels[265]=8'h46;
        labels[266]=8'h4f;
        labels[267]=8'h4c;
        labels[268]=8'h4c;
        labels[269]=8'h4f;
        labels[270]=8'h57;
        // 17: DOG TRACKING
        labels[272]=8'h44;
        labels[273]=8'h4f;
        labels[274]=8'h47;
        labels[276]=8'h54;
        labels[277]=8'h52;
        labels[278]=8'h41;
        labels[279]=8'h43;
        labels[280]=8'h4b;
        labels[281]=8'h49;
        labels[282]=8'h4e;
        labels[283]=8'h47;
        // 18: CAMERA TOO OLD
        labels[288]=8'h43;
        labels[289]=8'h41;
        labels[290]=8'h4d;
        labels[291]=8'h45;
        labels[292]=8'h52;
        labels[293]=8'h41;
        labels[295]=8'h54;
        labels[296]=8'h4f;
        labels[297]=8'h4f;
        labels[299]=8'h4f;
        labels[300]=8'h4c;
        labels[301]=8'h44;
        // 19: RETRY POWER OFF
        labels[304]=8'h52;
        labels[305]=8'h45;
        labels[306]=8'h54;
        labels[307]=8'h52;
        labels[308]=8'h59;
        labels[310]=8'h50;
        labels[311]=8'h4f;
        labels[312]=8'h57;
        labels[313]=8'h45;
        labels[314]=8'h52;
        labels[316]=8'h4f;
        labels[317]=8'h46;
        labels[318]=8'h46;
        // 20: STOPPED
        labels[320]=8'h53;
        labels[321]=8'h54;
        labels[322]=8'h4f;
        labels[323]=8'h50;
        labels[324]=8'h50;
        labels[325]=8'h45;
        labels[326]=8'h44;
        // 21: TURNING
        labels[336]=8'h54;
        labels[337]=8'h55;
        labels[338]=8'h52;
        labels[339]=8'h4e;
        labels[340]=8'h49;
        labels[341]=8'h4e;
        labels[342]=8'h47;
        // 22: START UNKNOWN
        labels[352]=8'h53;
        labels[353]=8'h54;
        labels[354]=8'h41;
        labels[355]=8'h52;
        labels[356]=8'h54;
        labels[358]=8'h55;
        labels[359]=8'h4e;
        labels[360]=8'h4b;
        labels[361]=8'h4e;
        labels[362]=8'h4f;
        labels[363]=8'h57;
        labels[364]=8'h4e;
        // 23: GOAL UNKNOWN
        labels[368]=8'h47;
        labels[369]=8'h4f;
        labels[370]=8'h41;
        labels[371]=8'h4c;
        labels[373]=8'h55;
        labels[374]=8'h4e;
        labels[375]=8'h4b;
        labels[376]=8'h4e;
        labels[377]=8'h4f;
        labels[378]=8'h57;
        labels[379]=8'h4e;
        // 24: MOVING
        labels[384]=8'h4d;
        labels[385]=8'h4f;
        labels[386]=8'h56;
        labels[387]=8'h49;
        labels[388]=8'h4e;
        labels[389]=8'h47;
        // 25: ARRIVED
        labels[400]=8'h41;
        labels[401]=8'h52;
        labels[402]=8'h52;
        labels[403]=8'h49;
        labels[404]=8'h56;
        labels[405]=8'h45;
        labels[406]=8'h44;
        // 26: PI LINK WAIT
        labels[416]=8'h50;
        labels[417]=8'h49;
        labels[419]=8'h4c;
        labels[420]=8'h49;
        labels[421]=8'h4e;
        labels[422]=8'h4b;
        labels[424]=8'h57;
        labels[425]=8'h41;
        labels[426]=8'h49;
        labels[427]=8'h54;
        // 27: POWER OFF WAIT
        labels[432]=8'h50;
        labels[433]=8'h4f;
        labels[434]=8'h57;
        labels[435]=8'h45;
        labels[436]=8'h52;
        labels[438]=8'h4f;
        labels[439]=8'h46;
        labels[440]=8'h46;
        labels[442]=8'h57;
        labels[443]=8'h41;
        labels[444]=8'h49;
        labels[445]=8'h54;
        // 28: WAIT LIDAR SCAN
        labels[448]=8'h57;
        labels[449]=8'h41;
        labels[450]=8'h49;
        labels[451]=8'h54;
        labels[453]=8'h4c;
        labels[454]=8'h49;
        labels[455]=8'h44;
        labels[456]=8'h41;
        labels[457]=8'h52;
        labels[459]=8'h53;
        labels[460]=8'h43;
        labels[461]=8'h41;
        labels[462]=8'h4e;
        // 29: WAIT GYRO ZERO
        labels[464]=8'h57;
        labels[465]=8'h41;
        labels[466]=8'h49;
        labels[467]=8'h54;
        labels[469]=8'h47;
        labels[470]=8'h59;
        labels[471]=8'h52;
        labels[472]=8'h4f;
        labels[474]=8'h5a;
        labels[475]=8'h45;
        labels[476]=8'h52;
        labels[477]=8'h4f;
        // 30: CHECK MOTOR PID
        labels[480]=8'h43;
        labels[481]=8'h48;
        labels[482]=8'h45;
        labels[483]=8'h43;
        labels[484]=8'h4b;
        labels[486]=8'h4d;
        labels[487]=8'h4f;
        labels[488]=8'h54;
        labels[489]=8'h4f;
        labels[490]=8'h52;
        labels[492]=8'h50;
        labels[493]=8'h49;
        labels[494]=8'h44;
        // 31: GOAL OUTSIDE MAP
        labels[496]=8'h47;
        labels[497]=8'h4f;
        labels[498]=8'h41;
        labels[499]=8'h4c;
        labels[501]=8'h4f;
        labels[502]=8'h55;
        labels[503]=8'h54;
        labels[504]=8'h53;
        labels[505]=8'h49;
        labels[506]=8'h44;
        labels[507]=8'h45;
        labels[509]=8'h4d;
        labels[510]=8'h41;
        labels[511]=8'h50;
        // 32: SEARCHING DOG
        labels[512]=8'h53;
        labels[513]=8'h45;
        labels[514]=8'h41;
        labels[515]=8'h52;
        labels[516]=8'h43;
        labels[517]=8'h48;
        labels[518]=8'h49;
        labels[519]=8'h4e;
        labels[520]=8'h47;
        labels[522]=8'h44;
        labels[523]=8'h4f;
        labels[524]=8'h47;
        // 33: SEARCH TURN 90
        labels[528]=8'h53;
        labels[529]=8'h45;
        labels[530]=8'h41;
        labels[531]=8'h52;
        labels[532]=8'h43;
        labels[533]=8'h48;
        labels[535]=8'h54;
        labels[536]=8'h55;
        labels[537]=8'h52;
        labels[538]=8'h4e;
        labels[540]=8'h39;
        labels[541]=8'h30;
        // 34: DOG NOT FOUND
        labels[544]=8'h44;
        labels[545]=8'h4f;
        labels[546]=8'h47;
        labels[548]=8'h4e;
        labels[549]=8'h4f;
        labels[550]=8'h54;
        labels[552]=8'h46;
        labels[553]=8'h4f;
        labels[554]=8'h55;
        labels[555]=8'h4e;
        labels[556]=8'h44;
        // 35: CAMERA FRONT SET
        labels[560]=8'h43;
        labels[561]=8'h41;
        labels[562]=8'h4d;
        labels[563]=8'h45;
        labels[564]=8'h52;
        labels[565]=8'h41;
        labels[567]=8'h46;
        labels[568]=8'h52;
        labels[569]=8'h4f;
        labels[570]=8'h4e;
        labels[571]=8'h54;
        labels[573]=8'h53;
        labels[574]=8'h45;
        labels[575]=8'h54;
        // 36: SET CAMERA FRONT
        labels[576]=8'h53;
        labels[577]=8'h45;
        labels[578]=8'h54;
        labels[580]=8'h43;
        labels[581]=8'h41;
        labels[582]=8'h4d;
        labels[583]=8'h45;
        labels[584]=8'h52;
        labels[585]=8'h41;
        labels[587]=8'h46;
        labels[588]=8'h52;
        labels[589]=8'h4f;
        labels[590]=8'h4e;
        labels[591]=8'h54;
        // 37: LASER AMBIGUOUS
        labels[592]=8'h4c;
        labels[593]=8'h41;
        labels[594]=8'h53;
        labels[595]=8'h45;
        labels[596]=8'h52;
        labels[598]=8'h41;
        labels[599]=8'h4d;
        labels[600]=8'h42;
        labels[601]=8'h49;
        labels[602]=8'h47;
        labels[603]=8'h55;
        labels[604]=8'h4f;
        labels[605]=8'h55;
        labels[606]=8'h53;
        // 38: WAIT LASER MATCH
        labels[608]=8'h57;
        labels[609]=8'h41;
        labels[610]=8'h49;
        labels[611]=8'h54;
        labels[613]=8'h4c;
        labels[614]=8'h41;
        labels[615]=8'h53;
        labels[616]=8'h45;
        labels[617]=8'h52;
        labels[619]=8'h4d;
        labels[620]=8'h41;
        labels[621]=8'h54;
        labels[622]=8'h43;
        labels[623]=8'h48;
        // 39: DOG POSITION SET
        labels[624]=8'h44;
        labels[625]=8'h4f;
        labels[626]=8'h47;
        labels[628]=8'h50;
        labels[629]=8'h4f;
        labels[630]=8'h53;
        labels[631]=8'h49;
        labels[632]=8'h54;
        labels[633]=8'h49;
        labels[634]=8'h4f;
        labels[635]=8'h4e;
        labels[637]=8'h53;
        labels[638]=8'h45;
        labels[639]=8'h54;
        // 40: SIX TASK DEMOS
        labels[640]=8'h53;
        labels[641]=8'h49;
        labels[642]=8'h58;
        labels[644]=8'h54;
        labels[645]=8'h41;
        labels[646]=8'h53;
        labels[647]=8'h4b;
        labels[649]=8'h44;
        labels[650]=8'h45;
        labels[651]=8'h4d;
        labels[652]=8'h4f;
        labels[653]=8'h53;
        // 41: 1 CAMERA + IMU
        labels[656]=8'h31;
        labels[658]=8'h43;
        labels[659]=8'h41;
        labels[660]=8'h4d;
        labels[661]=8'h45;
        labels[662]=8'h52;
        labels[663]=8'h41;
        labels[665]=8'h2b;
        labels[667]=8'h49;
        labels[668]=8'h4d;
        labels[669]=8'h55;
        // 42: 2 MOTOR CONTROL
        labels[672]=8'h32;
        labels[674]=8'h4d;
        labels[675]=8'h4f;
        labels[676]=8'h54;
        labels[677]=8'h4f;
        labels[678]=8'h52;
        labels[680]=8'h43;
        labels[681]=8'h4f;
        labels[682]=8'h4e;
        labels[683]=8'h54;
        labels[684]=8'h52;
        labels[685]=8'h4f;
        labels[686]=8'h4c;
        // 43: 3 LIDAR AVOID
        labels[688]=8'h33;
        labels[690]=8'h4c;
        labels[691]=8'h49;
        labels[692]=8'h44;
        labels[693]=8'h41;
        labels[694]=8'h52;
        labels[696]=8'h41;
        labels[697]=8'h56;
        labels[698]=8'h4f;
        labels[699]=8'h49;
        labels[700]=8'h44;
        // 44: 4 XY NAVIGATION
        labels[704]=8'h34;
        labels[706]=8'h58;
        labels[707]=8'h59;
        labels[709]=8'h4e;
        labels[710]=8'h41;
        labels[711]=8'h56;
        labels[712]=8'h49;
        labels[713]=8'h47;
        labels[714]=8'h41;
        labels[715]=8'h54;
        labels[716]=8'h49;
        labels[717]=8'h4f;
        labels[718]=8'h4e;
        // 45: 5 GIMBAL TRACK
        labels[720]=8'h35;
        labels[722]=8'h47;
        labels[723]=8'h49;
        labels[724]=8'h4d;
        labels[725]=8'h42;
        labels[726]=8'h41;
        labels[727]=8'h4c;
        labels[729]=8'h54;
        labels[730]=8'h52;
        labels[731]=8'h41;
        labels[732]=8'h43;
        labels[733]=8'h4b;
        // 46: 6 FIND FACE
        labels[736]=8'h36;
        labels[738]=8'h46;
        labels[739]=8'h49;
        labels[740]=8'h4e;
        labels[741]=8'h44;
        labels[743]=8'h46;
        labels[744]=8'h41;
        labels[745]=8'h43;
        labels[746]=8'h45;
        // 47: X FRONT Y LEFT
        labels[752]=8'h58;
        labels[754]=8'h46;
        labels[755]=8'h52;
        labels[756]=8'h4f;
        labels[757]=8'h4e;
        labels[758]=8'h54;
        labels[760]=8'h59;
        labels[762]=8'h4c;
        labels[763]=8'h45;
        labels[764]=8'h46;
        labels[765]=8'h54;
        // 48: MM / DEGREES
        labels[768]=8'h4d;
        labels[769]=8'h4d;
        labels[771]=8'h2f;
        labels[773]=8'h44;
        labels[774]=8'h45;
        labels[775]=8'h47;
        labels[776]=8'h52;
        labels[777]=8'h45;
        labels[778]=8'h45;
        labels[779]=8'h53;
        // 49: ORIGIN SET
        labels[784]=8'h4f;
        labels[785]=8'h52;
        labels[786]=8'h49;
        labels[787]=8'h47;
        labels[788]=8'h49;
        labels[789]=8'h4e;
        labels[791]=8'h53;
        labels[792]=8'h45;
        labels[793]=8'h54;
        // 50: MOTOR ERROR
        labels[800]=8'h4d;
        labels[801]=8'h4f;
        labels[802]=8'h54;
        labels[803]=8'h4f;
        labels[804]=8'h52;
        labels[806]=8'h45;
        labels[807]=8'h52;
        labels[808]=8'h52;
        labels[809]=8'h4f;
        labels[810]=8'h52;
        // 51: ALL STOP
        labels[816]=8'h41;
        labels[817]=8'h4c;
        labels[818]=8'h4c;
        labels[820]=8'h53;
        labels[821]=8'h54;
        labels[822]=8'h4f;
        labels[823]=8'h50;
        // 52: PID P30 I1 D0
        labels[832]=8'h50;
        labels[833]=8'h49;
        labels[834]=8'h44;
        labels[836]=8'h50;
        labels[837]=8'h33;
        labels[838]=8'h30;
        labels[840]=8'h49;
        labels[841]=8'h31;
        labels[843]=8'h44;
        labels[844]=8'h30;
        // 53: LIDAR LIVE
        labels[848]=8'h4c;
        labels[849]=8'h49;
        labels[850]=8'h44;
        labels[851]=8'h41;
        labels[852]=8'h52;
        labels[854]=8'h4c;
        labels[855]=8'h49;
        labels[856]=8'h56;
        labels[857]=8'h45;
        // 54: LIDAR OFF
        labels[864]=8'h4c;
        labels[865]=8'h49;
        labels[866]=8'h44;
        labels[867]=8'h41;
        labels[868]=8'h52;
        labels[870]=8'h4f;
        labels[871]=8'h46;
        labels[872]=8'h46;
        // 55: PLANNING
        labels[880]=8'h50;
        labels[881]=8'h4c;
        labels[882]=8'h41;
        labels[883]=8'h4e;
        labels[884]=8'h4e;
        labels[885]=8'h49;
        labels[886]=8'h4e;
        labels[887]=8'h47;
        // 56: NO ROUTE
        labels[896]=8'h4e;
        labels[897]=8'h4f;
        labels[899]=8'h52;
        labels[900]=8'h4f;
        labels[901]=8'h55;
        labels[902]=8'h54;
        labels[903]=8'h45;
        // 57: OBSTACLE
        labels[912]=8'h4f;
        labels[913]=8'h42;
        labels[914]=8'h53;
        labels[915]=8'h54;
        labels[916]=8'h41;
        labels[917]=8'h43;
        labels[918]=8'h4c;
        labels[919]=8'h45;
        // 58: DOG SIDE FOLLOW
        labels[928]=8'h44;
        labels[929]=8'h4f;
        labels[930]=8'h47;
        labels[932]=8'h53;
        labels[933]=8'h49;
        labels[934]=8'h44;
        labels[935]=8'h45;
        labels[937]=8'h46;
        labels[938]=8'h4f;
        labels[939]=8'h4c;
        labels[940]=8'h4c;
        labels[941]=8'h4f;
        labels[942]=8'h57;
        // 59: GOAL X Y MM
        labels[944]=8'h47;
        labels[945]=8'h4f;
        labels[946]=8'h41;
        labels[947]=8'h4c;
        labels[949]=8'h58;
        labels[951]=8'h59;
        labels[953]=8'h4d;
        labels[954]=8'h4d;
        // 60: LIDAR MAP
        labels[960]=8'h4c;
        labels[961]=8'h49;
        labels[962]=8'h44;
        labels[963]=8'h41;
        labels[964]=8'h52;
        labels[966]=8'h4d;
        labels[967]=8'h41;
        labels[968]=8'h50;
        // 61: SENSOR WAIT
        labels[976]=8'h53;
        labels[977]=8'h45;
        labels[978]=8'h4e;
        labels[979]=8'h53;
        labels[980]=8'h4f;
        labels[981]=8'h52;
        labels[983]=8'h57;
        labels[984]=8'h41;
        labels[985]=8'h49;
        labels[986]=8'h54;
        // 65: X MM
        labels[1040]=8'h58;
        labels[1042]=8'h4d;
        labels[1043]=8'h4d;
        // 66: Y MM
        labels[1056]=8'h59;
        labels[1058]=8'h4d;
        labels[1059]=8'h4d;
        // 67: YAW
        labels[1072]=8'h59;
        labels[1073]=8'h41;
        labels[1074]=8'h57;
        // 68: GX MM
        labels[1088]=8'h47;
        labels[1089]=8'h58;
        labels[1091]=8'h4d;
        labels[1092]=8'h4d;
        // 69: GY MM
        labels[1104]=8'h47;
        labels[1105]=8'h59;
        labels[1107]=8'h4d;
        labels[1108]=8'h4d;
        // 70: ID
        labels[1120]=8'h49;
        labels[1121]=8'h44;
        // 71: DETECT %
        labels[1136]=8'h44;
        labels[1137]=8'h45;
        labels[1138]=8'h54;
        labels[1139]=8'h45;
        labels[1140]=8'h43;
        labels[1141]=8'h54;
        labels[1143]=8'h25;
        // 72: VX MM
        labels[1152]=8'h56;
        labels[1153]=8'h58;
        labels[1155]=8'h4d;
        labels[1156]=8'h4d;
        // 73: VY MM
        labels[1168]=8'h56;
        labels[1169]=8'h59;
        labels[1171]=8'h4d;
        labels[1172]=8'h4d;
        // 74: CMD L
        labels[1184]=8'h43;
        labels[1185]=8'h4d;
        labels[1186]=8'h44;
        labels[1188]=8'h4c;
        // 75: CMD R
        labels[1200]=8'h43;
        labels[1201]=8'h4d;
        labels[1202]=8'h44;
        labels[1204]=8'h52;
        // 76: ACT L
        labels[1216]=8'h41;
        labels[1217]=8'h43;
        labels[1218]=8'h54;
        labels[1220]=8'h4c;
        // 77: ACT R
        labels[1232]=8'h41;
        labels[1233]=8'h43;
        labels[1234]=8'h54;
        labels[1236]=8'h52;
        // 78: FRONT
        labels[1248]=8'h46;
        labels[1249]=8'h52;
        labels[1250]=8'h4f;
        labels[1251]=8'h4e;
        labels[1252]=8'h54;
        // 79: LEFT
        labels[1264]=8'h4c;
        labels[1265]=8'h45;
        labels[1266]=8'h46;
        labels[1267]=8'h54;
        // 80: RIGHT
        labels[1280]=8'h52;
        labels[1281]=8'h49;
        labels[1282]=8'h47;
        labels[1283]=8'h48;
        labels[1284]=8'h54;
        // 81: REAR
        labels[1296]=8'h52;
        labels[1297]=8'h45;
        labels[1298]=8'h41;
        labels[1299]=8'h52;
        // 82: RADAR
        labels[1312]=8'h52;
        labels[1313]=8'h41;
        labels[1314]=8'h44;
        labels[1315]=8'h41;
        labels[1316]=8'h52;
        // 83: STEP
        labels[1328]=8'h53;
        labels[1329]=8'h54;
        labels[1330]=8'h45;
        labels[1331]=8'h50;
        // 84: VEL
        labels[1344]=8'h56;
        labels[1345]=8'h45;
        labels[1346]=8'h4c;
        // 85: START BLOCKED
        labels[1360]=8'h53;
        labels[1361]=8'h54;
        labels[1362]=8'h41;
        labels[1363]=8'h52;
        labels[1364]=8'h54;
        labels[1366]=8'h42;
        labels[1367]=8'h4c;
        labels[1368]=8'h4f;
        labels[1369]=8'h43;
        labels[1370]=8'h4b;
        labels[1371]=8'h45;
        labels[1372]=8'h44;
        // 86: GOAL BLOCKED
        labels[1376]=8'h47;
        labels[1377]=8'h4f;
        labels[1378]=8'h41;
        labels[1379]=8'h4c;
        labels[1381]=8'h42;
        labels[1382]=8'h4c;
        labels[1383]=8'h4f;
        labels[1384]=8'h43;
        labels[1385]=8'h4b;
        labels[1386]=8'h45;
        labels[1387]=8'h44;
        // 87: NO KNOWN PATH
        labels[1392]=8'h4e;
        labels[1393]=8'h4f;
        labels[1395]=8'h4b;
        labels[1396]=8'h4e;
        labels[1397]=8'h4f;
        labels[1398]=8'h57;
        labels[1399]=8'h4e;
        labels[1401]=8'h50;
        labels[1402]=8'h41;
        labels[1403]=8'h54;
        labels[1404]=8'h48;
        // 88: PLAN TIMEOUT
        labels[1408]=8'h50;
        labels[1409]=8'h4c;
        labels[1410]=8'h41;
        labels[1411]=8'h4e;
        labels[1413]=8'h54;
        labels[1414]=8'h49;
        labels[1415]=8'h4d;
        labels[1416]=8'h45;
        labels[1417]=8'h4f;
        labels[1418]=8'h55;
        labels[1419]=8'h54;
        // 89: START OFF MAP
        labels[1424]=8'h53;
        labels[1425]=8'h54;
        labels[1426]=8'h41;
        labels[1427]=8'h52;
        labels[1428]=8'h54;
        labels[1430]=8'h4f;
        labels[1431]=8'h46;
        labels[1432]=8'h46;
        labels[1434]=8'h4d;
        labels[1435]=8'h41;
        labels[1436]=8'h50;
        // 90: GOAL OFF MAP
        labels[1440]=8'h47;
        labels[1441]=8'h4f;
        labels[1442]=8'h41;
        labels[1443]=8'h4c;
        labels[1445]=8'h4f;
        labels[1446]=8'h46;
        labels[1447]=8'h46;
        labels[1449]=8'h4d;
        labels[1450]=8'h41;
        labels[1451]=8'h50;
        // 91: WAIT MAP SCANS
        labels[1456]=8'h57;
        labels[1457]=8'h41;
        labels[1458]=8'h49;
        labels[1459]=8'h54;
        labels[1461]=8'h4d;
        labels[1462]=8'h41;
        labels[1463]=8'h50;
        labels[1465]=8'h53;
        labels[1466]=8'h43;
        labels[1467]=8'h41;
        labels[1468]=8'h4e;
        labels[1469]=8'h53;
        // 92: MAP WAIT TIMEOUT
        labels[1472]=8'h4d;
        labels[1473]=8'h41;
        labels[1474]=8'h50;
        labels[1476]=8'h57;
        labels[1477]=8'h41;
        labels[1478]=8'h49;
        labels[1479]=8'h54;
        labels[1481]=8'h54;
        labels[1482]=8'h49;
        labels[1483]=8'h4d;
        labels[1484]=8'h45;
        labels[1485]=8'h4f;
        labels[1486]=8'h55;
        labels[1487]=8'h54;
        // 93: POSE NOT READY
        labels[1488]=8'h50;
        labels[1489]=8'h4f;
        labels[1490]=8'h53;
        labels[1491]=8'h45;
        labels[1493]=8'h4e;
        labels[1494]=8'h4f;
        labels[1495]=8'h54;
        labels[1497]=8'h52;
        labels[1498]=8'h45;
        labels[1499]=8'h41;
        labels[1500]=8'h44;
        labels[1501]=8'h59;
        // 94: MAP NOT READY
        labels[1504]=8'h4d;
        labels[1505]=8'h41;
        labels[1506]=8'h50;
        labels[1508]=8'h4e;
        labels[1509]=8'h4f;
        labels[1510]=8'h54;
        labels[1512]=8'h52;
        labels[1513]=8'h45;
        labels[1514]=8'h41;
        labels[1515]=8'h44;
        labels[1516]=8'h59;
        // 95: CONTACT STOP
        labels[1520]=8'h43;
        labels[1521]=8'h4f;
        labels[1522]=8'h4e;
        labels[1523]=8'h54;
        labels[1524]=8'h41;
        labels[1525]=8'h43;
        labels[1526]=8'h54;
        labels[1528]=8'h53;
        labels[1529]=8'h54;
        labels[1530]=8'h4f;
        labels[1531]=8'h50;
        // 96: FINAL LEG BLOCK
        labels[1536]=8'h46;
        labels[1537]=8'h49;
        labels[1538]=8'h4e;
        labels[1539]=8'h41;
        labels[1540]=8'h4c;
        labels[1542]=8'h4c;
        labels[1543]=8'h45;
        labels[1544]=8'h47;
        labels[1546]=8'h42;
        labels[1547]=8'h4c;
        labels[1548]=8'h4f;
        labels[1549]=8'h43;
        labels[1550]=8'h4b;
        // 97: CHECK SENSORS
        labels[1552]=8'h43;
        labels[1553]=8'h48;
        labels[1554]=8'h45;
        labels[1555]=8'h43;
        labels[1556]=8'h4b;
        labels[1558]=8'h53;
        labels[1559]=8'h45;
        labels[1560]=8'h4e;
        labels[1561]=8'h53;
        labels[1562]=8'h4f;
        labels[1563]=8'h52;
        labels[1564]=8'h53;
        // 98: NAV MATH FAULT
        labels[1568]=8'h4e;
        labels[1569]=8'h41;
        labels[1570]=8'h56;
        labels[1572]=8'h4d;
        labels[1573]=8'h41;
        labels[1574]=8'h54;
        labels[1575]=8'h48;
        labels[1577]=8'h46;
        labels[1578]=8'h41;
        labels[1579]=8'h55;
        labels[1580]=8'h4c;
        labels[1581]=8'h54;
        // 99: OUTSIDE MAP
        labels[1584]=8'h4f;
        labels[1585]=8'h55;
        labels[1586]=8'h54;
        labels[1587]=8'h53;
        labels[1588]=8'h49;
        labels[1589]=8'h44;
        labels[1590]=8'h45;
        labels[1592]=8'h4d;
        labels[1593]=8'h41;
        labels[1594]=8'h50;
        // 100: NO FACE IN VIEW
        labels[1600]=8'h4e;
        labels[1601]=8'h4f;
        labels[1603]=8'h46;
        labels[1604]=8'h41;
        labels[1605]=8'h43;
        labels[1606]=8'h45;
        labels[1608]=8'h49;
        labels[1609]=8'h4e;
        labels[1611]=8'h56;
        labels[1612]=8'h49;
        labels[1613]=8'h45;
        labels[1614]=8'h57;
        // 101: FACE SCORE LOW
        labels[1616]=8'h46;
        labels[1617]=8'h41;
        labels[1618]=8'h43;
        labels[1619]=8'h45;
        labels[1621]=8'h53;
        labels[1622]=8'h43;
        labels[1623]=8'h4f;
        labels[1624]=8'h52;
        labels[1625]=8'h45;
        labels[1627]=8'h4c;
        labels[1628]=8'h4f;
        labels[1629]=8'h57;
        // 102: WAIT FACE STABLE
        labels[1632]=8'h57;
        labels[1633]=8'h41;
        labels[1634]=8'h49;
        labels[1635]=8'h54;
        labels[1637]=8'h46;
        labels[1638]=8'h41;
        labels[1639]=8'h43;
        labels[1640]=8'h45;
        labels[1642]=8'h53;
        labels[1643]=8'h54;
        labels[1644]=8'h41;
        labels[1645]=8'h42;
        labels[1646]=8'h4c;
        labels[1647]=8'h45;
        // 103: FACE TRACKING
        labels[1648]=8'h46;
        labels[1649]=8'h41;
        labels[1650]=8'h43;
        labels[1651]=8'h45;
        labels[1653]=8'h54;
        labels[1654]=8'h52;
        labels[1655]=8'h41;
        labels[1656]=8'h43;
        labels[1657]=8'h4b;
        labels[1658]=8'h49;
        labels[1659]=8'h4e;
        labels[1660]=8'h47;
        // 104: SEARCHING FACE
        labels[1664]=8'h53;
        labels[1665]=8'h45;
        labels[1666]=8'h41;
        labels[1667]=8'h52;
        labels[1668]=8'h43;
        labels[1669]=8'h48;
        labels[1670]=8'h49;
        labels[1671]=8'h4e;
        labels[1672]=8'h47;
        labels[1674]=8'h46;
        labels[1675]=8'h41;
        labels[1676]=8'h43;
        labels[1677]=8'h45;
        // 105: FACE NOT FOUND
        labels[1680]=8'h46;
        labels[1681]=8'h41;
        labels[1682]=8'h43;
        labels[1683]=8'h45;
        labels[1685]=8'h4e;
        labels[1686]=8'h4f;
        labels[1687]=8'h54;
        labels[1689]=8'h46;
        labels[1690]=8'h4f;
        labels[1691]=8'h55;
        labels[1692]=8'h4e;
        labels[1693]=8'h44;
        // 106: FACE POSITION
        labels[1696]=8'h46;
        labels[1697]=8'h41;
        labels[1698]=8'h43;
        labels[1699]=8'h45;
        labels[1701]=8'h50;
        labels[1702]=8'h4f;
        labels[1703]=8'h53;
        labels[1704]=8'h49;
        labels[1705]=8'h54;
        labels[1706]=8'h49;
        labels[1707]=8'h4f;
        labels[1708]=8'h4e;
        // 107: HEAD TO 135
        labels[1712]=8'h48;
        labels[1713]=8'h45;
        labels[1714]=8'h41;
        labels[1715]=8'h44;
        labels[1717]=8'h54;
        labels[1718]=8'h4f;
        labels[1720]=8'h31;
        labels[1721]=8'h33;
        labels[1722]=8'h35;
        // 108: MOTION PAUSED
        labels[1728]=8'h4d;
        labels[1729]=8'h4f;
        labels[1730]=8'h54;
        labels[1731]=8'h49;
        labels[1732]=8'h4f;
        labels[1733]=8'h4e;
        labels[1735]=8'h50;
        labels[1736]=8'h41;
        labels[1737]=8'h55;
        labels[1738]=8'h53;
        labels[1739]=8'h45;
        labels[1740]=8'h44;
        // 109: WAIT CLEAR PATH
        labels[1744]=8'h57;
        labels[1745]=8'h41;
        labels[1746]=8'h49;
        labels[1747]=8'h54;
        labels[1749]=8'h43;
        labels[1750]=8'h4c;
        labels[1751]=8'h45;
        labels[1752]=8'h41;
        labels[1753]=8'h52;
        labels[1755]=8'h50;
        labels[1756]=8'h41;
        labels[1757]=8'h54;
        labels[1758]=8'h48;
    end

    // Six frozen20-bit values, one serial BCD converter and one2KB byte ROM.
    // No PID fractional formatter, debug capture bank, page editor, parallel
    // numeric conversions or live32-bit23-way field multiplexer remains.
    localparam S_IDLE=0,S_ROW=1,S_CONVERT=2,S_REQUEST=3,S_SEND=4,S_COMMIT=5;
    reg [2:0] state;
    reg [31:0] countdown,held_flags;
    reg [3:0] row,column;
    reg [6:0] title_id,status_id,current_label;
    reg [6:0] prefixes[0:5];
    reg signed [19:0] values[0:5];
    reg [5:0] numeric_mask,unknown_mask,degrees_mask;
    reg numeric_row,unknown_value,negative,started;
    reg [19:0] shift;
    reg [23:0] bcd,render_bcd;
    reg [4:0] bit_count;
    reg [7:0] label_byte;
    wire [5:0] view={page,subpage};
    wire [2:0] value_index=row[2:0]-3'd2;
    wire signed [19:0] selected_raw=values[value_index];
    wire [19:0] magnitude=selected_raw[19] ? -selected_raw : selected_raw;
    wire [23:0] adjusted_bcd;
    genvar digit;
    generate for(digit=0;digit<6;digit=digit+1)begin:bcd_adjust
        assign adjusted_bcd[digit*4+:4]=bcd[digit*4+:4]>=5 ? bcd[digit*4+:4]+4'd3 : bcd[digit*4+:4];
    end endgenerate
    wire [23:0] next_bcd={adjusted_bcd[22:0],shift[19]};
    wire [3:0] current_digit=render_bcd[23:20];
    wire [7:0] digit_byte=unknown_value ? 8'h2d : !started && current_digit==0 && column!=12 ? 8'h20 : 8'h30+{4'd0,current_digit};
    wire [7:0] numeric_byte=column==6 ? (unknown_value ? 8'h20 : negative ? 8'h2d : 8'h2b) :
        column>=7 && column<=12 ? digit_byte : 8'h20;
    assign text_byte_valid=state==S_SEND;
    assign text_byte_data=numeric_row && column>=5 ? numeric_byte : label_byte;
    assign text_byte_index={row,column};
    function [6:0] navigation_label;
        input [3:0] s;
        begin case(s)
            1:navigation_label=29;2:navigation_label=30;3,5:navigation_label=24;
            4:navigation_label=21;6:navigation_label=25;7:navigation_label=57;
            8:navigation_label=61;9:navigation_label=50;10:navigation_label=55;
            11:navigation_label=56;default:navigation_label=20;
        endcase end
    endfunction
    function [6:0] planner_label;
        input [7:0] s;
        begin case(s)
            1:planner_label=88;2:planner_label=85;3:planner_label=86;4:planner_label=87;
            8:planner_label=22;9:planner_label=23;11:planner_label=89;12:planner_label=90;
            13:planner_label=92;14:planner_label=93;15:planner_label=94;16:planner_label=95;
            17:planner_label=96;18:planner_label=97;19:planner_label=98;20:planner_label=99;21:planner_label=108;
            default:planner_label=56;
        endcase end
    endfunction
    reg [6:0] live_status,live_title;
    always @* begin
        live_title=40;
        case(view)
            1:live_title=41;32:live_title=42;19:live_title=43;33:live_title=44;
            40:live_title=45;43:live_title=46;44:live_title=58;27:live_title=59;17:live_title=60;
            default:begin end
        endcase
        live_status=armed ? navigation_label(nav_status) : 7'd20;
        if(ui_notice==9)live_status=49;
        else if(ui_notice==2)live_status=30;
        else if(ui_notice==3 || calibrating)live_status=29;
        else if(ui_notice==5)live_status=28;
        if((view==33 || view==43) && nav_status==11)live_status=planner_label(planner_error);
        if((view==33 || view==43) && waiting_map)live_status=91;
        else if((view==33 || view==43) && waiting_path && planner_error==0)live_status=109;
        if(page==5)begin
            live_status=follow_diagnostic<=39 && follow_diagnostic!=0 ? {1'b0,follow_diagnostic[5:0]} :
                follow_state==4 ? 7'd16 : follow_state==8 ? 7'd24 : 7'd9;
            if(follow_diagnostic>=40 && follow_diagnostic<=47)live_status=7'd100+{1'b0,follow_diagnostic[5:0]}-7'd40;
            if(armed && nav_status>=4 && follow_state<12)live_status=nav_status==11 ? planner_label(planner_error) : navigation_label(nav_status);
            if(follow_state==0)live_status=26;
            if(follow_state==12)live_status=27;
            if(follow_state==13)live_status=14;
            if(follow_state==21)live_status=(follow_diagnostic==12 || follow_diagnostic==13) ? {1'b0,follow_diagnostic[5:0]} : 7'd19;
        end
        if(motor_fault!=0)live_status=50;
        if(safety_stop)live_status=51;
    end
    integer field;
    generate if(PARALLEL_COMPAT!=0)begin:compatibility_only
        reg [2047:0] work,published;
        assign text_rows=published;
        always @(posedge clk or negedge reset_n)begin
            if(!reset_n)begin work<=0;published<=0;end
            else begin
                if(text_byte_valid && text_byte_ready)work[2047-({row,column}*8)-:8]<=text_byte_data;
                if(state==S_COMMIT)published<=work;
            end
        end
    end else begin:no_parallel_display
        assign text_rows=2048'd0;
    end endgenerate
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)begin
            state<=S_IDLE;countdown<=0;row<=0;column<=0;title_id<=0;status_id<=20;current_label<=0;
            held_flags<=0;flags<=0;text_valid<=0;numeric_mask<=0;unknown_mask<=0;degrees_mask<=0;
            numeric_row<=0;unknown_value<=0;negative<=0;started<=0;
            shift<=0;bcd<=0;render_bcd<=0;bit_count<=0;label_byte<=8'h20;
            for(field=0;field<6;field=field+1)begin values[field]<=0;prefixes[field]<=0;end
        end else begin
            text_valid<=0;
            if(countdown!=0)countdown<=countdown-1'b1;
            case(state)
                S_IDLE:if(countdown==0)begin
                    countdown<=CLK_HZ/10;
                    // Stable wire contract used by panel, map and motor owners.
                    held_flags<={gains_ready,subpage,lidar_error[4:0],nav_fault,nav_status,map_ready,safety_stop,scan_enable,
                        lidar_ready,calibrating,gyro_calibrated,geometry_valid,armed,page};
                    title_id<=live_title;status_id<=live_status;
                    numeric_mask<=0;unknown_mask<=0;degrees_mask<=0;
                    for(field=0;field<6;field=field+1)begin values[field]<=0;prefixes[field]<=0;end
                    case(view)
                        32:begin
                            numeric_mask<=6'b011111;degrees_mask[4]<=1;
                            values[0]<=$signed({4'd0,manual_speed_left});prefixes[0]<=74;
                            values[1]<=$signed({4'd0,manual_speed_right});prefixes[1]<=75;
                            values[2]<=speed_left[19:0];prefixes[2]<=76;
                            values[3]<=speed_right[19:0];prefixes[3]<=77;
                            values[4]<=heading_mdeg[19:0];prefixes[4]<=67;
                        end
                        33:begin
                            numeric_mask<=6'b011111;degrees_mask[2]<=1;
                            values[0]<=x_mm[19:0];prefixes[0]<=65;values[1]<=y_mm[19:0];prefixes[1]<=66;
                            values[2]<=heading_mdeg[19:0];prefixes[2]<=67;
                            values[3]<=goal_x_mm[19:0];prefixes[3]<=68;values[4]<=goal_y_mm[19:0];prefixes[4]<=69;
                        end
                        27:begin
                            numeric_mask<=6'b000111;
                            values[0]<=goal_x_mm[19:0];prefixes[0]<=68;values[1]<=goal_y_mm[19:0];prefixes[1]<=69;
                            values[2]<=selected_step[19:0];prefixes[2]<=83;
                        end
                        19:begin
                            numeric_mask<=6'b011111;
                            values[0]<=$signed({4'd0,front_mm});prefixes[0]<=78;
                            values[1]<=$signed({4'd0,left_mm});prefixes[1]<=79;
                            values[2]<=$signed({4'd0,right_mm});prefixes[2]<=80;
                            values[3]<=$signed({4'd0,rear_mm});prefixes[3]<=81;
                            values[4]<=$signed({11'd0,lidar_zero_deg});prefixes[4]<=82;
                            unknown_mask[3:0]<={rear_mm<50 || rear_mm>16000,right_mm<50 || right_mm>16000,left_mm<50 || left_mm>16000,front_mm<50 || front_mm>16000};
                        end
                        17:begin
                            numeric_mask<=6'b111111;degrees_mask[2]<=1;
                            values[0]<=x_mm[19:0];prefixes[0]<=65;values[1]<=y_mm[19:0];prefixes[1]<=66;
                            values[2]<=heading_mdeg[19:0];prefixes[2]<=67;
                            values[3]<=$signed({4'd0,front_mm});prefixes[3]<=78;
                            values[4]<=$signed({4'd0,left_mm});prefixes[4]<=79;
                            values[5]<=$signed({4'd0,right_mm});prefixes[5]<=80;
                            unknown_mask[5:3]<={right_mm<50 || right_mm>16000,left_mm<50 || left_mm>16000,front_mm<50 || front_mm>16000};
                        end
                        40,43,44:begin
                            numeric_mask<=view==43 ? 6'b001111 : view==44 ? 6'b000111 : 6'b000011;
                            values[0]<=$signed({4'd0,follow_selected});prefixes[0]<=70;
                            values[1]<=$signed({12'd0,follow_dog_score});prefixes[1]<=71;
                            if(view==43)begin
                                values[2]<={{4{visual_x[15]}},visual_x};prefixes[2]<=72;
                                values[3]<={{4{visual_y[15]}},visual_y};prefixes[3]<=73;
                            end else begin values[2]<={{4{follow_speed[15]}},follow_speed};prefixes[2]<=84;end
                        end
                        default:begin end
                    endcase
                    row<=0;column<=0;state<=S_ROW;
                end
                S_ROW:begin
                    column<=0;started<=0;numeric_row<=0;unknown_value<=0;
                    if(row==0)begin current_label<=title_id;state<=S_REQUEST;end
                    else if(row==1)begin current_label<=status_id;state<=S_REQUEST;end
                    else if(row>=2 && row<=7 && numeric_mask[value_index])begin
                        current_label<=prefixes[value_index];numeric_row<=1;unknown_value<=unknown_mask[value_index];
                        negative<=selected_raw[19];shift<=magnitude;bcd<=0;bit_count<=0;state<=S_CONVERT;
                    end else begin
                        current_label<=row==8 ? 7'd52 : row==9 ? (!held_flags[8] ? 7'd54 : held_flags[7] ? 7'd53 : 7'd28) : row==10 ? 7'd47 : 7'd0;
                        state<=S_REQUEST;
                    end
                end
                S_CONVERT:begin
                    bcd<=next_bcd;shift<=shift<<1;
                    if(bit_count==19)begin
                        render_bcd<=degrees_mask[value_index] ? {12'd0,next_bcd[23:12]} : next_bcd;
                        state<=S_REQUEST;
                    end else bit_count<=bit_count+1'b1;
                end
                S_REQUEST:begin label_byte<=labels[{current_label,column}];state<=S_SEND;end
                S_SEND:if(text_byte_ready)begin
                    if(numeric_row && column>=7 && column<=12)begin
                        if(current_digit!=0 || column==12)started<=1;
                        render_bcd<=render_bcd<<4;
                    end
                    if(column==15)begin
                        if(row==15)state<=S_COMMIT;
                        else begin row<=row+1'b1;column<=0;state<=S_ROW;end
                    end else begin column<=column+1'b1;state<=S_REQUEST;end
                end
                S_COMMIT:begin flags<=held_flags;text_valid<=1;state<=S_IDLE;end
                default:state<=S_IDLE;
            endcase
        end
    end
endmodule
