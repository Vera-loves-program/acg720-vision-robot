# 团队协作与开发

协作基线是2026-10-25的FPGA R7和Pi v22修订2。先读[README](README.md)与[文档索引](docs/README.md)，再选择自己负责的模块。

## 分支与提交

仓库所有者可在GitHub的Settings → Collaborators邀请队友；队友接受邀请后推送自己的分支。未获得写权限的成员可Fork后提交Pull Request。本文不自动邀请任何人。

每项改动建立分支，例如`feature/pi-face-tracking`、`fix/navigation-heading`、`docs/field-test`：

```bash
git switch main
git pull --ff-only origin main
git switch -c feature/your-task
# 修改并运行相关检查后：
git add path/to/changed-files
git commit -m "Describe the change and its purpose"
git push -u origin feature/your-task
```

在GitHub开Pull Request到`main`，写明触发场景、修改结果、运行的检查和尚未测过的硬件行为，由另一位成员审阅后合并。不要强推`main`；不要用reset/clean覆盖本机未提交文件。`pull --ff-only`失败时先检查双方提交，再在任务分支处理冲突。

## 模块分工与接口

| 范围 | 主要文件 |
| --- | --- |
| 里程计、直线与转向 | `src/robot_navigation.v` |
| 目标生命周期、规划和续走 | `src/robot_mission_control.v` |
| 雷达解析、建图、路径规划 | `src/rplidar_c1.v`、`lidar_grid_map.v`、`lidar_path_planner.v` |
| 硬件集成与控制权 | `src/vision_robot_ui_top.v`、`robot_subsystem.v` |
| Pi视觉、云台、通信与搜索 | `pc/usb_gimbal.py`、`gimbal_control.py`、`robot_follow.py`、`visual_task.py` |
| UI与协议 | `robot_ui_control`、`robot_status_text`、`udp_detection_feedback`及对应协议文档 |

修改FOL/TGT/VEC/GOL/RBT或VOFA字段时，FPGA与Pi两侧及协议文档同步更新。编码器系数、轮距、雷达安装和舵机标定应有团队共用的实测依据，不凭症状猜改。现场`config/gimbal.json`和`config/visual_task.json`留在设备上，通过团队已有渠道共享，不用示例覆盖。

实测报告记录代码commit、位流SHA、Pi manifest版本、测量方法、地面及光照条件。位置以两驱动轮中点O测量，不能用车头横移直接代替里程计Y误差。

## 本地检查

使用Python 3.10或更高版本，下列命令不启动底盘、不连接Pi或FPGA：

```bash
python -c "from pathlib import Path; Path('tmp').mkdir(exist_ok=True)"
python tools/check_navigation_tracking.py
python tools/check_mission_lifecycle.py
python tools/check_realtime_route.py
python tools/check_robot_navigation_math.py
python tests/test_orangepi_shell_bundle.py
```

Pi视觉相关检查需要已有NumPy/OpenCV环境，按修改范围选择：

```bash
python tools/check_independent_face_tracking.py
python tools/check_tilt_operating_window.py
python tools/check_robot_follow.py
```

Shell包检查会创建临时目录，Linux/macOS使用本机`sh`，Windows可使用Git for Windows的`sh.exe`；不执行服务安装或关机命令。部分历史构建工具仍含本机绝对路径，移机时先阅读脚本，不直接批量运行所有工具。

这些检查包含源码约束、协议Mock和独立数学模型，不能替代HDL动态仿真、高云完整时序分析或实车测试。与FPGA时序有关的修改应重新编译，并分别记录布线结果、同域裕量及跨域/复位违例。

## FPGA与Pi部署

Gowin 1.9.11.03 Education打开`vision_robot_ui.gprj`，器件为GW5AT-LV60PG484AC1/I0。`src/`、CST/SDC、厂商IP和三份`impl/*process_config.json`随仓库保留。不要在另一位成员编译时修改同一目录源码。

时钟/PLL/约束/厂商IP修改应说明具体原因，不用虚假false path掩盖真实违例。保留题1摄像头→DDR3→LCD和六轴IMU。编译、Programmer烧录、实际动作是独立步骤，实测动作由现场成员执行。

Pi代码包生成：

```bash
python tools/package_orangepi_client.py --code-only --output output/releases/acg720-orangepi-v22-code.zip
```

按[Pi部署说明](docs/香橙派v22人脸与小狗测试.md)上传并重启`acg720-follow.service`，确认`active (running)`和实际`FOLLOW runtime:`日志。现有工作环境不重装；新成员另行准备模型和实测标定。包保留现场模型、角度表和车头参考。版本打印22不代表服务已经启动。

`*.sh`必须使用LF换行，`.gitattributes`和打包器均做保护；不要叠加旧nohup进程。

## 提交范围

提交源码、配置示例、文档、必要测试及流程配置。`tmp/`、`archive/`、个人交接笔记、现场标定、环境密钥、照片/模型权重、`.fs`和生成ZIP已忽略。第三方文件保留原版权声明。
