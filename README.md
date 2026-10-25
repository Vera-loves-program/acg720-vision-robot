# ACG720 FPGA视觉机器人

高云ACG720 / GW5AT-60控制底盘、RPLIDAR C1、LCD和本地摄像头；Orange Pi 5负责USB相机识别、云台姿态解算和视觉任务，通过以太网与FPGA通信。当前源码对应 **FPGA R7 + Pi v22修订2**，更新于2026-10-25。

队友请先看[协作与开发指南](CONTRIBUTING.md)和[文档索引](docs/README.md)。本次同步保留本地与GitHub两边原有历史，以当前R7/v22代码作为协作起点。

| 任务 | 当前实现 |
| --- | --- |
| 1 本地摄像头 | OV5640→DDR3→LCD，保留已有缩放链和六轴IMU显示 |
| 2 底盘控制 | 前后移动、原地转向，速度环上电P30/I1/D0 |
| 3 雷达建图 | C1解析、车体/安装外参过滤、栅格地图 |
| 4 定点导航 | 一键原点/+X、固定直线段、规划与动态避障 |
| 5 云台跟踪 | Pi人脸/小狗检测和独立云台跟踪，初始135°、运行不低于90° |
| 6 目标任务 | 默认寻找人脸、云台搜索、底盘辅助转向、视觉方位/雷达簇匹配和接近目标 |

## 获取源码与构建

```bash
git clone https://github.com/Vera-loves-program/acg720-vision-robot.git
cd acg720-vision-robot
```

FPGA使用 **Gowin FPGA Designer 1.9.11.03 Education**，打开`vision_robot_ui.gprj`。项目、CST、SDC、厂商IP及三份流程配置随仓库保留；也可在安装目录内用`gw_sh.exe build.tcl`手动构建，编译不等于烧录。

Pi沿用现场Python 3.10、已有OpenCV和ONNX Runtime。制作代码更新包：

```bash
python tools/package_orangepi_client.py --code-only --output output/releases/acg720-orangepi-v22-code.zip
```

仓库不包含位流、更新ZIP、模型权重、照片或每台车的实测标定。新成员需要另行取得现场模型、`config/gimbal.json`和`config/visual_task.json`；示例配置不能代替实测值。模型格式/来源及校验信息见`models/*/manifest.json`。厂商IP和第三方文件保留原版权声明，本仓库未额外为其授予许可。

## 当前功能、部署与验证

现场操作见[当前交付与更新步骤](output/releases/README.md)。本机`output/releases/`保存R7位流、Pi v22包和构建资料；GitHub只保留说明与发布JSON，二进制需从源码生成。旧版和临时构建归档保留在本机，不上传仓库。

当前配套为 **FPGA R7测试候选 + Pi v22修订2**。本机R7候选在独立目录编译，原Gowin窗口不会同步那次编译状态；其他构建以各自日志和生成时间为准。构建、哈希与限制见[R7发布记录](output/releases/records/vision_robot_core_r7_2136.json)。

R7于2026-10-25 **21:36:26完成布线并生成位流**。四个主要时钟的报告值达到对应门槛：

| 时钟 | 报告值 / 要求（MHz） |
|---|---:|
| clk50M | 50.034 / 50 |
| GMII接收 | 125.559 / 125 |
| LCD | 50.921 / 45 |
| 网络发送 | 125.000 / 125 |

**整体仍未时序闭合**：报告还有1270个setup、21个hold违例端点，不能只凭上述频率或工具退出码宣称全部通过。27组离线检查通过，属于静态源码、独立数值与协议模型检查；未动态执行HDL或完成实车验收。源文件快照与原始报告在`tmp/core-route-20261025/build-212729*`。

## 本轮功能与更新

- 导航先对准至0.5°，再沿固定直线段行驶，横向修正限幅；最终双轴进入5mm、车身对齐路径至1°内才完成。这些是软件判定值，不是5mm实车绝对精度保证。
- 基础速度30 COUNT/10ms，转向上限12，按角度减速并使用120ms陀螺仪速率预测制动。速度环上电P30/I1/D0；XY导航不依赖轮位置环PID。
- 实时避障保持原目标；路线相近时按车体朝向优先右绕。受阻后只在新合格地图完成积分后重试；STOP、手动接管或重设原点取消旧任务。
- 保留本地OV5640→DDR3→LCD和六轴IMU显示。裁掉未使用的网络DET显示、重复文本缓冲及核心模式下的可选高斯滤波；FF由22137降至19853。Pi图像当前不传入LCD。

请按[直线导航与人脸联调](docs/直线导航与人脸联调_v22.md)操作，再按[Pi v22人脸与小狗测试](docs/香橙派v22人脸与小狗测试.md)更新Pi。**仅烧FPGA不会更新Pi程序或服务**。v22用于人脸/小狗独立跟踪与搜索，TILT初始135°、运行不低于90°；保留现场标定和模型。修订2修复了Shell脚本CRLF导致的`set: Illegal option -`；打包器强制Linux LF，实际服务启动仍需现场确认。

## 保留的架构与边界

唯一工程为`vision_robot_ui.gprj`，顶层`vision_robot_ui_top`，器件GW5AT-LV60PG484AC1/I0。默认ACTUATOR_TEST=0、ROBOT_MODE=1、SEND_CAMERA_UDP=0。任务1的本地摄像头与IMU必须保留。

不擅改CST/SDC、三份process_config及厂商IP/PLL，不用虚假时序例外掩盖违例。用户已授权综合/布局布线；代理未自动烧录或操作电机、舵机、Pi等设备。

VOFA保持COM8、115200/8N1/FireWater及四条速度曲线。Pi为192.168.10.3，FPGA为192.168.10.2，电脑有线为192.168.10.4；单网线直连时Pi使用standalone模式。

后续修改按[协作指南](CONTRIBUTING.md)创建任务分支、运行相关检查并提交Pull Request。旧构建与旧操作只供溯源，不能代替当前R7/v22说明。
