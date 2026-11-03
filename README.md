# ACG720 FPGA视觉机器人

当前协作分支：**`fix/teammate-obstacle-resume-20261103`**。FPGA以用户提供、已报告能起步/到点/遇障停车的队友源码包为基线；Pi保留现有 **v22修订4**。旧严格观测门限分支没有合入。

本轮修复题4GO TO GOAL停车后可能丢失地图完成事件、无法重试的问题，并将等长路线改为优先左绕。功能RTL仅改任务控制器和规划器。新增55项HDL检查与原有三组动态回归通过，原控制器在同一测试中可复现丢事件；尚未实车验证。

Gowin于2026年11月3日23:57完成布线并生成候选，**完整时序仍未闭合：732个setup、386个hold违例端点**。构建与烧录分开，代理没有自动下载或操作底盘。修复依据、资源/时序对比、测试限制及验收步骤见 [队友基线续走协作](docs/队友基线续走协作_20261103.md)。

## 工程与交付

- 本机工程：`D:\GOWIN_PROJECTS\acg720_obstacle_resume\vision_robot_ui.gprj`。
- 旧工程 `D:\GOWIN_PROJECTS\acg720_vision_robot` 保留，勿混用其位流。
- [本次候选文件与部署说明](output/releases/README.md)。Git只提交源码/记录，FS与ZIP在原开发机生成。
- [基线247文件SHA](docs/validation/teammate-field-baseline-20261103.json) 与 [本次验证记录](docs/validation/teammate-resume-20261103.json)。

FPGA使用Gowin FPGA Designer V1.9.11.03 Education，器件GW5AT-LV60PG484AC1/I0，顶层 `vision_robot_ui_top`。保留CST/SDC、厂商IP/PLL、三份process_config、本地摄像头和IMU。不要用虚假时序例外掩盖违例。

```bash
git clone --branch fix/teammate-obstacle-resume-20261103 https://github.com/Vera-loves-program/acg720-vision-robot.git
cd acg720-vision-robot
git switch -c feature/your-task
```

修改前阅读 [协作指南](CONTRIBUTING.md)。本分支导入基线提交 `969f061`；后续行为修复独立提交，便于比较与回退。队友从最新分支另开自己的分支，共同修改文件前协调，PR经另一成员审阅后再合并，不强推main。

## 功能与边界

| 任务 | 实现 |
|---|---|
| 1 本地摄像头 | OV5640→DDR3→LCD，保留缩放链与六轴IMU |
| 2 底盘控制 | 前后与定点旋转，速度环上电P30/I1/D0 |
| 3 雷达建图 | C1解析、安装外参/车体过滤、栅格地图、低速GO 1M |
| 4 定点导航 | 一键原点/+X，规划、直线路段、动态障碍重试、等长优先左绕 |
| 5 云台跟踪 | Pi人脸/小狗识别与独立云台跟踪 |
| 6 目标任务 | 默认寻找人脸，云台搜索、车体辅助转向、视觉方位/雷达匹配与接近 |

最终导航完成仍要求估计X/Y各5mm，不是实车5mm精度保证。题4基础速度30 counts/10ms，保持真实近障与传感器失效停车、人工STOP取消；不把UNKNOWN强行置FREE。

Pi图像当前不传入LCD，浏览器预览由Pi提供。本轮不修改Pi功能/协议，已部署v22修订4的机器无需重装或重启。Pi模型、标定和环境保留，不用示例配置覆盖实测值。确需另行制作更新包：

```bash
python tools/package_orangepi_client.py --code-only --output output/releases/acg720-orangepi-v22-code.zip
```

仓库不包含位流、更新ZIP、模型权重、照片或每台车标定；新成员另行取得模型及 `config/gimbal.json`、`config/visual_task.json`。保留第三方版权，不额外授予厂商IP许可。

[文档索引](docs/README.md) 中的历史R7/v22修订2数据仅作溯源，当前版本以本页及本次验证记录为准。
