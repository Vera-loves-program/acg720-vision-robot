# 当前交付：只看这个目录

**11月3日深夜跟踪回执/导航状态诊断：** 保留`follow-link-20261103/vision_robot_follow_link_2243_NOT_TIMING_CLOSED.fs`，SHA前缀`f4f16aa6`。修正方向UNKNOWN被误记为CHECK SENSORS，并缩短网口接收包长/IP校验路径；23项实际收包检查、1028项校验器断言及12组雷达RTL回归通过。完整时序仍未闭合，只作停轮诊断候选，不是已验收固件。22:57实验时序变差，已撤回并移入`experiment-2257`，勿按时间选它。[构建、限制与充电后采集步骤](../../docs/跟踪回执与导航状态修订_20261103.md)。Pi本轮不需重新安装v22修订4。

**Pi云台当前更新包：`acg720-orangepi-v22-r4-code.zip`。** 合入队友速度档并修复人脸推理调度/网页LCD选目标冲突；实际版本22、修订4，145项离线测试及2项Shell包检查通过，正常FPGA PWM兼容。本包不包含现场配置和模型，无需重刷系统；[完整更新与验证步骤](../../docs/香橙派v22修订4云台整合_20261103.md)。下面修订2包保留用于历史回溯。

**2026-11-03晚19:38修订**：移动世界建图/低速校正已分开，修正拒行错误覆盖，路径同长时优先左绕。候选为`lidar-moving-20261103/vision_robot_lidar_moving_1938_NOT_TIMING_CLOSED.fs`，SHA前缀`6d5b25ca`。12组实际HDL通过；该轮完整高云构建已完成，但仍有真实时序违例，不能作为已验收固件。详见[本轮修改、开源参考与完整构建结果](../../docs/行驶建图与左绕修订_20261103.md)。15:23旧候选不包含这些后续修改。未自动烧录或覆盖原impl输出。

**2026-11-03雷达验证候选**单独放在`lidar-20261103/`，源码与实际HDL测试通过，但最终未达到全部时序要求，不作为已验收实车固件。先看[本轮结果与验收步骤](../../docs/雷达避障诊断与起点连接_20261103.md)。R7原交付保留，未自动覆盖或烧录。

**Pi包已修订为v22修订2：** 修复Windows CRLF导致的`set: Illegal option -`。已解压早先v22包时可直接执行[说明顶部的换行修复命令](../../docs/香橙派v22人脸与小狗测试.md)，无需重刷、重新上传或重新编译FPGA。

这里的两个交付文件配套使用：

GitHub只保存本说明和发布JSON，下面的二进制文件仅在原开发机存在。新成员从`vision_robot_ui.gprj`编译位流，并运行`python tools/package_orangepi_client.py --code-only --output output/releases/acg720-orangepi-v22-code.zip`生成Pi包。现场模型/标定需另行取得，示例不代替实测。

| 文件 | 用途 |
| --- | --- |
| `vision_robot_core_r7_2136_NOT_TIMING_CLOSED.fs` | 本机FPGA R7测试候选，21:36已完成布局布线 |
| `acg720-orangepi-v22-code.zip` | 由打包器生成的香橙派v22应用程序更新包 |

`records/`中的发布JSON保留构建结果与哈希；源码快照ZIP、证据ZIP、旧位流和旧Pi包仅存原开发机，不提交Git。GitHub上的源码与Git历史用于协作和追溯。

## 为什么Gowin窗口没有显示已完成

21:36的R7是在独立工程目录中编译的，原工程窗口不会同步那次状态。可以在Programmer里直接选择本目录的R7 `.fs`，无需再次编译它。用户21:49已另行启动原工程编译；那一轮的结果以它自己的最新日志和生成时间为准。整理未改动正在编译的源码、约束或`impl`目录。

R7已布通，但完整时序仍有跨时钟和复位违例，因此文件标为`NOT_TIMING_CLOSED`测试候选；未完成实车验收。[详细结果](../../docs/R7编译结果与更新_20261025.md)。

## 香橙派更新：不用重刷系统或SD卡

网线接电脑与Pi，电脑PowerShell执行：

```powershell
scp D:\GOWIN_PROJECTS\acg720_vision_robot\output\releases\acg720-orangepi-v22-code.zip orangepi@192.168.10.3:/home/orangepi/
ssh orangepi@192.168.10.3
```

登录Pi后执行：

```bash
cd /home/orangepi
unzip -o acg720-orangepi-v22-code.zip
cd /home/orangepi/acg720_vision
python3 -c "import json; print(json.load(open('manifest.json'))['client_bundle_version'])"
sh tools/start_fpga_follow.sh --restart
sh tools/start_fpga_follow.sh --status
```

版本应输出`22`，服务应显示`active (running)`。旧服务的2100启动参数会更新为`--initial-tilt-angle 135`。不要再叠加启动旧的nohup命令。

本包只更新应用代码，保留现场`config/gimbal.json`、`config/visual_task.json`、识别模型和已有Python环境。新版包含135°/90°下限、人脸和小狗独立跟踪、默认寻找人脸的修改。完成后把网线接回FPGA，按[人脸与小狗测试步骤](../../docs/香橙派v22人脸与小狗测试.md)操作。
