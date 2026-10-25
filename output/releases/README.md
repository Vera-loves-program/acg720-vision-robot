# 当前交付：只看这个目录

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
