# ACG720 视觉机器人：当前 R4 工程

固定使用 `vision_robot_ui.gprj`，顶层为 `vision_robot_ui_top`。目标器件为 ACG720-60K / `GW5AT-LV60PG484AC1/I0`。

[公开客户端说明](docs/香橙派使用说明.md) 概述当前 R4。包含实际部署信息的完整逐步指南保留在使用者电脑的 deliverables 中。旧版本入口和重复指南已清理，历史自编文件可在 Git 中查看。

当前两项设计资料：[扩展板规划](docs/扩展板规划.md)含传感器初始化、完整 P7 分配和电机抗干扰方案；[视觉模型与数据检查](docs/视觉模型与数据检查.md)区分队友提供的标注/检测结果，并安排实时人脸检测和小狗训练。两项尚未加入 R4 固件或客户端。

当前链路：OV5640 800×480 RGB565 → FPGA/DDR3 → 可切换高斯滤波 → 1024×600 RGB LCD；同时发送 400×240 RGB565 UDP 图像。R4 LCD 支持本地 1×/2×显示，网口图像保持全画面。GT911 提供触点输入，视觉识别和电机跟随尚未实现，UI 急停状态也不代表已经有电机硬件急停。

| 用途 | 位置 |
| --- | --- |
| Gowin 入口 | `vision_robot_ui.gprj` |
| 顶层/约束 | `src/vision_robot_ui_top.v`、`src/lcd_interaction.cst`、`src/lcd_interaction.sdc` |
| 当前位流 | `impl/pnr/vision_robot_ui.fs`，由用户编译/烧录 |
| SSPI/CPU 复用脚设置 | `impl/vision_robot_ui_process_config.json` |
| 香橙派统一入口 | `client.py`，支持 `view`、`probe`、`capture`、`burst` |
| 客户端打包 | `tools/package_orangepi_client.py` |
| 照片 | `dataset/`，不发布到 GitHub |

视频按行传输，每行 802 字节，240 行组成完整帧。32 字节 UI 状态包另按类型解析；它不是每帧处理参数确认。不要同时启动多个图像接收程序。实际网络部署配置见本地指南。

完整本地工程保留 `src/vendor` 和 `src/gowin_pll_45` 的厂商依赖。公开仓库仅同步自编源码、配置和文档，不能把公开仓库当成这些 IP、位流、照片的完整备份。编译请使用保留厂商依赖的本地工程。

维护时先 pull，保存自编源码后同步 GitHub；厂商代码和个人照片不发布。`tools/sync_github.ps1` 维护自编文件快照，当前只有一个 R4 项目入口。编译设置释放板卡的 SSPI/CPU 复用脚，并保持已验证的 DDR3/摄像头/网口配置。

