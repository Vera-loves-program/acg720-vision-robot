# ACG720 视觉机器人：学习助手与 R5 缩放源码

固定使用 `vision_robot_ui.gprj`，顶层为 `vision_robot_ui_top`。目标器件为 ACG720-60K / `GW5AT-LV60PG484AC1/I0`。

日常操作只看 [香橙派使用说明](docs/香橙派使用说明.md)：包含解压、网络、SSH、收图、拍照、目录清理、PCB 规划、显示方案和 Flash 固化。旧版本入口和重复指南已清理，历史自编文件可在 Git 中查看。

当前两项设计资料：[扩展板规划](docs/扩展板规划.md)含学习助手分工、传感器初始化、完整 P7 分配和电机抗干扰方案；[视觉模型与数据检查](docs/视觉模型与数据检查.md)记录 **YOLOv8n 玩偶狗＋YuNet 人脸检测**。用户已确认狗检测在香橙派运行，正面约 0.97+、其他角度下降，存在对人及杂物的高分误报；这不是统计准确率。最新程序同帧显示 `dog` 和 `human`，YuNet 在本地 225 张原图对照官方实现通过，组合模型尚待香橙派实测。FPGA DET1 反馈仍仅包含狗，LCD 源码未编译/上板；目标关联、学习计时、语音和电机跟随未接入。

当前链路：OV5640 800×480 RGB565 → FPGA/DDR3 → 可切换高斯滤波 → 1024×600 RGB LCD；同时发送 400×240 RGB565 UDP 图像。R5 源码加入以双指位置为锚点的 1～4×细分缩放、双指平移，以及 ZOOM IN MODE 双击选中心和 +/− 按钮。显示取图使用完整 RGB565 源像素；网口仍发送全画面，采集照片不包含 UI 或本地放大裁剪。

**本轮未编译、仿真或上板验证。现有 `impl/pnr/vision_robot_ui.fs` 仍是先前编译的位流；需用户在同一个工程入口重新编译后烧录，才会获得 R5。** 视觉调试中的双击只发布/取消选点请求，不表示已经识别或跟踪；UI 急停状态也不代表电机硬件急停。

| 用途 | 位置 |
| --- | --- |
| Gowin 入口 | `vision_robot_ui.gprj` |
| 顶层/约束 | `src/vision_robot_ui_top.v`、`src/lcd_interaction.cst`、`src/lcd_interaction.sdc` |
| 当前位流 | `impl/pnr/vision_robot_ui.fs`，由用户编译/烧录 |
| SSPI/CPU 复用脚设置 | `impl/vision_robot_ui_process_config.json` |
| 香橙派统一入口 | `client.py`，支持 `view`、`probe`、`capture`、`burst`、`detect` |
| 客户端打包 | `tools/package_orangepi_client.py` |
| 玩偶模型准备 | `tools/prepare_dog_model.py`；本地 `models/dog_plush/`，权重不提交 Git |
| 人脸模型准备 | `tools/prepare_face_model.py`；固定官方 YuNet 校验值与 MIT 许可证 |
| 检测及框回传 | `pc/detect_dog.py`；`src/udp_detection_feedback.v`，UDP 5001 的 DET1 包 |
| 照片 | `dataset/`，不发布到 GitHub |
| 电脑端录像 | `pc/record_video.py`，操作见使用说明第 6.2 节；视频在 `dataset/videos/` |

FPGA 固定 `192.168.10.2`，接收端有线接口 `192.168.10.3/24`，图像 UDP 6102；每行 802 字节，240 行组成完整帧。32 字节 UI 状态包另按类型解析；它不是每帧处理参数确认。不要同时启动多个图像接收程序。

完整本地工程保留 `src/vendor` 和 `src/gowin_pll_45` 的厂商依赖。公开仓库仅同步自编源码、配置和文档，不能把公开仓库当成这些 IP、位流、照片的完整备份。编译请使用保留厂商依赖的本地工程。

维护时先 pull，保存自编源码后同步 GitHub；厂商代码、模型权重和个人照片不发布。保持同一个工程与同名客户端 ZIP，不再新增版本目录。保留 SSPI/CPU 复用脚、DDR3 校准与时钟设置；最新同名 ZIP 内部版本 8，增加 CPU YuNet，人脸显示操作见使用说明第 18 节。本轮未改 FPGA 源码、CST、相机/DDR 或网口格式。
