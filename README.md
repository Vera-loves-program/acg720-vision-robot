# ACG720 Vision Robot · FPGA 摄像头、LCD UI 与 UDP 视频

这是一个从已上板正常显示的 `lcd_camera_switch` 独立出来的新工程。原来的静态图片、图片 ROM 和构建产物没有进入本工程；原工程保留作已验证的回退版本。目标器件为 ACG720-60K / `GW5AT-LV60PG484AC1/I0`，OV5640 的实际采集尺寸为 **800×480 RGB565**，LCD 扫描尺寸为 **1024×600**。

> 本版本已写好 HDL 与电脑接收程序，**尚未综合、布局布线、烧录或做实板收包验证**。因此“已写入工程”不等于“网口已联通”。用户此前希望自己编译，本次没有替用户运行 Gowin。首次上板请先看 LED、LCD 和 UDP 包，再调整 PHY/时序。UI 主色目前是临时的深蓝＋青绿方案；主页仓库暂时无法读取，得到主页配色后可再统一。

## 现在的功能

- 摄像头常驻显示：原尺寸 800×480 放在 1024×600 LCD 左侧，右侧由 FPGA 实时绘制状态和控制提示，没有图片 ROM。
- [UI 示意图](<docs/ui_preview.svg>)展示排版与临时配色；其中中间的抽象画面只是占位示意，不会烧入 FPGA。
- 真正的 FPGA **3×3 RGB565 高斯滤波**：默认开启，作用于摄像头画面写入 DDR3 之前；LCD 与网络流都使用滤波后的像素。S4 短按（20 ms 消抖）切换滤波启停，状态在下一次摄像头帧起点生效。核为因果 3×3 窗口，画面效果相对原图左上偏约 1 像素；边界两行/两列透传。
- 网络降采样：在 FPGA 内每隔一行/一列取样，将 800×480 转为 400×240，保留 RGB565 彩色；每个 UDP 包承载一行，低于标准以太网 MTU。
- FPGA → 电脑：OV5640 → 滤波 → 行取样 → 异步 FIFO → UDP/RGMII → RJ45。电脑端 `pc/udp_video_viewer.py` 拼帧和显示。
- 电脑 → FPGA：向 `192.168.10.255:5001` 发送 8 字节 UDP 命令，可远程设置滤波、调试十字线和停止状态锁存。命令在以太网 CRC 和应用层 XOR 校验通过后才执行。

`STOP LATCH` 目前**只是 FPGA/界面的状态位**，因为项目尚未连接电机驱动器和硬件急停回路，不能把它当成实际的紧急制动。未来驱动电机时，需要独立的物理急停按钮直接切断驱动使能，同时让 FPGA 状态机也响应急停；网络按钮可作为补充控制，不能承担唯一安全功能。

## 地址与图像格式

| 项目 | 值 |
| --- | --- |
| FPGA IPv4 | `192.168.10.2/24` |
| 电脑有线网口 IPv4 | `192.168.10.3/24` |
| FPGA → 电脑视频 | UDP，目的 `192.168.10.3:6102`，广播目的 MAC |
| 电脑 → FPGA 命令 | UDP，目的广播 `192.168.10.255:5001` |
| 视频包 | 恰好 802 字节：2 字节**大端**行号 `0..239`，随后 400 个**大端** RGB565 像素 |
| 帧边界 | 行号 0 开始新帧；接收端只显示 240 行均到齐的帧 |

广播控制包免去了当前版本尚未实现的 ARP 应答；请在隔离的测试网段使用。UDP 本身可能丢包，所以接收程序会丢弃残帧。此协议只适合当前的 400×240/单机链路，未来如改尺寸或跨网段，需要增加帧号、协议版本和路由/ARP 设计。

命令包是 `ASCII "VRB1" | opcode | value | sequence | XOR(前 7 字节)`，共 8 字节。`opcode=1` 设置滤波（`value=0/1`），`2` 设置调试叠加，`3` 锁存停止状态，`4` 清除停止状态。电脑接收程序的快捷键为 **F、D、E、R、Q**。若用 S4 改过滤波，电脑端的本地 F 状态并不知道按键结果；以 LCD 上的 `GAUSS ON/OFF` 为准。

## 上板和电脑连接

1. 用网线连接 ACG720 的 RJ45 和电脑有线网口；两端也可以通过普通千兆交换机连接。保持两板正常供电，不通过 LCD 排线给网口供电。
2. 把电脑**有线网卡**配置为 `192.168.10.3`、子网掩码 `255.255.255.0`。不要把这个地址配到 Wi-Fi 上。测试时关闭阻止 UDP 6102 入站的防火墙规则，或只为 Python 程序/该端口放行。
3. 在 Windows Gowin FPGA Designer 中打开 [`vision_robot.gprj`](<vision_robot.gprj>)，顶层是 `vision_robot_top`。SSPI、CPU 复用脚作为普通 IO 的配置复制自原工程的 `impl/vision_robot_process_config.json`；若 GUI 未读取配置，应在 Place & Route 的 Dual-Purpose Pin 页面确认相应选项。由你执行综合、布局布线和 SRAM 烧录。
4. 电脑上使用 Python 虚拟环境安装依赖：`python -m pip install -r pc/requirements.txt`。Windows 可直接运行 `python pc/udp_video_viewer.py --bind 192.168.10.3`。窗口应显示视频与完整帧/丢帧计数。收包时如出现红蓝/字节顺序异常，可试 `--byte-order little` 并回查 FIFO 输出顺序。
5. 若一定从 **WSL2** 收包，请先确保 WSL 能从该有线网口收到局域网入站 UDP。Windows 11 可参考 [微软 WSL 网络文档](https://learn.microsoft.com/en-us/windows/wsl/networking) 将 `%USERPROFILE%\.wslconfig` 配置为 `[wsl2]` 下 `networkingMode=mirrored`，随后运行 `wsl --shutdown` 再进入 WSL。还需检查 Windows/Hyper-V 防火墙。WSL 内用 `python3 -m pip install -r pc/requirements.txt`、`python3 pc/udp_video_viewer.py --bind 0.0.0.0`。若 WSL 图形窗口无法出现，可先在 Windows 原生 Python 中运行同一脚本；FPGA 协议无需更改。

该脚本需要有图形桌面（Windows 或 WSLg）。它并不模拟 RK3588 NPU，只负责当前的网络视频/控制链路。

## LED 与 LCD

| LED | 含义 |
| --- | --- |
| D0 | 50 MHz 板载时钟每 0.5 秒翻转 |
| D1 | 上电等待完成 |
| D2 | LCD 帧扫描心跳 |
| D3 | OV5640 SCCB 初始化完成 |
| D4 | DDR3 校准完成 |
| D5 | 以太网 PLL 与 PHY 初始化完成，**不代表网线已经协商成功** |
| D6 | 收到摄像头场同步 |
| D7 | LCD 相机 FIFO 欠读 |

LCD 右栏的 `PHY INIT` 与 D5 含义相同；没有读取 PHY Link Status，因此不能据此判断网线是否真的连通。`GAUSS ON/OFF` 来自滤波器当前帧的真实状态。`2X SAMPLE` 和 `RGB565` 描述网络视频格式。调试模式仅在画面中心绘制十字线；以后可增加边缘图、红色候选框、丢包/延迟/测距等信息。停止栏现为协议状态预览。

## 工程结构与来源

- `src/vision_robot_top.v`：摄像头/DDR/LCD/以太网顶层集成。
- `src/gaussian3x3_rgb565.v`：自行编写的实时 3×3 高斯流水线。
- `src/video_400x240_packetizer.v`：降采样与逐行包格式。
- `src/lcd1024_vision_ui.v`：RGB565 面板与 5×7 英文字形绘制。
- `src/udp_command_parser.v`：反向 UDP 控制命令解析。
- `src/vendor/`：高云教材第 45、59 章例程与生成的 Gowin IP；PHY、FIFO、DDR、摄像头初始化沿用相应教学代码。`eth_udp_rx_gmii.v` 增加了本地 `/24` 广播地址的接收条件。这些第三方代码请保留原始声明；公开发布前应确认其再分发许可，建议 GitHub 仓库先设为**私有**。
- `pc/udp_video_viewer.py`：Windows/WSL 通用的 Python/OpenCV UDP 接收与控制程序。

原工程使用图片 ROM 时的布线报告为 96/118 BSRAM，其中静态图片占约 61 块。本工程去掉了图片 ROM，为网络 FIFO 和图像行缓存留出资源；**实际资源及时序仍须以新工程布线报告为准**。

## 版本管理

该目录就是准备上传的独立 Git 仓库，不包含 `impl` 的构建/布线/位流产物，避免仓库膨胀。上传并配置 `origin` 后，Windows PowerShell 可运行 `./tools/sync_github.ps1 -Message "描述这次修改"`；脚本先 fetch/pull --rebase，再提交和推送。若拉取出现冲突，脚本停止并保留现场，不会强制覆盖。GitHub 不代替本地构建缓存；若要保留某次可烧录位流，可作为有版本号的发布附件，而不要把每次构建产物都提交进 Git。
