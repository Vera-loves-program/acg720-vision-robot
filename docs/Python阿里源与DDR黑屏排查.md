# Python 阿里源安装与本次 DDR 黑屏排查

适用于你的 WSL Ubuntu 22.04，以及 `D:\GOWIN_PROJECTS\acg720_vision_robot` 工程。下列 Linux 命令都在出现 `Vera@...$` 的 WSL 终端执行。

2026-10-02 更新：用户确认 R2 修正版摄像头画面恢复，不需要继续执行下方黑屏排查与重新烧录步骤。当前电脑连接、补齐不完整虚拟环境的完整操作见 [WSL 专用连接指南](电脑网线连接与运行指南.md)。下方保留此前诊断记录。

## 1. 你已经有 Python

本机检查得到：WSL 中有 **Python 3.10.12** 和 **pip 22.0.2**。`py -3` 是 Windows Python 启动器的命令；进入 Ubuntu 后应使用 `python3`。Windows 找不到 `py` 只能说明启动器没有安装或没有加入 PATH，不能证明整个电脑没有 Python。

不要按终端建议安装 `pythonpy` 来解决这个问题：它不是 Windows 的 Python 启动器。

先执行：

```bash
cd /mnt/d/GOWIN_PROJECTS/acg720_vision_robot
python3 --version
python3 -m pip --version
```

## 2. 安装项目依赖，使用阿里 PyPI 源

先尝试创建项目专用的虚拟环境。这个环境放在 WSL 自己的用户目录，避免混用 Windows 的 `.venv`：

```bash
cd /mnt/d/GOWIN_PROJECTS/acg720_vision_robot
python3 -m venv ~/.venvs/acg720-vision
~/.venvs/acg720-vision/bin/python -m pip install -i https://mirrors.aliyun.com/pypi/simple/ -r pc/requirements.txt
~/.venvs/acg720-vision/bin/python -c "import cv2, numpy; print('OpenCV:', cv2.__version__, 'NumPy:', numpy.__version__)"
```

如果最后一行能显示两个版本号，项目依赖就准备好了。阿里 PyPI 源提供 Python **包**，不负责安装 Python 解释器；该索引地址来自 [阿里 PyPI 镜像说明](https://developer.aliyun.com/mirror/pypi)。

如果第一步提示缺少 `ensurepip` / `python3-venv`，或者后面提示缺少 `libGL.so.1` 等系统库，先执行下一节，再回到这里重新执行。不要使用 `sudo pip install`。

## 3. 如缺少系统组件，通过阿里 Ubuntu 源补齐

以下命令专门对应你的 **Ubuntu 22.04（jammy）**。它创建临时源文件，只让这两次 apt 命令使用阿里源，不覆盖现有系统源配置。`sudo` 要输入你的 Ubuntu 用户密码；输入时不会显示字符。

```bash
cat > /tmp/acg720-aliyun-jammy.list <<'EOF'
deb https://mirrors.aliyun.com/ubuntu/ jammy main restricted universe multiverse
deb https://mirrors.aliyun.com/ubuntu/ jammy-updates main restricted universe multiverse
deb https://mirrors.aliyun.com/ubuntu/ jammy-security main restricted universe multiverse
deb https://mirrors.aliyun.com/ubuntu/ jammy-backports main restricted universe multiverse
EOF

sudo apt-get -o Dir::Etc::sourcelist=/tmp/acg720-aliyun-jammy.list -o Dir::Etc::sourceparts=- update
sudo apt-get -o Dir::Etc::sourcelist=/tmp/acg720-aliyun-jammy.list -o Dir::Etc::sourceparts=- install python3 python3-venv python3-pip libgl1 libglib2.0-0
```

镜像说明见 [阿里 Ubuntu 镜像](https://developer.aliyun.com/mirror/ubuntu)。如果这些命令联网失败，把实际错误保留下来；开头的 “localhost 代理未镜像到 WSL” 提示与 `py` 命令不存在是两个不同问题。

## 4. 启动电脑接收程序

先按 [电脑网线连接与运行指南](电脑网线连接与运行指南.md) 第 2～4 节核对有线网卡 `192.168.10.3/24`、镜像网络和防火墙。此前的 `172.28.58.92` 是 NAT 环境地址；仅安装 Python 不能让 FPGA 发给 Windows 网卡的 UDP 自动进入 WSL。网络模式说明见 [微软 WSL 网络文档](https://learn.microsoft.com/en-us/windows/wsl/networking)。

网络配置完成、摄像头采集恢复后，先检查收包：

```bash
cd /mnt/d/GOWIN_PROJECTS/acg720_vision_robot
python3 pc/udp_probe.py --bind 0.0.0.0 --seconds 10
```

这个检查器不需要安装任何第三方 Python 包。退出检查器后，再启动图像窗口；两个程序不要同时占用 6102 端口：

```bash
~/.venvs/acg720-vision/bin/python pc/udp_video_viewer.py --bind 0.0.0.0
```

本次 DDR 未就绪时，当前代码还没有开始摄像头采集，电脑可能没有视频包。先准备好电脑环境，再完成下节的 FPGA 排查。

## 5. 本次黑屏目前能确定什么

你报告的是 `CAM READY`、`DDR WAIT`，D3/D7 亮、D4/D6 灭，重新烧录没有恢复。对应当前代码：

| 现象 | 含义与限制 |
| --- | --- |
| CAM READY / D3 亮 | 摄像头寄存器初始化流程结束；不是 DDR 就绪状态，也不能独立证明每次 SCCB 传输都成功 |
| DDR WAIT / D4 灭 | DDR3 IP 的 `init_calib_complete` 没有置高 |
| D6 灭 | 采集逻辑只有在摄像头初始化和 DDR 校准都完成后才解除复位，因此此时不会记录摄像头帧 |
| D7 亮 | LCD 读取摄像头像素时没有得到有效数据，与当前 DDR 未就绪的结果一致 |

所以这次已经定位到 **DDR 初始化/校准尚未完成**，还不能确定具体是 PLL、动态相位控制、复位过程还是布局时序导致。字体修正本身没有改变 DDR 连线；上次摄像头正常也不能保证重新布局后的时钟和时序仍正常。高云对 `init_calib_complete` 的定义见 [DDR3 Memory Interface 用户指南](https://cdn.gowinsemi.com.cn/IPUG281.pdf)。

我读取了你在 2026-10-01 23:55 左右生成的布局/时序报告：有 150 个 setup 和 164 个 hold 违例。跨时钟路径需要分别判断，不能把总数都当作硬件故障；其中摄像头高斯滤波路径报告也存在同一时钟域的负裕量。因此仍有时序问题需要处理，**这次复位调整不能宣称已经解决全部问题**。

## 6. 前一轮诊断与当前烧录入口

前一轮保留字体修正，增加 DDR PLL 锁定状态显示，并让 DDR 动态相位控制寄存器和 mDRP 控制器随 S0 系统复位重新启动；用户反馈仍未恢复。当前 R2 进一步修正 PLL 控制权交接、初始化倍率与高斯/UI/UDP 路径，详见上述修复文档。没有运行 Gowin 编译或 FPGA 仿真，仍需上板确认。

按以下顺序操作：

1. 先保存工作，关闭当前工程，打开 `D:\GOWIN_PROJECTS\acg720_vision_robot\vision_robot_netfix.gprj`。顶层应是 `vision_robot_top`。
2. 重新运行 Synthesize、Place & Route；两步成功后再烧录。R2 使用独立输出名 `vision_robot_netfix`。
3. Programmer 中手动核对选择的是 **这次刚生成的** `D:\GOWIN_PROJECTS\acg720_vision_robot\impl\pnr\vision_robot_netfix.fs`，以及它的修改时间。
4. 烧录后等待几秒。如果仍未出现画面，按住 **S0** 一秒再松开，等待几秒。S4 是滤波切换按钮，不是系统复位。
5. 标题应为 `VERA NET FIX R2`；观察右栏的 DDR 状态，以及 D4/D6/D7。若没有该标题，先核对工程与烧录文件。

新版本的三种 DDR 状态：

| LCD 状态 | 接下来定位什么 |
| --- | --- |
| DDR PLL WAIT | DDR 专用 PLL 尚未锁定，先查 DDR 时钟/PLL 复位 |
| DDR CAL WAIT | PLL 已锁定但 DDR 初始化/校准尚未完成，继续查 DDR 相位控制与校准链路 |
| DDR READY | DDR 校准完成；如果仍然黑，再查摄像头帧到达和 FIFO 数据通路 |

请记录烧录后，以及按 S0 后，分别显示哪一种状态。这个观察能把下一次排查范围进一步缩小。
