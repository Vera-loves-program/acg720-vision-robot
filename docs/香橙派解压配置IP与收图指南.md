# 香橙派解压 配置 IP 和接收 FPGA 图像

这份指南按你发来的香橙派截图编写，不需要再对照电脑 WSL 文档。目标是把已经在电脑上跑通的接收程序，放到香橙派上运行。FPGA 保持当前正常显示和发图的工程，不重新烧录。

## 1. 先看清楚在哪里操作

你的香橙派已确认是 Orange Pi 1.2.2 Jammy，基于 Ubuntu 22.04.5。用户名是 `orangepi`，有线网卡是 `eth0`，Wi-Fi 是 `wlan0`，NetworkManager 版本为 1.36.6。截图中 `wlan0` 已连接，`eth0` 为 DOWN，先接网线并启用有线连接，不需要重装系统。

本文只有第 3 节在 **Windows 电脑**操作，其他终端命令都在 **香橙派自己的终端**操作。香橙派不运行 `wsl`，也不使用电脑上的 `/home/Vera/.venvs/...` 或 `/mnt/d/...`。

打开终端：从左上角 Applications 找 Terminal Emulator，或用已有 VS Code 的 Terminal → New Terminal。看到 `orangepi@orangepi5:...$` 就是香橙派终端。命令一条一条执行，只复制命令，不复制提示符。终端粘贴一般用 Ctrl+Shift+V，回车执行。`sudo` 询问密码时输入香橙派账户密码，屏幕不显示字符是正常的，不是没输入。

## 2. 这次网线怎样接 地址怎样分

```text
FPGA RJ45 网口 ── 普通网线 ── 香橙派 RJ45 网口 eth0
                                  │ HDMI
                                显示屏

香橙派 wlan0 ── Wi-Fi ── 上网下载软件
电脑          ── Wi-Fi ── 查资料和复制文件
```

| 设备或接口 | 地址 | 用途 |
| --- | --- | --- |
| FPGA | 192.168.10.2 | 已固定，保持不变 |
| 香橙派 eth0 | 192.168.10.3，掩码 255.255.255.0 | 收到 FPGA 发往 .3 的图像 |
| 电脑有线网卡 | 192.168.10.4，掩码 255.255.255.0 | 避免和香橙派重复；这次可不接网线 |
| 两台设备的 Wi-Fi | 保留现有自动分配地址 | 上网用，不改成 192.168.10.x |

测试有线口的网关、DNS 均留空，FPGA 不是路由器。香橙派继续通过已有 Wi-Fi 上网。有线网段只用于 FPGA 图像和控制数据。

**当前 FPGA 只把图像发往 192.168.10.3:6102。**把电脑改为 .4 后，它不会自动也收到同一视频。先让香橙派成为唯一接收端；电脑和香橙派同时收图需要以后改转发或发送方案。

两个设备只有在同一网络内使用同一 IP 才会冲突。如果电脑已经拔掉测试网线，不必为了直连香橙派马上改电脑；但这里仍给出完整的改法，以免之后接交换机时重复。

## 3. Windows 电脑把有线地址改为 .4

这一节不在 WSL 或香橙派输入命令。

1. 在电脑按 Win+R，输入 `ncpa.cpl`，按回车，打开网络连接窗口。
2. 找到此前用于连接 FPGA 的 **以太网**或 USB 有线网卡。不要改 Wi-Fi，也不要选 `vEthernet (WSL)`。
3. 右键这张有线网卡 → 属性。
4. 单击 Internet 协议版本 4（TCP/IPv4）→ 属性。
5. 选择“使用下面的 IP 地址”，填写：

| 输入项 | 填写 |
| --- | --- |
| IP 地址 | 192.168.10.4 |
| 子网掩码 | 255.255.255.0 |
| 默认网关 | 留空 |
| DNS | 不填写测试网段 DNS；可保持自动获取 |

6. 若“高级”里之前手动加过额外的 192.168.10.3 地址，将这一旧地址移除，只保留本次 .4；其他不属于测试配置的内容不动。
7. 点确定，关闭属性窗口。保持电脑 Wi-Fi 原来的设置。
8. 打开 Windows PowerShell，输入 `ipconfig`，检查对应以太网 IPv4 已不是 .3。网线拔掉时可能只显示“媒体已断开”，可以回属性窗口核对 .4 已保存。

Windows 的 IP 配置方式可参考 [微软网络设置说明](https://support.microsoft.com/zh-CN/Windows/Experience/Connectivity-Networking/essential-network-settings-and-tasks-in-windows)。

以后若重新让电脑单独接收 FPGA，需要先让香橙派断开测试网线，再把电脑有线口改回 .3，并恢复相应的 WSL 接收地址；不能两台都接上后同时使用 .3。

## 4. 先拿新版完整 ZIP

你截图打开的是旧包 `acg720-orangepi-client-20261003.zip`。它带的是早期查看器，没有这次字节顺序自动判断及新版拍照功能。请使用现在提供的：

```text
acg720-orangepi-client-v2-20261003.zip
```

把新 ZIP 从电脑复制到 U 盘，再插到香橙派。打开香橙派文件管理器，在左侧找到 U 盘，把 ZIP 复制到香橙派的 Desktop（桌面）。也可用你刚才已经成功的文件传输方式。

这份新包自带 `acg720_vision_client_v2` 顶层文件夹，与旧目录分开，不需要删除旧文件。它包含以下文件，全部都要解压：

```text
acg720_vision_client_v2/
  pc/
    udp_probe.py
    udp_video_viewer.py
    capture_dataset.py
    video_stream.py
    ui_telemetry.py
    requirements.txt
  香橙派操作指南.md
```

这里的两个辅助模块也必须保留，不能只拷贝 viewer 一个文件。包里没有 Windows/WSL 虚拟环境；香橙派将使用自己的 Python 和图像库。

## 5. 在截图的 Xarchiver 里解压

你现在看到的是压缩包内部目录，**打开 ZIP 不等于已经解压**。

1. 关闭旧 ZIP 的窗口，双击新版 ZIP。
2. 在 Xarchiver 菜单点 Action → Extract（提取/解压），或按 Ctrl+E。不同语言可能显示中文“操作 → 解压”。
3. 在目的目录一栏选择你的 Home（主目录），即 `/home/orangepi`。GUI 路径框填写完整路径，不写 `~`。
4. 选择解压全部文件（All files），保留文件目录结构。不要只解压刚选中的一个文件，也不要把所有文件平铺到主目录。
5. 本包已经包含顶层文件夹；若窗口有“另建包含目录”选项，可以不选，避免外面又多套一层同名目录。
6. 点 Extract，等解压完成。
7. 打开文件管理器的 Home，进入 `acg720_vision_client_v2`，再进入 `pc`，应能看到上节六个文件。

Xarchiver 的 Extract 菜单和 Ctrl+E 快捷键可见 [项目源码](https://github.com/ib/xarchiver/blob/master/src/interface.c)。

如果 GUI 不好找，改用下面的终端方法。新 ZIP 已放桌面时，在香橙派运行：

```bash
python3 -m zipfile -e ~/Desktop/acg720-orangepi-client-v2-20261003.zip ~/
```

这条命令用 Python 自带工具解压到主目录，不用先安装 unzip。若报“文件不存在”，在文件管理器找到 ZIP，查看它的实际位置；中文桌面目录或 Downloads 都可能与上面不同。不要通过 sudo 去解决文件找不到。

解压方式参考 [Python zipfile 命令说明](https://docs.python.org/3/library/zipfile.html#command-line-interface)。不要对已有且被你修改过的同名目录反复解压覆盖；改用新的解压位置。

## 6. 确认目录和 Python

在香橙派终端分别运行：

```bash
cd ~/acg720_vision_client_v2
```

```bash
pwd
```

期望是 `/home/orangepi/acg720_vision_client_v2`。再运行：

```bash
ls pc
```

应看到上面的六个文件。最后查看：

```bash
python3 --version
```

新版程序按 Python 3.10 或以上使用。Ubuntu 22.04 通常提供 3.10；若实际低于 3.10，先发版本输出，不要替换系统 Python。

若 cd 报错但你在文件管理器确实找到项目，进入该文件夹后右键 → Open Terminal Here（在此打开终端）。再运行 pwd，使用真实的项目目录，避免重复套文件夹造成路径错误。

## 7. 用命令设置香橙派 eth0 地址

推荐用这一节，命令已经按你截图中的 eth0 写好。**只在香橙派本机终端执行**，保持 Wi-Fi 连接。先将 FPGA 和香橙派用网线直接相连，两块板正常供电。

先看看网卡和保存的连接配置：

```bash
nmcli device status
```

```bash
nmcli -f NAME,TYPE,DEVICE connection show
```

这里“设备 eth0”和“连接名称”不是一回事。为避免猜原来的 Wired connection 名称，创建一个专用配置 `fpga-camera`。如果上面的列表已经有这个名字，跳过下一条创建命令，直接修改。

第一次创建运行：

```bash
sudo nmcli con add type ethernet ifname eth0 con-name fpga-camera
```

再依次运行，每条成功后再执行下一条：

```bash
sudo nmcli con mod fpga-camera ipv4.addresses 192.168.10.3/24 ipv4.method manual
```

```bash
sudo nmcli con mod fpga-camera ipv4.gateway "" ipv4.dns ""
```

```bash
sudo nmcli con mod fpga-camera ipv4.never-default yes
```

```bash
sudo nmcli con mod fpga-camera connection.autoconnect yes
```

```bash
sudo nmcli con mod fpga-camera connection.autoconnect-priority 100
```

最后激活：

```bash
sudo nmcli con up fpga-camera
```

成功一般显示 `Connection successfully activated`。这些命令保存一个长期有线配置，不是重启就消失的临时地址；不会修改 wlan0。`/24` 对应掩码 255.255.255.0；`never-default yes` 让 FPGA 这条线不抢 Wi-Fi 的默认上网路线。依据：[NetworkManager 配置示例](https://networkmanager.pages.freedesktop.org/NetworkManager/NetworkManager/nmcli-examples.html)和[连接属性说明](https://networkmanager.pages.freedesktop.org/NetworkManager/NetworkManager/nm-settings-nmcli.html)。

检查结果：

```bash
ip -br addr
```

期望 eth0 一行包含 `UP` 和 `192.168.10.3/24`，wlan0 仍是原 Wi-Fi 地址。再运行：

```bash
ip route get 192.168.10.2
```

结果应包含 `dev eth0` 和 `src 192.168.10.3`。它只检查发送路径，不能代替真正收包测试。

若提示没有可用设备或 eth0 仍 DOWN，先确认网线插进了两块板的 RJ45、FPGA 已上电。不要改 wlan0 来代替 eth0。若显示 unmanaged 或 NetworkManager 未运行，把实际输出发来，不要另外修改 netplan 或重装网络组件。

## 8. 如果想用鼠标设置 IP

这一节是第 7 节的替代方法，不必两种都做。桌面菜单名称可能因镜像稍有差别。

1. 在香橙派终端输入 `nm-connection-editor`，或点桌面网络图标 → Edit Connections（编辑连接）。
2. 选 Ethernet/有线连接，点击编辑。若新建连接，类型选 Ethernet，名字可用 `fpga-camera`，设备选择 eth0。
3. 进入 IPv4 Settings，将 Method 改为 Manual（手动）。
4. 点 Add，填写地址 `192.168.10.3`、掩码 `24` 或 `255.255.255.0`、网关留空。DNS 留空。
5. 如果有 Routes → “Use this connection only for resources on its network”，勾选它，让 Wi-Fi 继续负责上网。
6. 保存，在网络菜单选择这条有线连接，重新连接。
7. 回终端执行 `ip -br addr` 和 `ip route get 192.168.10.2`，核对第 7 节预期。

图形界面的固定地址设置参考 [Ubuntu 说明](https://help.ubuntu.com/stable/ubuntu-help/net-fixed-ip-address.html)。若没有编辑器命令，直接使用第 7 节已确认可用的 nmcli，不用先安装图形编辑器。

## 9. 先做 10 秒收包测试

在香橙派终端进入项目后运行：

```bash
cd ~/acg720_vision_client_v2
```

```bash
python3 pc/udp_probe.py --bind 192.168.10.3 --seconds 10
```

等待 10 秒自动结束。这个程序只用 Python 标准库，不要求 OpenCV，不会弹图像窗口。正常应看到：

```text
Packets received: 大于零
Sources: 包含 192.168.10.2
UDP payload lengths: 包含 802 字节
Complete frames: 大于零
PASS: complete video frames reached this computer.
```

其中输出里的 computer 在这里指运行脚本的香橙派。行头顺序可能显示 little，这是当前传输格式的结果，新版查看器会自动判断。

此 FPGA 工程没有实现完整的 ARP/ICMP 应答，所以不要拿 `ping 192.168.10.2` 不通来判断摄像头网络失败。以收到图像包和完整帧为准。

同时只运行一个 probe、查看器或拍照脚本。probe 结束后出现 `orangepi@...$`，再运行下一项。

## 10. 准备显示图像的 Python 环境

先检查系统里是否已经有图像库：

```bash
python3 -c "import cv2, numpy; print(cv2.__version__, numpy.__version__)"
```

若输出两个版本号，不必再安装这两个库。若出现 `No module named cv2` 或 `numpy`，在香橙派保持 Wi-Fi 上网后安装 Ubuntu 的对应包：

```bash
sudo apt update
```

```bash
sudo apt install python3-opencv python3-numpy python3-venv
```

若只缺 venv，也可只安装 `python3-venv`。询问是否继续时输入 `Y` 并回车。apt 使用香橙派现有的软件源，不替换厂商源；暂不复制电脑 x86 Ubuntu 的源到 ARM 板上。后续需要 pip 安装的 Python 包继续使用你要求的阿里源。

为本工程建立独立环境，并允许它使用刚安装的系统 OpenCV：

```bash
mkdir -p ~/.venvs
```

```bash
python3 -m venv --system-site-packages ~/.venvs/acg720-vision
```

```bash
~/.venvs/acg720-vision/bin/python -c "import cv2, numpy; print('READY', cv2.__version__, numpy.__version__)"
```

期望输出 `READY` 和版本号。已有同名环境却仍提示缺库时，先发输出，不要删除过去的环境。如果是纯显示接收任务，不必再 pip install requirements.txt 升级 OpenCV，也不必安装 YOLO 或 RKNN。

虚拟环境和 `--system-site-packages` 的作用参考 [Python venv 文档](https://docs.python.org/3/library/venv.html)。Linux 使用 `bin/python`；电脑的 Windows Scripts 路径和 WSL 虚拟环境不能直接复制到香橙派。

以后确实需要用 pip 安装某个新包时，使用这个环境并指定阿里 PyPI，例如：

```bash
~/.venvs/acg720-vision/bin/python -m pip install -i https://mirrors.aliyun.com/pypi/simple/ 包名
```

这只是格式示例，“包名”不是本次要安装的内容，不要原样执行。若 apt 下载失败，发失败信息和现有源配置，再确定 ARM 对应镜像，不强行改系统源。

## 11. 在香橙派显示屏上看摄像头

确认 probe 已退出，在香橙派的桌面终端依次执行：

```bash
cd ~/acg720_vision_client_v2
```

```bash
~/.venvs/acg720-vision/bin/python pc/udp_video_viewer.py --bind 192.168.10.3
```

显示屏上应弹出 `VERA Vision Lab - FPGA UDP Viewer`。刚启动可能暂时 WAITING，收到完整帧后出现实时画面；终端会打印 `FORMAT: row=little, pixels=little` 或正确检测出的 big 格式。

网络图像是 400×240，预览放大显示，细节少于 LCD 的 800×480 是当前 FPGA 降采样的结果。现在没有运行 AI 检测，双击选点只是界面预备功能，不表示已经识别目标。

退出：单击视频窗口，切到英文输入，按 Q 或 Esc；也可回运行程序的终端按 Ctrl+C。若窗口打不开，请从连接显示屏的香橙派本机桌面运行，先不通过无图形转发的 SSH 执行。

## 12. 可选测试控制和拍照

**控制：**有画面后单击窗口，按 D 请求切换调试模式，观察 FPGA LCD 是否变化；按 F 请求切换滤波，观察 LCD 实际状态。这些是未确认成功的反向控制测试，没有应答机制，不能仅凭电脑界面按钮改变就判定 FPGA 已执行。E 目前只是停止状态请求，不是已接入的电机急停。

**拍照：**先按 Q 退出查看器，再运行：

```bash
~/.venvs/acg720-vision/bin/python pc/capture_dataset.py --bind 192.168.10.3 --fps 3
```

等有实时画面、complete 增加，单击画面并切到英文输入。按 1 拍人物、2 拍玩偶、3 拍耳机线，再按空格开始一次 5 秒连拍，最多约 15 张。结束后再切组，按 Q 退出。

本次照片在香橙派主目录下：

```text
/home/orangepi/acg720_vision_client_v2/dataset/captures
```

在香橙派文件管理器进入这个目录，找到刚才的日期时间文件夹，再看 vera、dog_plush 或 earphone_cable 里的 PNG。它们不在电脑 D 盘。将照片复制到 U 盘即可传给队友标注。

## 13. 发生问题时看这一页

| 现象 | 先做什么 |
| --- | --- |
| 解压后找不到项目 | 在文件管理器找到真正的 pc 上级目录，在此打开终端，运行 pwd 和 ls pc |
| No module named video_stream 或 ui_telemetry | 没有解压新版完整包，或只复制了主脚本；保留全部六个文件 |
| eth0 DOWN 或激活失败 | 查网线、两板供电和 nmcli device status；不要改 Wi-Fi 代替有线 |
| Cannot assign requested address | 检查 eth0 是否有 192.168.10.3/24，再启动接收器 |
| Address already in use | 退出其他占用 UDP 6102 的查看器、收包器和采集脚本 |
| probe 收包为 0 | 确认 FPGA 接的是香橙派、LCD 正常、eth0 的地址正确；继续查防火墙 |
| 包多而完整帧为 0 | 发包长度、bad、完整/残帧和行头顺序统计；不能用 ping 或不断重装环境解决 |
| 有帧但画面颜色不正常 | 默认使用新版自动顺序；若仍异常，保留行头自动，仅用 --byte-order little 或 big 比较已知颜色 |
| Wi-Fi 上网受影响 | 查 ip route，默认路线应走 wlan0；确认 fpga-camera 的 never-default 是 yes 且网关为空 |
| apt 报网络错误 | 先确认香橙派 Wi-Fi 能上网，再发完整错误；不把 FPGA 当网关 |

如果 0 收包，再看系统是否启用了 UFW：

```bash
sudo ufw status
```

若命令不存在或状态 inactive，说明不是启用的 UFW 在拦截。若 active，再只放行来自 FPGA 的图像包：

```bash
sudo ufw allow from 192.168.10.2 to any port 6102 proto udp
```

不要直接关闭整个防火墙。如果仍不通，发以下实际输出：ip -br addr、nmcli device status、ip route get 192.168.10.2，以及 10 秒 probe 的全部结果。

## 14. 以后香橙派有线口改接路由器时

`fpga-camera` 为直连 FPGA 配置，不是普通上网配置。切回路由器时，先在网络编辑器选普通有线配置，把 IPv4 方法恢复 Automatic（DHCP），再激活它。

若旧自动有线连接存在，可以在网络菜单选择它。为免 FPGA 配置自动抢回，先在本机终端运行：

```bash
sudo nmcli con mod fpga-camera connection.autoconnect no
```

日后再接 FPGA，可重新执行 `sudo nmcli con up fpga-camera`。普通上网时不要把 .3 静态地址或空网关方案照搬到路由器网络。

## 15. 完成后怎样判断成功

这一步移植成功应同时满足：eth0 有 .3 地址；Wi-Fi 仍能上网；10 秒 probe 显示 PASS；香橙派窗口持续显示 FPGA 画面。反向控制、拍照及 AI 模型分开验证，不把看见画面等同于全部功能完成。

本机只验证打包内容和主机脚本，未远程修改香橙派网络，也未重新编译 FPGA。实际的香橙派依赖安装、收包和显示结果以你按步骤操作后的输出为准。
