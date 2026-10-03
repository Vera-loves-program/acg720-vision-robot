# 香橙派收图排查、FPGA 掉电保存和电脑远程编辑

更新：2026-10-04。针对你现在的 **FPGA R3 + 香橙派 V2 客户端**。R3 是 FPGA 程序版本，V2 是 Python 接收包版本，名字不同不表示不能配合。本次不要求烧录 R4，也不改摄像头、DDR 或触摸代码。

## 1. 先弄清楚现在的问题

你截图中，10 秒收到约 75000 个包，其中视频包长度为 802 字节，100 个 UI 状态包也解析正常。这已经说明网线、接收 IP 和 UDP 路径能工作。

问题是：四万多个视频包的开头不能解析成有效行号，剩下的也没有组成完整帧。开头的 `61 00` 可以被解释为行号 97，但也可能只是 RGB565 像素。**落在 0～239 内的数，不一定真是行号。**因此旧脚本的 `Valid video rows` 不能单独作为成功依据。

`Complete frames: 0; incomplete frames: 0` 也不代表“没有丢帧”。旧脚本遇到行号 0 才开始组帧；一直没遇到它时，这两个数都可能是 0。我已加入行号 0、行号分布、相邻行变化和原始包样本的诊断。

我对比了 R2/R3 的发送连接，R3 增加了视频与 UI 状态包的仲裁。目前截图更像视频行头或 FIFO 包边界异常，但还不能证明具体哪一行 RTL 有错。先在同一香橙派上对比之前能收图的 R2，避免盲目修改。

**下面的收包诊断只用 Python 自带库，不需要 OpenCV、pip 或虚拟环境。环境安装失败不会阻止这一步。**

## 2. 每种窗口在哪里操作

| 窗口 | 如何认出它 | 本文用途 |
| --- | --- | --- |
| 香橙派终端 | `orangepi@orangepi5:...$` | 收包、安装软件、启用 SSH |
| 电脑 Windows PowerShell | `PS ...>` | SSH、复制文件；不需要 Windows Python |
| 电脑高云 Programmer | 有器件和 Operation 表格 | 选择 `.fs`，下载 FPGA |
| VS Code 的 SSH 终端 | 状态栏显示 SSH，终端用户名是 orangepi | 等同香橙派终端 |

不要复制 `$` 或 `PS ...>` 提示符。香橙派终端粘贴通常用 Ctrl+Shift+V。`sudo` 输入密码时不会显示字符，输入后回车即可。

## 3. 连接和地址保持这样

```text
FPGA 网口 ───── 网线 ───── 香橙派 eth0
192.168.10.2               192.168.10.3/24

电脑 Wi-Fi ── 同一个可互通的无线网络 ── 香橙派 wlan0
             用于 SSH、复制文件和上网
```

香橙派有线口不设网关和 DNS。Wi-Fi 保留自动分配地址；不改成 192.168.10.3。电脑若接入同一个测试有线网段，使用 192.168.10.4，不能与香橙派重复。

在香橙派运行：

```bash
ip -4 -br addr
```

确认 `eth0` 有 `192.168.10.3/24`。`wlan0` 的地址另外记下来，稍后给电脑 SSH 用。你之前的 `10.8.180.11` 只是当时地址，现在可能变了。

你目前已经收到大量包，不需要再次修改 IP，也不需要先让 FPGA 能被 ping 通。当前 FPGA 没有完整的 ping/ARP 应答功能。

## 4. 拿到这次的小工具包

电脑文件位置：

```text
D:\GOWIN_PROJECTS\deliverables\acg720-support-20261004.zip
```

包内只有诊断脚本、APT 软件源工具和本文，没有 FPGA 位流或新 UI。

如果尚未配置 SSH，这次先把小包复制到香橙派桌面。之后就可以用第 8 节的方法传文件，不必继续用 U 盘。

在香橙派终端输入：

```bash
python3 -m zipfile -e ~/Desktop/acg720-support-20261004.zip ~/
ls ~/acg720_support_20261004
```

应看到 `pc`、`tools` 和 Markdown 指南。如果 ZIP 不在 Desktop，把命令中的路径改成文件管理器里实际的位置。解压无需 sudo。

先保留旧诊断脚本，再更新它：

```bash
cd ~/acg720_vision_client_v2
cp -n pc/udp_probe.py pc/udp_probe.before-20261004.py
cp ~/acg720_support_20261004/pc/udp_probe.py pc/udp_probe.py
```

其他 Python 文件保持原样。V2 包已有它需要的 `pc/ui_telemetry.py`。

## 5. 先做 R3 与 R2 的收图对照

### 5.1 测当前 R3

关闭其他 viewer、capture 和 probe。每次只运行一个接收程序，避免争用 6102 端口。

在香橙派运行：

```bash
cd ~/acg720_vision_client_v2
python3 pc/udp_probe.py --bind 192.168.10.3 --seconds 10 --report ~/fpga-r3.probe.json
```

10 秒后自动结束。记录这些输出：

- `Complete frames`：是否大于 0。
- `Row 0 headers`：是否一直为 0。
- `distinct row numbers`：看到了多少种行号，最多 240。
- `Most common row numbers`：是否长期集中在几个像素值似的数上。
- `Most common invalid 2-byte headers`：无效开头的分布。

报告保存到主目录 `fpga-r3.probe.json`，其中包含 16 个视频包的像素样本。可以单独发我排查，不要提交到公开 GitHub。

### 5.2 只换 FPGA，香橙派设置保持相同

在电脑 Programmer 中，仍选 **SRAM Program**，把文件改为之前能正常收图的 R2：

```text
D:\GOWIN_PROJECTS\acg720_vision_robot\impl\pnr\vision_robot_netfix.fs
```

下载完成，等摄像头和网口初始化，然后在香橙派运行：

```bash
cd ~/acg720_vision_client_v2
python3 pc/udp_probe.py --bind 192.168.10.3 --seconds 10 --report ~/fpga-r2.probe.json
```

R2 没有 R3 的 UI 状态包，因此 `R3 UI observations: 0` 正常。

| 对照结果 | 下一步 |
| --- | --- |
| R2 完整帧大于 0，R3 一直为 0 | 问题与 R3 集成相关；把两份报告发我，继续查 FPGA 发送逻辑 |
| 两者都有完整帧 | 检查之前实际下载的文件、复位后的状态，以及接收程序是否同时打开 |
| 两者都没完整帧 | 把两份报告发我；再用同一 R2 在原电脑接收端对照，不能仅凭目前截图认定香橙派或 FPGA 故障 |

这个对照无需重新编译。**不要通过忽略行号、拿不同帧的行拼图来假装收图成功。**

如果发现 `.fs` 不存在，先停在这里，告诉我实际文件列表，不随便选一个同名文件。`vision_robot_ui.fs` 每次构建都会被覆盖，文件名相同不能保证内容仍是 R3；LCD 标题可用来确认实际下载的版本。

## 6. 修复香橙派软件安装

### 6.1 为什么现在下载失败

截图中的核心错误是 `Hash Sum mismatch`，收到的文件只有 218 字节，与软件源记录的安装包不符。可能是镜像站、缓存、校园网登录页面或代理返回了错误内容；截图还不能区分这些原因。最后一句 `held broken packages` 不是已经查明软件被锁定的证据。

先在香橙派浏览器打开一个网页，确认能正常上网；校园网需要登录时先完成认证。不要关闭 APT 校验来绕过错误。

### 6.2 使用阿里云的 ARM 软件源

你的香橙派是 Ubuntu 22.04 ARM64，软件源应使用 **ubuntu-ports**。阿里云的 [Ubuntu Ports 配置说明](https://developer.aliyun.com/mirror/ubuntu-ports/) 提供了 Jammy 的 HTTPS 地址。

在香橙派依次运行：

```bash
cat /etc/os-release
dpkg --print-architecture
python3 ~/acg720_support_20261004/tools/fix_orangepi_apt.py
```

最后一条只是预览，不改系统。正常时会列出准备修改的源配置文件，并显示目标 `https://mirrors.aliyun.com/ubuntu-ports/`。

确认是 Ubuntu 22.04、`arm64` 后运行：

```bash
sudo python3 ~/acg720_support_20261004/tools/fix_orangepi_apt.py --apply
```

工具先备份原文件到 `/var/backups/acg720-apt-日期时间/`，再替换 Ubuntu Ports 地址。发行版名、组件、签名设置和香橙派厂商源都保留。请记下它打印的备份路径。

接着逐条运行：

```bash
sudo apt clean
sudo apt update
```

**等 `apt update` 成功再安装。**如果仍有下载失败，先不要反复安装或执行系统升级，把从 `Err:` 开始的错误发我。

如果工具显示没有匹配项，可能已使用阿里源，也可能实际源不是 ubuntu-ports。不要删除其他源。可查看：

```bash
grep -R -n -E '^[[:space:]]*(deb |deb-src |URIs:)' /etc/apt/sources.list /etc/apt/sources.list.d
```

若源地址包含账号密码，不要公开发送原样输出。

### 6.3 仍然 Hash mismatch 时

在香橙派运行下面这段，检查 HTTPS 返回的是否是签名索引：

```bash
python3 - <<'PY'
from urllib.request import urlopen
url = 'https://mirrors.aliyun.com/ubuntu-ports/dists/jammy/InRelease'
with urlopen(url, timeout=20) as response:
    print('Final URL:', response.url)
    print(response.read(100).decode('utf-8', errors='replace'))
PY
```

正常开头应包含 `-----BEGIN PGP SIGNED MESSAGE-----`。若是 HTML、登录提示、跳转页面或连接失败，先处理上网问题；可以暂时用手机热点验证。这个检查只确认该 URL 的内容，不能替代 APT 对全部索引和软件包的校验。

若 HTTPS 正常、但 `apt update` 仍报错，只清理已有下载索引再试一次：

```bash
sudo find /var/lib/apt/lists -maxdepth 1 -type f ! -name lock -delete
sudo apt update
```

这里清理的是可重新下载的 APT 索引，不删除源配置。不要使用 `--allow-unauthenticated`。

如果需要还原源，把下面路径换成工具打印的实际备份目录：

```bash
sudo cp -a /var/backups/acg720-apt-实际日期时间/. /etc/apt/
sudo apt update
```

### 6.4 安装接收图像所需的软件

软件源更新成功后，在香橙派运行：

```bash
sudo apt install python3-opencv python3-numpy python3-venv openssh-server git
```

询问是否继续时输入 `y`，回车。完成后检查：

```bash
python3 -c "import cv2, numpy; print('OpenCV', cv2.__version__); print('NumPy', numpy.__version__)"
```

看到版本号即可先运行，不必再通过 pip 重装 OpenCV：

```bash
cd ~/acg720_vision_client_v2
python3 pc/udp_video_viewer.py --bind 192.168.10.3
```

这条查看器命令先在**香橙派连接显示屏的本地桌面终端**运行。没有完整帧时，它仍会等待；安装图像库本身不会修复坏行头。

以后需要独立虚拟环境时，只创建一次：

```bash
mkdir -p ~/.venvs
python3 -m venv --system-site-packages ~/.venvs/acg720-vision
~/.venvs/acg720-vision/bin/python -c "import cv2, numpy; print('Dependencies OK')"
```

它沿用 apt 安装的图像库。若以后增加其他 pip 依赖，用这个环境并使用阿里源：

```bash
~/.venvs/acg720-vision/bin/python -m pip install -i https://mirrors.aliyun.com/pypi/simple/ 包名
```

`包名` 是将来需要的具体库，不是现在要原样执行的命令。运行脚本时可以始终指定 `~/.venvs/acg720-vision/bin/python`，不必反复创建环境。

## 7. FPGA 程序怎样掉电保存

### 7.1 SRAM 和 Flash 的区别

你之前选择的 **SRAM Program** 适合调试，但断电后 FPGA 会丢失这次下载的配置。要开机自动运行，需要将经过验证的 `.fs` 写到板上的外部 Flash。

普通用户逻辑复位按键只复位代码，不应擦除配置；重新配置按键或 Programmer 的 Reprogram 则会重新装载 Flash 中的旧程序。因此“按复位就没了”还要区分你按的是哪一个键。最终以**断电后再上电，无需重新下载也能运行**作为固化成功的标准。

### 7.2 按你目前的 Programmer 版本操作

以下步骤依据你本机 V1.9.11.03 附带的《Gowin Programmer 用户指南》SUG502-2.2.1，第 3.4.5 节。板卡通用原理图标注外部 Flash 为 `MX25L12845G`；操作选项不需要据此随意修改工程引脚。

1. 先用 SRAM Program 验证准备保存的那份 `.fs`。当前 R3 收图仍有问题，建议先验证 R2 的完整帧，再把稳定版本固化。
2. 打开 Programmer，确认器件仍是 `GW5AT-60B`。
3. 双击器件所在行的 **Operation** 单元格，打开 **Device Configuration**。也可用 Edit → Configure Device。
4. **Access Mode** 选择 `External Flash Mode 5A`。
5. **Operation** 选择 `exFlash Erase,Program,Verify thru GAO-Bridge 5A`。
6. **File name** 选择刚才已经通过 SRAM 测试的 `.fs`，例如 R2 的 `vision_robot_netfix.fs`。
7. Flash Type 先保持软件默认的 `Generic Flash`，起始地址保持 `0x000000`。
8. 点 Save。回到主表确认操作已不再是 SRAM Program，文件路径正确。
9. 点下载/运行按钮，等待擦除、写入和校验全部完成。写入期间保持板卡电源和 USB 下载连接稳定。这会替换 Flash 中原来的开机程序。
10. 完成后断电，稍等，再上电；不要再次点 SRAM 下载。检查 LCD 摄像头画面，并运行 probe 验证完整帧。

如果下拉列表没有上述选项，先发 Device Configuration 窗口截图，不能随意改为 Internal Flash Mode 或 Slave SPI Mode。它们不是此处要选的访问方式。

新版本手册把 `5A` 显示为 `Arora V`；当前官方手册可见 [Gowin Programmer 用户指南，第 3.4.5 节](https://cdn.gowinsemi.com.cn/SUG502.pdf)。本机旧版里的 5A 名称正常。

固化后仍可用 SRAM Program 临时试新代码，断电后回到 Flash 保存的版本。只有确认新版本稳定，再更新 Flash。以后若“重启变成旧版”，先检查是不是只下载了 SRAM。

## 8. 电脑用 SSH 和 VS Code 编辑香橙派

### 8.1 在香橙派开启 SSH，只做一次

安装好上面的 `openssh-server` 后，在香橙派运行：

```bash
sudo systemctl enable --now ssh
systemctl is-active ssh
ss -lnt | grep ':22'
ip -4 -br addr
```

应看到 `active` 和监听的 22 端口。记下 **wlan0 的当前 IPv4 地址**。以下示例假设仍是 `10.8.180.11`；请全部换成你实际查到的地址。

### 8.2 在电脑 PowerShell 验证连接

电脑和香橙派连接能互通的同一个 Wi-Fi/局域网。在 **Windows PowerShell** 输入：

```powershell
ssh -V
Test-NetConnection 10.8.180.11 -Port 22
ssh orangepi@10.8.180.11
```

`ssh -V` 正常会打印 OpenSSH 版本。若 Windows 找不到 ssh，在“设置 → 应用 → 可选功能”查找并安装 **OpenSSH 客户端**；不是服务器。

第一次连接会询问是否信任主机。可在香橙派先查看指纹：

```bash
sudo ssh-keygen -lf /etc/ssh/ssh_host_ed25519_key.pub
```

核对电脑提示的指纹，输入 `yes`，再输入香橙派账户密码。不要把密码发到聊天里。登录后看到 `orangepi@orangepi5` 就已连接成功；输入 `exit` 返回电脑 PowerShell。

若 `TcpTestSucceeded` 为 False，先检查香橙派 SSH 是否 active、Wi-Fi IP 是否改变。校园 Wi-Fi 可能隔离设备：即使双方都能上网，也未必能 SSH。可把电脑和香橙派接到同一个允许设备互通的热点/路由器测试。这个问题不需要修改 FPGA 有线 IP。

### 8.3 在电脑 VS Code 打开香橙派目录

1. 打开电脑 **Windows 版 VS Code**。
2. 左侧扩展，搜索 `Remote - SSH`，安装发布者为 Microsoft 的扩展。
3. 按 F1，选择 **Remote-SSH: Connect to Host...**。
4. 输入 `orangepi@你的香橙派WiFi地址`。比如 `orangepi@10.8.180.11`，不能原样输入中文占位词。
5. 系统类型选择 Linux，按提示输入密码，等待初始化结束。
6. 新窗口左下角应显示 `SSH: ...`。
7. File → Open Folder，输入 `/home/orangepi/acg720_vision_client_v2`，点确定。
8. 左侧可以打开 `pc` 内文件；按 Ctrl+S，修改直接保存到香橙派。
9. Terminal → New Terminal。这是香橙派终端，可直接运行 `python3 pc/udp_probe.py ...`，不需要进入 WSL。

Remote-SSH 支持 ARM64 Linux 主机，远端 VS Code Server 与香橙派原来安装的 VS Code 独立。使用方式参考 [微软 Remote-SSH 文档](https://code.visualstudio.com/docs/remote/ssh)。

如果 SSH 已成功，但 VS Code 初始化失败，查看 View → Output → Remote - SSH 的错误。先继续使用普通 SSH 和下一节的 scp，程序环境无需因此重装。

### 8.4 一个容易误解的地方：远程编辑不等于远程桌面

SSH 可以编辑文件、运行诊断和训练脚本，但不会自动把香橙派桌面上的 OpenCV 窗口显示到电脑。

- `udp_probe.py` 没有 GUI，可以在 VS Code SSH 终端运行。
- `udp_video_viewer.py` 和 `capture_dataset.py` 需要 GUI，当前先在香橙派本地桌面终端运行。
- 如果远程运行它们出现 `could not connect to display`，先换回香橙派桌面终端，不要反复重装 Python。

你可以在电脑上修改文件，同时保留香橙派屏幕显示画面。之后如需在电脑浏览器远程查看视频，我们再增加网页查看器；现在不用为此改变接收环境。

### 8.5 以后如何不用 U 盘更新文件

**建议当前以电脑的工程为主，香橙派负责运行。**我修改电脑侧的工程后，你用 PowerShell 传过去。以下仍须把示例 IP 换成实际地址：

先只更新诊断脚本：

```powershell
scp "D:\GOWIN_PROJECTS\acg720_vision_robot\pc\udp_probe.py" orangepi@10.8.180.11:/home/orangepi/acg720_vision_client_v2/pc/
```

需要同步整个客户端时，先关闭香橙派正在运行的接收程序，再执行：

```powershell
scp -r "D:\GOWIN_PROJECTS\acg720_vision_robot\pc" orangepi@10.8.180.11:/home/orangepi/acg720_vision_client_v2/
```

目标是项目目录，结果仍是 `acg720_vision_client_v2/pc`。不会重新创建 Python 环境，项目内 `dataset` 照片也不在这个复制范围内。它会覆盖同名程序文件；若你在香橙派改过代码，先把修改取回电脑或提交保存。

从香橙派取回本次报告，在 PowerShell 执行：

```powershell
New-Item -ItemType Directory -Force "D:\GOWIN_PROJECTS\acg720_vision_robot\diagnostics"
scp orangepi@10.8.180.11:/home/orangepi/fpga-r3.probe.json "D:\GOWIN_PROJECTS\acg720_vision_robot\diagnostics\"
scp orangepi@10.8.180.11:/home/orangepi/fpga-r2.probe.json "D:\GOWIN_PROJECTS\acg720_vision_robot\diagnostics\"
```

报告目录已被 Git 忽略，避免意外上传像素样本。

### 8.6 也可以以后直接从 GitHub 更新

网络和 Git 安装正常后，在香橙派单独建立一次仓库目录：

```bash
cd ~
git clone https://github.com/Vera-loves-program/acg720-vision-robot.git
cd ~/acg720-vision-robot
python3 pc/udp_probe.py --bind 192.168.10.3 --seconds 10
```

以后更新：

```bash
cd ~/acg720-vision-robot
git pull --ff-only
```

这个目录和旧 ZIP 的 `~/acg720_vision_client_v2` 是两个目录，要选定一个作为后续运行目录，避免编辑一个、运行另一个。旧目录里已拍的照片先保留。仓库更新不会更新 FPGA 已烧录的程序。

若 Git 提示本地修改会被覆盖，先保存/提交修改，不能用 `reset --hard` 丢掉它。VS Code SSH 编辑香橙派文件也不会自动同步回电脑或 GitHub。

## 9. 现在按这个顺序做

1. 更新诊断脚本，测试当前 R3，保存 `fpga-r3.probe.json`。
2. 同一香橙派、同一设置，下载旧 R2 后再测，保存 `fpga-r2.probe.json`。
3. 修复 APT 下载，安装 OpenCV 和 SSH；不用重装系统。
4. 电脑 SSH 连通后，用 VS Code 编辑和 scp 更新，停止反复创建环境和拷 ZIP。
5. 确认选定版本能正常收图，再按第 7 节写入外部 Flash，断电重启验证。

本次在电脑完成了诊断脚本的合成包统计和真实本机 UDP 回环测试，以及软件源文本替换测试；没有编译 FPGA，也没有远程修改你香橙派的系统。真正的 R2/R3 上板对照仍需你运行，不能把这些软件测试当成硬件问题已修好。
