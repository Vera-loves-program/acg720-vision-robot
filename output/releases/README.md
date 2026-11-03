# 当前交付：预设演示测试候选（11月4日03:31）

**布线已完成，完整时序未闭合；未烧录、未实车验证。** 最新RTL来源为 `7ac3189`，分支 `feature/preset-demo-20261104`，[PR #3](https://github.com/Vera-loves-program/acg720-vision-robot/pull/3)。这份是待现场验证的候选，不能称为稳定版。

本机新FS的完整路径：

```text
D:\GOWIN_PROJECTS\acg720_obstacle_resume\output\releases\preset-demo-20261104\vision_robot_preset_demo_20261104_0331_NOT_TIMING_CLOSED.fs
```

SHA256：`8c96072354b93df40c9e4774c1b5c65c0be9612b443338deb6d3dffe08ffcf51`，20,059,120字节。文件只在本机生成，未上传GitHub。

同目录还有 `build-result.json` 和 `validation-and-sources.zip`，包含冻结编译输入、构建报告与HDL日志。Git中保留[验证记录](../../docs/validation/preset-demo-20261104.json)。

如由现场成员决定做受控下载测试，在Gowin Programmer中双击当前器件行的FS File，手动选择上述完整路径；操作保留SRAM Program，点击绿色运行按钮并核对下载日志。SRAM下载断电会丢失，不会更新Flash。代理没有连接下载器或操作底盘。

下载后的新版识别特征：题5、题6都有 **PRESET DEMO**。点击后LCD标题变为 **PRESET DEMO**，完整等待5秒，再在传感器就绪时启动。题5是直行、左30°、直行、回原方向、直行；题6是两次150°、右调8°、直行停止。STOP、HOME、BACK可以取消。它是明确标注的预设轨迹，不依赖或伪造视觉目标。

**Pi保持现有v22修订6，无需上传新包或修改参数。** [详细按键、参数与验收](../../docs/预设轨迹演示与5秒启动_20261104.md)。

本轮五组HDL测试通过：544条序列断言、515条带实际导航器/命令寄存级的断言、517条实际触摸/子系统断言、208条命令原子性/取消断言，另有54,736周期电机控制器差分测试；原雷达保护/UI/规划回归也已运行通过。这些均为离线验证。

最终布局布线资源：Logic 39,033（比基线+492）、Register 20,408（+295）、CLS 25,266（+111）、BSRAM 60、DSP 7.5。地图存储没有增加。

时序仍有 **776个setup、376个hold违例端点**，Recovery/Removal总计数未提供，`upload_allowed=false` 保留。主时钟Fmax 50.018/50MHz、接收以太网125.697/125MHz、发送以太网125.313/125MHz、LCD46.276/45MHz；主时钟裕量很小。名义时钟指标达标不能证明全工程时序闭合。

---

下面是历史基线与Pi代码包记录，不能混作本次预设演示FS。

# 本轮交付：Pi修订6与原FPGA续走候选

使用 `pi-tracking-r6-20261104/acg720-orangepi-v22-r6-code.zip` 修复首帧黑屏，保留标定/模型；[更新步骤](../../docs/香橙派v22修订6启动修复_20261104.md)。修订5原包保留溯源，不再部署。

Pi新包位于本机 `pi-tracking-r5-20261104/acg720-orangepi-v22-r5-code.zip`。本次Pi工作分支为 `fix/pi-tracking-smooth-20261104`，需人工上传代码包并重启服务，保留现场标定和模型。[限位核对、安装与调参流程](../../docs/香橙派v22修订5跟踪调参_20261104.md)。以下FPGA交付不变，不需要因为此次Pi改动再编译或下载FPGA。

当前工程为 `D:\GOWIN_PROJECTS\acg720_obstacle_resume`，分支 `fix/teammate-obstacle-resume-20261103`。基于用户报告能起步、到点、遇障停车的队友源码，修复地图完成事件被丢弃导致的等待，等长路线优先左绕。旧工程目录中的位流不是本次结果。

文件在本机 `teammate-resume-20261103/`：

| 文件 | 用途 |
|---|---|
| `vision_robot_teammate_resume_2357_NOT_TIMING_CLOSED.fs` | 11月3日23:57生成的FPGA测试候选 |
| `fpga-source-2357.zip` | 本次编译冻结的FPGA输入和哈希 |
| `validation-evidence-2357.zip` | 两次构建报告/日志、新增HDL正反例、原有动态回归日志 |

位流SHA256：`2d3ada689372f22dbe0bdde4aec4507b68596de77143153db3accfbafd72b132`，19,986,320字节。

**布线完成，完整时序未闭合**：732个setup、386个hold违例端点，Recovery/Removal计数未知，构建记录 `upload_allowed=false` 保留。未通过自动下载门槛，未烧录或实车验收。不能只凭FS存在、工具退出0或主要时钟达标宣称全部通过。

GitHub只保存源码、说明和 [验证记录](../../docs/validation/teammate-resume-20261103.json)，不上传FS、ZIP、照片、模型或实测标定。

## 手动选文件与验收

Gowin Programmer保留原器件设置，在FS File中手动选上述完整文件，核对文件名/哈希后再决定测试下载。SRAM Program断电丢失，不能当作已更新Flash。代理不自动下载或操作底盘。

题4先测空场(1000,0)，再测试障碍存在时规划绕行、移走后同一目标恢复、等待时STOP取消；记录位流SHA、LCD状态和实测位姿。[详细步骤、修复依据与测试限制](../../docs/队友基线续走协作_20261103.md)。

## Pi

本轮新增Pi v22修订5应用更新，协议格式不变。部署前备份现场gimbal.json；请看顶部的修订5流程，不要照历史修订2/4指南覆盖当前代码。设备不会因源码或Git更新而自动更新。

历史R7记录保留在 `records/vision_robot_core_r7_2136.json`，仅供溯源。
