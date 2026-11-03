# 本轮交付：Pi修订5与原FPGA续走候选

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
