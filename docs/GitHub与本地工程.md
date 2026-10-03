# GitHub 与完整本地工程的范围

仓库：[Vera-loves-program/acg720-vision-robot](https://github.com/Vera-loves-program/acg720-vision-robot)。它当前是公开仓库。

此次同步自编的 FPGA 顶层/滤波/显示/触摸/协议模块、Python 主机工具、工程入口、配置、测试与文档。主机工具可以直接下载运行；新增拍照工具与图像查看器所需的共享模块均包含在 `pc/`。

完整 FPGA 工程继续位于 `D:\GOWIN_PROJECTS\acg720_vision_robot`。其中 `src/vendor/` 与 `src/gowin_pll_45/` 是厂商例程/生成 IP，此次不发布到公开仓库。新 R3 工程继续引用你本地已存在的相同依赖，不能只下载公开仓库就宣称具备完整可编译的 FPGA 依赖。厂商依赖的公开再分发范围尚未确认，先保留在本地。

编译/布局布线产物、虚拟环境与采集照片不提交普通源码 Git。自己的照片通过压缩包/网盘发给队友，位流和数据另外备份；上传源码本身不会自动释放本地文件空间。

## 后续同步

Codex 修改时先获取远端最新状态，以远端当前 `main` 为父提交发布自编文件，避免覆盖别人的历史。本地完整工程也记录独立的 Git 提交。两个历史分别服务于完整本地回退和公开自编源码回溯，不直接把含厂商依赖的本地完整历史推入公开仓库。

手动同步时，在 Windows PowerShell 运行：

```powershell
Set-Location D:\GOWIN_PROJECTS\acg720_vision_robot
.\tools\sync_github.ps1 -Message "描述本次修改"
```

脚本使用 `D:\GOWIN_PROJECTS\.github-publish\acg720-vision-robot` 作为独立发布目录，先拉取远端，再复制允许发布的自编文件，然后提交/推送。它不自动删除远端额外文件；如需要删除某个已发布旧文件，应另作明确的删除提交。发布目录有未提交修改或拉取冲突时会停止，保留现场。

Git HTTPS 推送需要你已配置 GitHub 登录/凭据；脚本不写入或打印 token。Codex 本次可以使用已连接的 GitHub 工具同步，不要求重新配置 Windows Python。

厂商依赖和生成 IP 不属于同步白名单。未来如果需要公开完整依赖，先确认对应授权，再调整发布范围；不用为拍照和新 UI 文档的发布等待这一事项。
