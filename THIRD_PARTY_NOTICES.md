# 授权范围与第三方声明

本项目贡献者拥有并有权授权的原创源码与文档采用根目录 [MIT License](LICENSE)。保留原作者署名；贡献者各自保有其作品版权。

The root MIT License applies only to original project code and documentation
that the project contributors have the right to license. Third-party code,
vendor IP, generated vendor components, dependencies, and model artifacts are
not relicensed by this project. Their original notices and applicable upstream
terms take precedence for those components.

## FPGA厂商及板级参考文件

- `src/vendor/` 包含厂商IP、生成文件和板级参考代码；`src/gowin_pll_45/` 包含高云PLL生成文件。这两个目录不属于本项目MIT授权范围。
- 高云PLL文件保留 `Copyright (C)2014-2025 Gowin Semiconductor Corporation. All rights reserved.`。DDR、FIFO等含受保护的高云IP，不因仓库公开而取得额外的使用或再分发许可。适用许可由原厂条款确定；本仓库没有为未明确授权的第三方文件补授许可。
- `src/vendor/ethernet/crc32_d8.v` 保留 Easics NV 1999–2008 版权、原许可条件及免责声明；使用和分发时保留文件原声明。
- 其他文件已有的第三方版权、许可或来源说明继续有效，即使它位于上述目录之外。

## 人脸模型

- YuNet来源与固定版本见 `models/face_yunet/manifest.json`。
- 原作者为 Shiqi Yu；原MIT全文保存在 `models/face_yunet/LICENSE`，不要用本项目版权行替换。

## 小狗模型及Python依赖

- 小狗模型来源、模型哈希和导出信息见 `models/dog_plush/manifest.json`。模型权重、训练图片及数据集不因根目录LICENSE而自动采用MIT；运行或分发模型时遵守模型来源及相关工具的适用许可。
- NumPy、OpenCV、ONNX Runtime及模型准备工具等依赖分别适用各自上游许可。本项目不替这些依赖重新授权。
- 不随源码公开个人照片、现场实测标定、密钥或账号信息；示例配置是本项目文档的一部分。

重新分发本项目代码时，应同时保留根目录LICENSE、本声明以及相关第三方文件中的原始声明。本说明只界定本项目的授权范围，不替代第三方许可证正文。
