# V4 参赛代码包

- Vivado 2018.3 工程入口为 vivado/v4.xpr，器件为 xc7z020clg400-2。
- 使用 Vivado 2018.3 批处理执行 tcl/build.tcl 可重新构建。构建脚本只生成下载文件。
- release/v4.bit、v4.ltx、v4.dcp 和时序报告均来自本次对当前源码的重新构建。
- camera/main.py 为配套 OpenMV 采图和 UART 传输脚本。模型推理在 FPGA。
- model_candidates 保存板端使用的模型 ROM；config 保存姿态和路径 ROM。
- model_development 保存历史模型开发代码及训练导出模型。历史脚本中的本机路径需按本地数据目录调整。原始采集数据未随代码包提交，FPGA 构建不依赖训练数据。
- scripts/test_v4_radius.ps1 与 scripts/test_coordinates.ps1 对应本次四项通过的功能仿真。测试向量在 work/coordinate_test。若 Vivado 安装路径不同，调整脚本内工具路径。
- 本次验证结果见 reports/delivery.json、reports/radius.log、reports/cycle.log 和 release/timing.rpt。模型独立验证记录为历史验证资料。
