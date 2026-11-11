# Ozone现场教程纠正（2026-11-11）

用户要求沿用原有Ozone调试流程，已将docs/12改为Ozone主流程，增加debug/ozone/chassis.jdebug和gimbal.jdebug；
原OpenOCD/GDB步骤移至docs/16。README、docs/00、06、08、09、14的相关入口同步。
本次仅修改文档和调试工程配置；未修改生产源码、构建参数或固件。

核对依据：本机Ozone.exe文件版本3.2.6.0；本机UM08025_Ozone.pdf共348页，API/常量依照该版手册。
已核对OnProjectLoad、File.Open、Project.SetDevice/SetHostIF/SetTargetIF/SetTIFSpeed、
AddSvdFile、Edit.SysVar、Debug.SetConnectMode/SetResetMode及对应常量。

离线检查78项通过：两份脚本的设置/API常量、ELF相对路径、ARM ELF格式、SVD文件、nm符号及GDB中对象字段。
教程开头5个Test-Path对象均存在；本地Markdown链接与git diff --check通过。
两板ELF SHA-256与fix-verification-2026-11-10.json逐项一致，未重新生成固件。

验证边界：本机电脑控制接口不可用，未用Ozone GUI实际打开工程或验证它对当前ELF调试信息的解析。
未连接探针/控制板，未执行下载、校验、Attach、Continue或任何真实电机控制；
不能把手册核对和GDB离线字段存在当成Ozone上板验证。
现场首次打开应先核对工程和ELF，再按docs/12在驱动力隔离条件下验证下载/校验及变量显示。


## 运行步骤纠正

早期工程默认Reset & Halt，却没有把恢复运行作为遥控试验的明确前置步骤；照着停住的会话推杆不会得到控制响应。
两份工程已统一改用`Debug.SetResetMode(RM_RESET_AND_RUN)`，同时在下载前明确遥控开机急停和隔离驱动力。
docs/12和17新增Running/周期计数检查，以及旧会话Halted时Continue恢复的具体步骤；Attach不等于Continue。
本机Ozone3.26可执行文件含`RM_RESET_AND_RUN`标识，不含`RN_RESET_AND_RUN`；手册常量表中的RN拼写不能直接照抄。
本次仍为离线配置/文档验证，未启动真实目标；未把脚本配置改动宣称为已验证板上运行。
