# 16 · OpenOCD / GDB备用流程（2026-11-11）

正常现场操作以[Ozone主教程](12-基地现场一页纸.md)为准。本页仅在现场明确使用ST-Link/OpenOCD时由队员采用。
Ozone工程使用原有、受支持的J-Link/J-Trace；本页的ST-Link配置不能直接套进Ozone。
同一块板一次只由一个软件占用探针，先关闭另一调试会话。

这是备用流程，不要求原先使用Ozone的人改用命令行。烧录/暂停前先隔离电机驱动力；
未实测电机实际卸力和故障响应，不把下面命令当成整车安全验收。

## 2. 打开电脑，检查文件

按`Win+R`，输入`powershell`回车，复制整块：

```powershell
$ProjectRoot = 'D:/ForTest/steered_skeleton/steered_skeleton'
$OpenOcdExe = 'D:\ForDownload\stm32-env\openocd\bin\openocd.exe'
$OpenOcdScripts = 'D:\ForDownload\stm32-env\openocd\share\openocd\scripts'
Set-Location -LiteralPath $ProjectRoot
Test-Path $OpenOcdExe
Test-Path "$OpenOcdScripts\interface\stlink.cfg"
Test-Path "$OpenOcdScripts\target\stm32f4x.cfg"
Test-Path "$ProjectRoot\Chassis\build\steered_chassis.elf"
Test-Path "$ProjectRoot\Gimbal\build\steered_gimbal.elf"
```

应看到**5个True**。有False先停，说明路径或文件没准备好。这些路径只属于这台电脑。
换电脑时改路径，或把已核对的两份ELF带到队里的烧录电脑。

没有改代码可直接使用配套ELF。需要重编时，在同一窗口执行：

```powershell
cmake -S Chassis -B Chassis/build/audit -G Ninja -DCMAKE_BUILD_TYPE=Debug
if ($LASTEXITCODE -ne 0) { throw '底盘配置失败，停止' }
cmake --build Chassis/build/audit
if ($LASTEXITCODE -ne 0) { throw '底盘编译失败，停止' }
cmake -S Gimbal -B Gimbal/build/audit -G Ninja -DCMAKE_BUILD_TYPE=Debug
if ($LASTEXITCODE -ne 0) { throw '云台配置失败，停止' }
cmake --build Gimbal/build/audit
if ($LASTEXITCODE -ne 0) { throw '云台编译失败，停止' }
```

找不到cmake、Ninja或ARM编译器时，请队员恢复工具链环境，不要改源代码处理。
ELF在各板`build/`，HEX/BIN在`build/audit/`。出现error或链接失败时不能烧旧ELF冒充新版本。
本轮修正了原RWX段属性；本次工具链构建无警告，仍须确认自己这次编译最后退出码0。

## 3. 烧录：底盘一份、云台一份

### 3.1 先隔离驱动力

让队员关闭/隔离电机驱动电源，只给待烧的控制板按规定供电。一次只接一块板。
ST-Link按具体型号连接SWDIO、SWCLK、GND、所需目标电压参考，必要时连接NRST。
**不要照着“3V3对3V3”就并接两路电源。** VAPP/VTref与某些探针电源输出的含义不同，
不认识针脚就让队员按型号手册接线。参考[ST官方UM1075](https://www.st.com/resource/en/user_manual/dm00026748.pdf)。

### 3.2 底盘板

确认ST-Link接底盘板，在刚才的PowerShell窗口复制：

```powershell
& $OpenOcdExe -s $OpenOcdScripts -f interface/stlink.cfg -f target/stm32f4x.cfg -c 'adapter speed 1000' -c "program {$ProjectRoot/Chassis/build/steered_chassis.elf} verify reset exit"
if ($LASTEXITCODE -ne 0) { throw '底盘烧录失败，停止' }
```

需要看到`Programming Finished`、`Verified OK`且无error。`verify`核对写入，`reset`使程序开始运行，
所以此时仍保持驱动力隔离。命令含义见[OpenOCD官方烧录说明](https://openocd.org/doc/html/Flash-Programming.html)。
灯亮、文件存在或编译成功，都不能代替烧录验证。

### 3.3 云台板

控制板断电后换线到云台板，重新按规定供电，执行：

```powershell
& $OpenOcdExe -s $OpenOcdScripts -f interface/stlink.cfg -f target/stm32f4x.cfg -c 'adapter speed 1000' -c "program {$ProjectRoot/Gimbal/build/steered_gimbal.elf} verify reset exit"
if ($LASTEXITCODE -ne 0) { throw '云台烧录失败，停止' }
```

同样确认`Verified OK`。两板成功后断电、拔调试器，再让队员恢复正常供电连接。

| 报错/现象 | 先处理什么 |
|---|---|
| 找不到openocd或ELF | 回第2节核对路径 |
| 找不到cfg | 查`$OpenOcdScripts`和`-s` |
| ST-Link打不开 | USB线、驱动、是否被另一个软件占用 |
| 目标连接失败 | 控制板电源、共地、SWD和NRST，请队员核对 |
| 反复复位难连接 | 保持驱动力隔离，请队员按NRST/复位方案连接，不为烧录关安全逻辑 |
| 两板可能烧反 | 隔离驱动力，按标签重新烧正确文件，不用推杆判断 |

## 10. 请队员帮你读状态：用真实符号，不调用目标函数

先隔离驱动力。隔离可能改变在线/CAN条件，此时Dead可能正常，不能冒充原带电运行状态。
**不要带运动指令halt或打断点**：本机OpenOCD目标配置会在halt时冻结看门狗，电机可能保持上一指令。

只接要查的一块板，第一个PowerShell窗口运行服务：

```powershell
& 'D:\ForDownload\stm32-env\openocd\bin\openocd.exe' -s 'D:\ForDownload\stm32-env\openocd\share\openocd\scripts' -f interface/stlink.cfg -f target/stm32f4x.cfg -c 'adapter speed 1000'
```

第二个窗口打开GDB；查云台时把下面ELF换成云台路径：

```powershell
& 'D:\ForDownload\stm32-env\arm-gnu-toolchain-15.2.rel1-mingw-w64-i686-arm-none-eabi\bin\arm-none-eabi-gdb.exe' 'D:/ForTest/steered_skeleton/steered_skeleton/Chassis/build/steered_chassis.elf'
```

看到`(gdb)`后逐行输入，注意不是PowerShell命令：

```text
set language c++
target extended-remote localhost:3333
monitor halt
source D:/ForTest/steered_skeleton/steered_skeleton/docs/audit_evidence/field_read.gdb
```

脚本读取静态对象，避免调用GetRobot等目标函数。两板ELF中的符号与字段类型已离线核对，板上数值仍需实读。

| 输出 | 怎么理解 |
|---|---|
| state_ | 0=Dead，1=恢复自检，2=Working |
| estop_ | true=急停 |
| can_tx_guard_.faulted_ | true=CAN故障锁存 |
| IMU calibrated_/sample_valid_/healthy_ | 三者都需正常；sample_valid_是缓存，还需用uwTick减last_sample_tick_并检查时钟/DRDY时间戳≤20ms，halt时不会自动更新 |
| rc_checker_ | 本地接收机缺省可离线；底盘所选本地/云台转发来源必须有效 |
| motor_checkers_ | 8个或2个电机槽位；结合打印的时间戳与uwTick，不只看缓存is_offline_ |
| peer received_ | false=尚未成功收过对板帧；true还需看last_rx_tick_新鲜度 |

确认物理驱动力隔离后，可额外读取拨杆：

```text
print $r->rc_->rc_l_switch_
print $r->rc_->rc_r_switch_
```

原始值1=上，3=中，2=下；如果程序尚未初始化或指针为空，不强行读取，让队员检查启动过程。
CPU暂停时拨杆不会继续被软件解码。要换位置比较读数，保持驱动力隔离，由队员`continue`运行采样，
再用GDB的Ctrl+C暂停读取；不要误以为暂停时改变拨杆就能更新变量。
查底盘时还可读`print $r->remote_armed_`和`print $r->mode_`；云台没有这两个字段。
查完`quit`，服务窗口Ctrl+C；再测试运动须按第4节完整重新上电确认。
禁止set variable、手工setInput强行启动。


## 参考

[OpenOCD官方烧录说明](https://openocd.org/doc/html/Flash-Programming.html)。
本页命令来自此前现场教程，文件路径已离线核对；未连接控制板执行烧录或在线GDB读取。
