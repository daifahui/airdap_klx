# CLAUDE.md - AirDAP 工程开发简介

> 面向后续接入的 AI 代理与新开发者。先读本文件建立全局模型，再按“推荐阅读顺序”深入源码。
> 本文件是导读，不替代源码；如与源码冲突，以源码为准。

## 1. 项目是什么

AirDAP 是基于沁恒 CH570/CH572（RISC-V，100 MHz，USB Full-Speed + 2.4 GHz 私有射频）的低成本
**无线串口 + CMSIS-DAP 调试器**。

一套固件支持三种产品角色：

- **Local**：纯有线。USB 复合设备 = CDC 串口 + CMSIS-DAP v1/v2 + DFU，本地驱动 SWD/JTAG 与 UART。
- **Host**：插电脑的一端。USB 复合设备不变，但串口字节流和 DAP 命令通过 2.4 GHz 转发给 Target。
- **Target**：接目标板的一端。不枚举 USB，负责把无线数据还原成物理 UART / SWD / JTAG。

两台设备自动完成发现、配对、信道选择；也支持 TinyDongle（只有 Host，`BSP_USB_DONGLE=1`）。

许可：

- 工程原创软件：MIT（`firmware/LICENSE`）。
- 硬件：CERN-OHL-P-2.0。
- `src/dap` 中 Arm CMSIS-DAP 部分：Apache-2.0。
- `src/ch572` 中适配的 WCH EVT 代码：保留原版权与许可。
- 详见 `firmware/NOTICE`。

## 2. 关键事实速查

| 项目 | 值 |
|---|---|
| MCU | CH570/CH572，RV32IMC，最高 100 MHz |
| RAM | 12 KB（链接脚本），栈 1.5 KB |
| 应用 Flash | `0x00000000` ~ `0x0003C000`（240 KB），见 `src/flash_layout.h` |
| USB | Full-Speed 设备，EP0 + EP1~EP6，VID:PID = `0x1209:0x572D`，bcdDevice `0x0101` |
| USB 接口 | DAP Bulk、CDC ACM、DAP HID、DFU Runtime（共 5 个 interface） |
| 串口 | 300 baud ~ 1.5 Mbaud，5~8 数据位，1/1.5/2 停止位，无/奇/偶/mark/space 校验 |
| SWD | GPIO 后端 ≤ 12.5 MHz；SPI 加速 ≥ 1 MHz，最高 25 MHz |
| JTAG | GPIO 位操作，最高约 12.5 MHz（实测常用 6.25 MHz） |
| RF | WCH RF Basic 2M，私有协议，非 BLE；默认 0 dBm，信道 36（2478 MHz） |
| 公共 AA | `0x71764129` |
| 配对窗口 | 100 ms USB 预断开 + 2000 ms USB/RF 联合扫描 |
| RF 租约 | 900 ms 无有效数据则重新发现 |
| 看门狗 | 仅复位模式，约 335.5 ms @100 MHz；复位后进入 DFU |
| 固件升级 | USB DFU Runtime，更新范围 `0x0~0x3C000` |

## 3. 仓库结构

```text
AirDAP/
├─ README.md / README_zh.md          使用说明、模式逻辑、实测性能
├─ AirDAP.jpg
├─ hardware/README.md                原理图/PCB 外链（OSHWHub）
├─ .github/
│  ├─ workflows/build.yml            CI 构建 FullRole 与 Dongle
│  └─ scripts/verify_build.py        校验镜像、符号地址、RAM 边界、DFU 包
└─ firmware/
   ├─ LICENSE / NOTICE               软件许可与第三方声明
   ├─ .clang-format                  代码格式
   ├─ projects/                      MounRiver Studio 工程
   │  ├─ AirDAP.wvsln                整个解决方案
   │  ├─ Local/ Host/ Target/       三个角色镜像工程
   │  ├─ Dongle/                    TinyDongle：固定 Host
   │  ├─ DFU/                       DFU 模式固件（运行在 RAM）
   │  ├─ FullRole/                  把 DFU+Local+Host+Target 合并成整片镜像
   │  └─ ld/                        local/host/target/dongle/dfu/fullrole 链接脚本
   └─ src/
      ├─ main.c                     前台协作式主循环
      ├─ app_config.h               编译期功能裁剪与产品参数
      ├─ flash_layout.h             Flash 分区 ABI
      ├─ ring.h                     SPSC 字节环
      ├─ runtime.c/h                RAM 代码使用的拷贝函数
      ├─ ch572/                     芯片头、startup.S、时钟、Flash 控制器
      ├─ bsp/                       引脚、时钟门控、UID、时基、LED
      ├─ usb/                       USB 设备栈、描述符、CDC/DAP/DFU 端点
      ├─ dap/                       CMSIS-DAP 协议、DAP 队列、SWD/JTAG 后端
      ├─ rf/core/                   RF Basic 2M 物理层与校准
      ├─ rf/                        发现、可靠链路、信道策略、消息复用、UART/DAP 桥
      ├─ uart/                      UART 驱动与波特率规划
      ├─ mode/                      角色镜像热切换与自动模式状态机
      └─ dfu/                       Runtime DFU + 独立 DFU 应用
```

## 4. 构建、产物与验证

### 4.1 构建方式

工程使用 MounRiver Studio（MRS）原生日程，链接脚本在 `firmware/projects/ld/`。

- IDE：打开 `firmware/projects/AirDAP.wvsln`，选择 `FullRole` 或 `Dongle` 构建。
- 命令行（CI 使用）：

```bash
mrs-build firmware/projects/AirDAP.wvsln \
  --expect FullRole/obj/FullRole.hex \
  --expect FullRole/obj/FullRole.bin \
  --expect FullRole/obj/FullRole.dfu \
  --expect Dongle/obj/Dongle.hex \
  --expect Dongle/obj/Dongle.bin \
  --expect Dongle/obj/Dongle.dfu \
  --require-fresh
```

- 典型产物在 `firmware/projects/<Variant>/obj/`：`.hex`、`.bin`、`.dfu`。
- `obj/` 是生成目录，通常不入库；仓库里可能残留 Dongle 的 `obj/` 用于参考。

### 4.2 构建变体

| 工程 | 关键宏 | 链接地址 | 内容 |
|---|---|---|---|
| Local | `APP_MODE=0` | `0x00002000` | USB + 本地 DAP + 本地 UART + DFU，无 RF |
| Host | `APP_MODE=2` | `0x00010000` | USB + DAP 队列 + RF Host + RF UART + DFU；无本地 SWD/UART 引擎 |
| Target | `APP_MODE=3` | `0x00020000` | RF Target + 本地 DAP + 物理 UART；无 USB 栈 |
| Dongle | `APP_MODE=2`、`BSP_USB_DONGLE=1` | `0x00010000` | 固定 RF Host；无目标供电、无本地 DAP/UART 引擎 |
| DFU | 无 | `0x00000000` / RAM | DFU 模式设备，从 RAM 运行 |
| FullRole | 无（合并） | — | 把 DFU@0、Local@0x2000、Host@0x10000、Target@0x20000 拼成整片 |

`app_config.h` 由这些宏推导出 `APP_COMBINED_HOST`、`APP_COMBINED_TARGET`、`APP_HAS_RF`、
`APP_HAS_DAP`、`USB_HAS_DAP`、`RF_DAP_TARGET` 等开关。**新增功能时必须先确认目标变体是否编译该模块。**

### 4.3 验证

CI 只保证 FullRole 与 Dongle 的最终镜像：

```bash
python3 .github/scripts/verify_build.py
```

该脚本会检查：

- `_start`、`__flash_end` 是否落在对应 bank；Host/Dongle 还必须 `<= 0x1E000`；
- 栈底、mailbox 地址 `0x20000090`、mailbox 结束 `0x200000D0`；
- `FullRole.bin` 是否等于四个 bank 按地址拼接，且 `0x1E000~0x20000` 保持擦除态；
- `Dongle.bin` 是否等于 DFU + Host；
- DFU 镜像恰为 8192 字节，入口指针位于 `0x1FFC`；
- `.dfu` 包尾部 VID/PID/CRC 是否正确。

工程没有单元测试框架；修改后至少要做目标板实测（USB 枚举、串口回环、DAP 连接、无线配对）。

## 5. 镜像、Flash 与 RAM

### 5.1 Flash 分区

| 区域 | 起始 | 大小 | 说明 |
|---|---|---|---|
| DFU | `0x00000000` | 8 KB | 冷启动路由 + DFU 应用 |
| Local | `0x00002000` | 56 KB | 有线角色 |
| Host | `0x00010000` | 64 KB | 无线 Host / 启动扫描角色 |
| Host 绑定 A/B | `0x0001E000` / `0x0001F000` | 4 KB × 2 | `bind_store` 记录，Host 镜像不得越过 `0x1E000` |
| Target | `0x00020000` | 112 KB | 无线 Target |
| 结束 | `0x0003C000` | — | 应用上限 |

地址定义在 `src/flash_layout.h`，必须与链接脚本、`FullRole` 合并脚本、DFU 地址范围保持一致。

### 5.2 RAM 与代码放置

- 12 KB RAM，栈 1.5 KB（`PROVIDE(__stack_size = 1536)`），实际启动保留 768 字节以上的下限由链接断言保证。
- `.highcode`：中断处理、位操作引擎、启动/时钟关键代码，拷到 RAM 执行。
- `.cold_flash`：不常执行的管理代码，留在 Flash。
- OVERLAY：
  - `.ov_boot` / `.rf_uart_boot`：UID/Flash 引导代码；
  - `.ov_gpio`、`.ov_spi`、`.ov_jtag`：三套 SWD/JTAG 后端，按命令边界切换；
  - `.ov_sleep` / `.rf_sleep`：USB 睡眠闭包，复用已停止的 RAM 区域。
- 角色切换时 RAM 工作集会被新镜像整体替换，只有 `0x20000090` 起的 64 字节 mailbox 保留。
- `startup.S` 装 `.highcode` 时会跳过 mailbox 区间；bss 清理也不能覆盖它。

## 6. 启动链与角色切换

### 6.1 冷启动

```text
复位
 -> 0x0000 DFU 冷启动路由 (boot.S)
    - 采样复位原因；看门狗复位 -> 进 DFU
    - 检查 0x10000 是否为合法 JAL；非法 -> 进 DFU
    - 清空 mailbox，防止旧热启动状态
    - 跳 0x10000 Host 镜像
 -> Host startup.S
    - 装 highcode/向量表到 RAM，清 bss，进入用户态
 -> main()
    - WDOG_Feed
    - 冷启动打开目标 LDO
    - MODE_MemoryBoot() 载入覆盖代码
    - BSP_Init()：HSE 电容、100 MHz PLL、UID、SysTick、中断优先级
    - MODE_ImageBoot()：热启动恢复或冷启动初始化模式控制器
    - DAP/UART/USB 初始化
    - RF_BridgeStart(HOST)，等待 100 ms 预断开，再进入 2 s 扫描
```

**冷启动总是先运行 Host 镜像**；Host 镜像既是“扫描胶囊”，也是最终 Host 角色。

### 6.2 2 秒窗口决策

`mode/app_mode.c` 只产出三种结果：

| 窗口内事实 | 结果 |
|---|---|
| USB 已配置 且 收到 Target 的 ADVERT/OFFER | Host |
| 到 2 s 截止，USB 已配置 | Local |
| 到 2 s 截止，USB 未配置 | Target |

Host 角色必须同时满足“USB 已配置 + 对端匹配”；只有对端广播但 USB 未枚举，截止后仍选 Target。
RF 握手在 Host 侧还受 `MODE_AppModeRfCommitAllowed()` 提交门控。

### 6.3 热切换（不复位芯片）

1. `MODE_ImageRequest(role)`：停 RF、停 UART、释放目标引脚、断开 USB；
2. 等 100 ms 让 USB 主机看到断开；
3. 把模式控制器状态写入 mailbox；
4. 关中断、停 SysTick、清 PFIC；
5. `ECALL` -> `mode_image_entry.S` 的机器模式处理程序直接跳到目标 Flash 基址。

不经过芯片复位，GPIO/LDO_EN/外设保持；新镜像 `startup.S` 重装 RAM 工作集，但保留 mailbox。
因此 Target 在角色切换时不会掉电，USB 也不会经历真正的芯片复位。

### 6.4 手动模式与回退

- USB 厂商请求 `0x01`：`0 Auto / 1 Local / 2 Host / 3 Target`，见 `usb/usb_device.c`。
- 手动选择置 `USB_OVERRIDE`，断电失效。
- USB 手动选 Target 后 10 秒无连接，自动回 Local。
- DFU DETACH 后模式控制器冻结，不再接受切换。

## 7. 运行时数据流

### 7.1 Local 有线模式

```text
USB CDC OUT EP5 --DMA--> UART_TxSubmit --> 256B TX 环 --UART IRQ--> TX FIFO --> PA2
PA3 --> UART RX FIFO --UART IRQ--> 4x64B 包环 --EP6 DMA 直接指向槽位--> USB CDC IN
USB DAP Bulk EP2 / HID EP1 --> DAP 队列(8 槽) --前台执行--> SWD/JTAG --> 原端点返回
```

- CDC 接收用 4 个 64 字节包槽：满包立即发布，不足 64 字节最多等 1 ms 封包；EP6 DMA 直接指向槽位，主机 ACK 后回收。
- CDC 发送侧在 TX 空间 >= 64 且接收压力低于一半时才 ACK 下一个 OUT 包，用 NAK 做背压。
- `SET_LINE_CODING` 会先规划“系统时钟 + 分频值”的最优组合，再在 TX 排空、RX 静默后切换。
- DAP 前台一次最多连续执行 8 条命令，但总时长限制在 1 ms 内。

### 7.2 无线串口（Host <-> Target）

```text
Host:   USB CDC OUT -> RF_UartSubmit -> 512B 可靠 TX 环
        -> RF_LinkBuild 加 22B 帧头 -> RF TX 2M

Target: RF RX -> RF_LinkReceive 校验 session/seq/ack/credit
        -> 512B RX 环 -> RF_UartPoll -> 64B 分片 -> UART TX 环 -> 物理串口

反向:   物理串口 RX -> Target 1024B RX 环 -> RF_TargetSend 64B 分片
        -> RF -> Host RX 环 -> USB CDC IN 64B -> 主机 ACK 后回收
```

RF 链路帧：

```text
[2B PHY tag/len][22B RU v6 头][payload <= 229B]
RU 头: 'R','U' | ctrl(ver<<4 | role) | len | session(8) | seq(4) | ack(4) | credit(2)
```

可靠性：

- 累计 ACK + 接收 credit；未 ACK 字节留在 TX 环，重传不回收。
- 3 ms 超时重传；Host 立即重试，Target 延迟约 1 ms。
- 重复帧按 sequence 丢弃；credit 超界/session 不匹配直接判坏帧。
- Target 回复一轮最多连发 2 帧（458 字节），受对端 credit 限制。

上层 12 字节类型头复用 UART/DAP/状态：

```text
type | channel | flags | version | id(4) | length(2) | status(2)
```

- 串口数据：`UART_DATA`（本工程 64 字节分片；协议校验上限 192）。
- 串口参数：`UART_LINE` / `UART_LINE_RESULT`（fence + CDC line coding）。
- DTR/BREAK：`UART_CONTROL`。
- DAP：`DAP_COMMAND` / `DAP_RESPONSE` / `DAP_ABORT`。
- 统计/遥测：`STATUS`（12 字节，Target 每 20 ms 轮转一组）。

串口参数切换有“栅栏”：Host 在 LINE 消息里带“已接受字节数 fence”，Target 必须等物理 UART
发完 fence 之前的字节且移位寄存器排空，才应用新参数并回 `LINE_RESULT`。

### 7.3 无线 DAP

```text
Host:   USB Bulk/HID OUT -> DAP 队列 -> RF_DapTransmit
        -> [DAP_COMMAND 头 + 64B 命令] -> RF 流
Target: RF 流 -> rf_dap.c 零拷贝填入 DAP 队列 -> 每轮最多执行 4 条
        -> DAP_ProcessCommand -> SWD/JTAG -> 响应
        -> [DAP_RESPONSE 头 + 响应] -> RF 流
Host:   RF_DapPoll 校验 id 与命令回显 -> 填回 DAP 队列 -> USB IN 原端点
```

- Host 为每条命令分配单调递增 32 位 `identity`，Target 原样回显。
- `DAP_ABORT` 按 id 取消尚未执行的 Transfer/TransferBlock/原子命令组。
- Host 的 USB DAP 与 CDC 端点由同一个 USB ISR 串行化；RF 前台通过 `PFIC_SetPendingIRQ(USB_IRQn)` 唤醒 USB 所有者。

### 7.4 发现、配对、信道

```text
Target: 公共 AA 0x71764129，每 20~32 ms 发 ADVERT（UID + nonce）
Host:   静默扫描 -> 锁定 UID/nonce -> PROBE -> Target OFFER
        -> Host COMMIT -> Target ACK -> 双方切到 UID 派生私有 AA
        -> Host READY -> Target READY -> 配对完成
```

- 私有 AA：UID 前 6 字节做 FNV-1a，再用第 7/8 字节校验和验证。
- session 是 Host 生成的 64 位 nonce，每次重新发现轮换。
- 900 ms 无有效数据重新发现；Target 重新广播。
- 信道候选 `{36, 35, 34}`（2478/2472/2466 MHz）。Host 每 250 ms 采样一个信道，
  按丢包评分，48 ms 内切换；静默 300 ms 回到 0 号候选。

### 7.5 控制面

- 目标供电：冷启动默认拉高 PA4；USB 厂商请求 `0x02` 可读/设；USB 挂起或对端睡眠请求时关闭。
- USB 挂起：Host 先发睡眠请求并等 ACK，再停 RF、关目标电、把睡眠代码拷到 RAM 覆盖区、`WFI`；
  恢复后重启 RF、重新配对、恢复串口确认参数。
- DFU：Runtime DFU 收到 DETACH -> 状态 ACK -> 前台 `USB_DfuEnter()` -> 跳 `0x1FFC` 入口
  -> DFU 应用从 RAM 运行，擦写校验 `0x0~0x3C000` -> 软复位回 Host 扫描。

## 8. 硬件与片上外设

### 8.1 引脚

| 引脚 | 功能 | 说明 |
|---|---|---|
| PA0/PA1 | USB D-/D+ | `BSP_UsbDisconnect()` 会释放 |
| PA2 | UART TX | remap 1 |
| PA3 | UART RX | remap 1 |
| PA4 | LDO_EN | 高电平使能目标 3.3 V，推挽输出 |
| PA5 | SWCLK / TCK | GPIO 直驱或 SPI SCK |
| PA6 | nRESET | 开漏模拟 |
| PA7 | SWDIO / TMS | GPIO 直驱或 SPI MOSI/MISO |
| PA8 | LED2 | 高电平点亮 |
| PA9 | LED1 | 高电平点亮 |
| PA10 | TDI | JTAG |
| PA11 | TDO | JTAG，`BSP_TDO_GPIO` 可改 |

ISP 按键由硬件在复位时拉电平进入 ROM ISP，不占固件 GPIO。

### 8.2 片上功能使用

| 功能 | 用途 |
|---|---|
| USB FS 设备 | EP0 + EP1~EP6，DMA，复合设备 |
| 2.4 GHz RF Basic 2M | 私有 AA、2 Mbps、CRC、DMA、BLEB/BLEL 中断 |
| UART1 + 8B FIFO | 物理串口，最高 1.5 Mbaud |
| SPI0 两线主模式 | SWD 加速，最高 25 MHz |
| GPIOA | SWD/JTAG 位操作、LED、LDO_EN、nRESET |
| SysTick 32 位计数器 | ms/us 时基，精确处理 HCLK 切换 |
| 看门狗 WTR | 仅复位模式，前台阶段喂狗 |
| Flash 控制器 | UID 读取、DFU 擦写校验、软件复位 |
| RTC/LSI | 发现 nonce 熵；USB 挂起唤醒 |
| ADC/比较器/TMR/PWM/I2C/键盘扫描 | 未使用；DFU 入口会静默关闭 |

## 9. 代码架构与并发模型

没有 RTOS，全部是**中断 + 单前台协作**。

- 中断优先级：UART = 0（最高），USB/SysTick/BLE = 1；UART 可抢占 RF/USB。
- 中断只做时间敏感的事：UART FIFO 搬运、USB 端点完成、RF 收发完成、SysTick 计时。
- 前台 `main()` 顺序：DFU 轮询 -> BSP/LED/模式轮询 -> 镜像切换 -> USB 轮询 -> RF 链路 ->
  RF UART -> RF DAP。
- 互斥与门控：
  - `BSP_DapEnter/Leave`：整条 DAP 命令期间独占；
  - `BSP_ClockAcquire/Apply/Release`：改 HCLK 前必须取得时钟所有权；
  - `RF_BridgeClockPause/Resume`：改频前暂停射频；
  - `USB_Flush`：切换角色/睡眠前确认 USB DMA 与 SIE 已归还；
  - `BSP_stopRequested`：停止请求贯穿所有前台任务，未完成传输失败关闭。
- 所有缓冲静态分配，无堆。
- 零拷贝优先：
  - USB Bulk DMA 直接写 DAP 队列槽；
  - Local UART 包环槽直接作为 EP6 DMA 缓冲；
  - Target RF RX DMA 直接作为 DAP 命令缓冲。
- 看门狗只在前台“完成一个阶段”时喂，不在定时 ISR 里喂；前台死循环会在约 335 ms 后复位。

## 10. 必须保持的不变量（改代码前必读）

1. **Flash 分区不能随意移动**：`0x0/0x2000/0x10000/0x20000/0x3C000` 同时被链接脚本、FullRole
   合并、DFU 地址范围、CI 校验依赖。
2. **mailbox 固定在 `0x20000090`，长度 64 字节**：`startup.S` 跳过该区间，`mode_image_entry.S`
   从该地址读 magic/role。改角色状态字段必须保持 64 字节 ABI。
3. **RF 链路 v6 头 22 字节不能改布局**：`'RU'`、control(version<<4|role)、length、session(8)、
   sequence(4)、ack(4)、credit(2)。两端必须同版本。
4. **RF 消息头 12 字节不能改布局**：type/channel/flags/version/id(4)/length(2)/status(2)。
   `RF_MESSAGE_VERSION = 2`，控制记录版本 `1`，发现记录版本 `0x48`。
5. **DAP 队列所有权**：`DAP_QueueAcceptSource` 之后，请求缓冲归队列所有；`SLOT_RESPONSE`
   之前不能复用；USB IN ACK 或 RF 可靠队列接管后才能 retire。
6. **ISR 不做长操作**：RF start/stop、discover、角色切换、DFU 进入、UART 重新配置都只能在前台。
7. **改 HCLK 必须走 `BSP_ClockAcquire`**：UART 配置、RF 启动、DAP 命令都依赖该门控；
   不能在 ISR 中改频。
8. **看门狗不能周期性在 ISR 喂**：只能在完成一个前台阶段后喂，否则会掩盖死循环。
9. **每个角色镜像只链接自己的模块**：Local 没有 RF；Host 没有本地 DAP 引擎/物理 UART；
   Target 没有 USB 栈。新增源文件时先看 `app_config.h` 的条件宏。
10. **串口包环语义**：Local 的 4x64 字节槽是“USB 包边界”，不能改成普通字节环而不改 EP6 DMA 逻辑。
11. **DFU 运行在 RAM**：DFU 代码、栈、数据都在 `0x20000000` 起；不要给它增加超出 8 KB 的
    Flash 载荷或破坏 `0x1FFC` 入口指针。
12. **HardFault 处理**：释放目标引脚并两灯常亮，随后看门狗复位进入 DFU；不要改成静默重启。
13. **Host 绑定区 `0x1E000~0x20000` 只能由 `cfg/bind_store.c` 访问**：Host/Dongle 链接断言与
    CI 都会拦截越过 `0x1E000` 的镜像；`0x3F000` 以上是芯片配置区（UID 在 `0x3F018`），任何代码都不得写入。

## 11. 常见修改指南

### 改引脚

- 修改 `bsp/board.h` 的 `PIN_*` / `BSP_TDO_GPIO`。
- 检查 `bsp/board.c` 的 `BSP_ReleaseTarget()`、`SWJ_DebugPinsSetup()`、`UART_Init()` 的 remap。
- 检查 `dap/debug_pins.h` 的输入/输出/上拉集合。
- 如果改了 UART 引脚，确认 `BSP_UART_TX_REMAP` / `BSP_UART_RX_REMAP` 与 CH572 复用表一致。

### 改串口波特率/帧格式

- 修改 `app_config.h` 的 `UART_MAX_BAUD`、`UART_RX_SIZE`、`UART_TX_SIZE`。
- 修改 `uart/uart_line.c` 的时钟候选与例外表（当前 100/60/40 MHz，921600 用 75 MHz）。
- 无线串口还要同步 `rf/rf_uart.c` 的分片上限 `RF_UART_DATA_SIZE` 与 `rf_message.h` 的
  `RF_MESSAGE_UART_MAX_DATA`。

### 增加 CMSIS-DAP 命令

- 在 `dap/dap.c` 的 `DAP_ProcessStandardCommand()` 中加 case，并实现处理函数。
- 若命令需要长执行，保持“整条命令”语义；不要跨 `BSP_DapEnter/Leave` 拆分。
- 无线 DAP 无需额外改动：RF 层按 64 字节命令透传，响应按 id 匹配。
- 新增响应长度不能超过 `DAP_PACKET_SIZE`（64）。

### 调整 RF 参数

- 产品参数在 `app_config.h`：默认信道、公共 AA、DMA 大小、TX 功率编码、队列大小、帧上限。
- 物理层固定 Basic 2M、CRC、白化在 `rf/core/rf_basic_2m.c` 与 `rf/rf_bridge.c`。
- 信道候选/采样时间在 `rf/rf_policy.h`。
- 修改任何链路字段必须双端同步，并递增 `RF_LINK_VERSION` / `RF_MESSAGE_VERSION`。

### 修改 USB 描述符

- 接口/端点顺序在 `usb/usb_descriptors.c`，端点号宏在 `usb/usb_descriptors.h`。
- 改端点后必须同步 `usb_device.c` 的 `USB_Init()`、`USB_ResetEndpoints()` 和 ISR 分支。
- 改接口数量会影响 DFU 的 `USB_DFU_RUNTIME_INTERFACE`、MS OS 描述符和 `USB_CONFIG_LENGTH`。

### 修改启动/模式策略

- 窗口时长：`app_config.h` 的 `APP_BOOT_USB_WINDOW_MS`、`mode/app_mode.h` 的
  `APP_MODE_PAIR_WINDOW_MS` / `APP_MODE_PRE_DISCONNECT_MS` / `APP_MODE_TARGET_RETURN_MS`。
- 决策逻辑：`mode/app_mode.c` 的 `MODE_Decide()`。
- 切换执行：`mode/mode_image.c` 的 `MODE_ImageRequest()` / `MODE_ImagePoll()`。
- 改角色地址必须同步 `flash_layout.h`、五个链接脚本、`fullrole.ld` 和 CI 校验。

## 12. 代码风格

- 使用 `firmware/.clang-format`：LLVM 基础、4 空格、100 列、Attach 大括号、大小写标签缩进。
- 声明对齐、赋值对齐、宏对齐已启用；`SortIncludes: Never`。
- 命名习惯：
  - 模块前缀：`BSP_`、`USB_`、`DAP_`、`RF_`、`MODE_`、`UART_`、`SWJ_`、`DFU_`；
  - 宏与编译开关全大写；
  - 静态函数小写模块前缀，例如 `RF_LinkBuild`。
- 不使用动态内存；不使用阻塞式长延时（除启动/DFU 受控流程）。
- 注释解释“为什么”和硬件约束，不重复代码表面含义。
- 修改汇编/链接脚本/启动代码时，优先保持现有顺序和 fence；这些地方对时序敏感。

## 13. 调试与排障

| 现象 | 优先检查 |
|---|---|
| USB 不枚举 | `USB_Init()`、描述符长度、EP DMA 地址、`BSP_UsbDisconnect()` |
| 串口无数据 | DTR 是否置位（Host 要求 DTR=1 才转发）、波特率栅栏是否完成、TX 环是否满 |
| DAP 连不上 | `SWJ_Port()` 后端选择、目标供电、nRESET、SWD 线路恢复 `SWD_Recover()` |
| 无线配不上 | Target 是否在广播、公共 AA/信道、USB 是否已配置、`MODE_AppModeRfCommitAllowed()` |
| 无线频繁重连 | 900 ms 租约、信道评分、RF 重传计数、CRC 错误计数 |
| 升级后变砖 | 冷启动路由是否检测到合法 JAL、DFU 入口 `0x1FFC`、FullRole 拼接地址 |
| 看门狗复位 | 前台是否长时间阻塞、是否在 ISR 里喂狗、硬件等待是否有界 |

可观测计数器：

- `RF_txPackets`、`RF_rxPackets`、`RF_retries`、`RF_crcErrors`、`RF_frameErrors`、
  `RF_duplicates`、`RF_dmaOverflows`、`RF_hardwareTimeouts`（`rf/rf_bridge.c`）。
- `UART_stats`：`rx_bytes`、`tx_bytes`、`rx_overflow`、`line_errors`、`max_rx_high_water`
  （`uart/uart_io.c`）。
- `rf_uart_state_t`：`errors`、`pressure`、`fault`、`last_status`（`rf/rf_uart.c`）。

LED 语义：

- Host：未绑定两灯灭；已绑定但目标不在线为 1 s 单脉冲；已配对每 2 s 双闪 200 ms。
- Target：未配对 1 s 交替闪；已配对灭；DAP 活动显示 connect/running；串口活动脉冲反相。
- Local：DAP connect/running + TX/RX 活动脉冲。
- HardFault：两灯常亮。

## 14. 推荐阅读顺序

1. `README_zh.md`（产品行为与性能）
2. `firmware/src/app_config.h`（功能裁剪与产品参数）
3. `firmware/src/flash_layout.h`（地址 ABI）
4. `firmware/src/main.c`（初始化与前台调度）
5. `firmware/src/mode/app_mode.c` + `mode/mode_image.c`（模式决策与热切换）
6. `firmware/src/usb/usb_descriptors.c` + `usb/usb_device.c`（USB 复合设备与端点）
7. `firmware/src/dap/dap.c` + `dap/dap_queue.c` + `dap/swj.c`（DAP 协议与后端）
8. `firmware/src/rf/rf_bridge.c` + `rf/rf_link.c` + `rf/rf_message.c`
   + `rf/rf_uart.c` + `rf/rf_dap.c` / `rf/rf_dap_host.c`（无线链路与业务复用）
9. `firmware/src/uart/uart_io.c` + `uart/uart_line.c`（物理串口）
10. `firmware/src/dfu/`（升级链）
11. `firmware/projects/ld/*.ld` 与 `.github/scripts/verify_build.py`（布局校验）

## 15. 目标绑定（Host/Dongle）

- 记录：`src/cfg/bind_store.c`，32 字节记录（magic/version/flags/uid/seq/crc）在两扇区
  `0x1E000` / `0x1F000` 间 ping-pong；写前擦旧扇区、写 seq+1、回读校验，失败时保留旧记录。
- 启动：`MODE_ImageBoot()` 在模式控制器初始化前调用 `BIND_Load()`，填充
  `MODE_mailbox.binding.peer_uid/peer_aa/bound`；`MODE_AppModeSetBound()` 让 Host 直接 READY，
  目标不在线时无限等待，不降级 Local，也不连接其他 UID。
- 配对：`RF_DiscoveryComplete()` 在 Host 分支把 peer UID 交给 `BIND_RequestSave()`；
  Flash 擦写只在主循环 `BIND_Poll()` 中执行（位于 `RF_DapPoll` 之后），先暂停射频并确认
  `RF_DapIdle()`，ISR 只登记请求。
- 解绑 / scan-only：`USB 0x03 OUT` 清记录并 `RF_BridgeForget()`；`USB 0x04 OUT` 只进入
  scan-only。scan-only 记录 ADVERT、不发 PROBE、不自动配对，直到 BIND 或复位。
- 上位机协议与脚本：`doc/01_bind_guide/binding_zh.md`、`tool/01_airdap_bind/airdap_bind.py`。
- 升级固件（DFU 覆盖 `0x0~0x3C000`）可能擦掉绑定记录，升级后需重新配对或重新 bind。

## 15. 术语表

| 术语 | 含义 |
|---|---|
| Local | 纯有线角色，USB 直连本机 UART/DAP |
| Host | 插 PC 的无线端，负责 USB <-> RF 转发 |
| Target | 接目标板的无线端，负责 RF <-> 物理 UART/SWD/JTAG |
| Dongle | 只有 Host 的 TinyDongle 固件 |
| FullRole | DFU+Local+Host+Target 合并后的整片镜像 |
| bank | 一个角色镜像占用的 Flash 区域 |
| mailbox | `0x20000090` 起的 64 字节角色切换共享内存 |
| session | 一次配对会话的 64 位 nonce |
| AA | RF access address；公共 `0x71764129`，数据用 UID 派生私有 AA |
| ADVERT | Target 在公共 AA 上的广播 |
| PROBE/OFFER/COMMIT/ACK/READY | 四步发现握手及其确认 |
| fence | 串口参数生效前的“已接受字节数”栅栏 |
| credit | 接收方通告的剩余 RF 接收空间 |
| RD/RC/RP | 发现记录 / 信道记录 / 对端睡眠记录 |
| blocked/stopRequested | 前台停止请求，贯穿所有数据所有者 |

---

维护本文件时请保持“结论 + 文件位置 + 约束”的结构；不要把临时调试步骤写进来。
