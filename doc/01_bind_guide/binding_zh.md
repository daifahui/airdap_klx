# AirDAP 目标绑定与上位机协议

本文说明 Host/Dongle 侧的目标绑定行为、Flash 记录和 USB 厂商请求。Target 镜像行为不变。

## 1. 行为语义

| 状态 | 上电/插 USB 后的行为 |
|---|---|
| 未绑定（Flash 无有效记录） | 沿用 2 秒窗口：连第一个听到的 Target，配对成功后把该 UID 写入 Host Flash |
| 已绑定 | 只接受该 UID；目标不在线时保持 Host 无限等待，不降级 Local、不连接其他 Target |
| 已绑定且无 USB | 仍是 Host，继续等待绑定的 Target |
| scan-only（forget 或 0x04 OUT 后） | 记录所有 ADVERT 到发现列表，不发 PROBE、不自动配对；BIND 或复位后退出 |

- 绑定记录保存在 Host bank 末尾的两个 4 KB 扇区：`0x1E000`（A）、`0x1F000`（B）。
- 记录长度 32 字节：

  | 偏移 | 字段 | 说明 |
  |---|---|---|
  | 0 | magic | `0x314E4442`（小端 "BDN1"） |
  | 4 | version | `1` |
  | 6 | flags | bit0 = bound |
  | 8 | uid[8] | Target 的 8 字节 UID |
  | 16 | seq | 单调递增序号 |
  | 20 | crc | 前 20 字节的 CRC-32 |
  | 24 | reserved | 0 |

- 写入流程：擦除较旧扇区 -> 写入 `seq+1` -> 关闭 Flash 控制器 -> XIP 回读校验。
  写失败时保留另一扇区中的旧记录；擦写期间断电，重启后仍能读到旧或新记录之一。
- 擦写只在主循环的 `BIND_Poll()` 执行：先暂停射频（`RF_BridgeClockPause`），
  确认 `RF_DapIdle()`（没有进行中的 DAP 命令），整段关中断后在 RAM 中完成。
  USB 中断只登记请求，不直接写 Flash。

## 2. USB 厂商请求

所有请求都是 `bmRequestType` 为 `0xC0`（IN）或 `0x40`（OUT）、`wValue = 0`。
不匹配的 `bmRequestType`/`wValue`/`wIndex`/`wLength` 一律 STALL。

### 0x03 绑定状态 / 设置绑定

| 方向 | wIndex | wLength | 含义 |
|---|---|---|---|
| IN | 0 | 32 | 返回绑定状态 |
| OUT | 0 | 8 | 绑定指定 UID，状态 ACK 后提交 |
| OUT | 0 | 0 | 清除绑定并进入 scan-only |

IN 的 32 字节：

| 偏移 | 内容 |
|---|---|
| 0 | flags：bit0=bound，bit1=paired（RF 会话已建立），bit2=scan-only |
| 1~7 | 0 |
| 8~15 | 绑定 UID；未绑定时全 0 |
| 16~23 | 本机 UID |
| 24~31 | 0 |

OUT 8 字节：目标 UID。前 6 字节是工厂 UID，第 7/8 字节是三个 16 位小端字之和的低 16 位；
校验不通过或全 0 时 STALL。数据阶段先暂存，状态 ACK 后交给前台保存；写成功后 Host 立即生效
（必要时重建 RF 会话）。

### 0x04 发现列表 / 进入 scan-only

| 方向 | wIndex | wLength | 含义 |
|---|---|---|---|
| IN | 页号 0~3 | 32 | 读取一页发现列表 |
| OUT | 0 | 0 | 进入 scan-only，不清除绑定 |

每页 32 字节，最多 2 项：

| 偏移 | 内容 |
|---|---|
| 0 | 有效项总数（0~8） |
| 1 | 页号 |
| 2 | 本页项数（0~2） |
| 3 | 0 |
| 4 + 12*n | uid[8] |
| 12 + 12*n | last_seen，设备毫秒时基，小端 u32 |
| 28~31 | 0 |

发现列表在 UID 过滤之前记录，因此已绑定时也能看到其他 Target。列表最多 8 项，
超出后淘汰最久未出现的项。

### 退出无线

沿用原有 `0x01` 模式请求：`0x40`、`wValue = 1`（Local）、`wIndex = 0`、无数据。
只在本次运行生效；断电后若 Flash 中仍有绑定，则恢复 Host 并只连接该 Target。

## 3. 工具

脚本位于 `tool/01_airdap_bind/airdap_bind.py`，依赖 pyusb：

```bash
pip install pyusb
python3 tool/01_airdap_bind/airdap_bind.py list
python3 tool/01_airdap_bind/airdap_bind.py --serial 0123456789ABCDEF show
python3 tool/01_airdap_bind/airdap_bind.py --serial 0123456789ABCDEF bind 0011223344556677
python3 tool/01_airdap_bind/airdap_bind.py --serial 0123456789ABCDEF forget
python3 tool/01_airdap_bind/airdap_bind.py --serial 0123456789ABCDEF peers
python3 tool/01_airdap_bind/airdap_bind.py --serial 0123456789ABCDEF exit-wireless
```

Windows 需要为设备安装 WinUSB/libusb 驱动（可用 Zadig），否则 pyusb 无法打开设备。
只有一台 Host 时可以省略 `--serial`；多台时必须指定，`list` 会显示每台的 USB 序列号（即本机 UID）。

## 4. 注意事项

- 首次配对先给 Target 上电，再给 Host 上电或插 USB；否则 2 秒窗口结束后 Host 会进入 Local，
  而 Local 没有 RF，无法再完成配对。
- `forget` 会清除 Flash 记录并进入 scan-only；此时能看到所有 Target，但不会自动连接。
- DFU 升级范围是 `0x0~0x3C000`，包含绑定扇区，升级后可能需要重新配对或 bind。
- 两台 Host 绑定同一 UID 时，后到者会一直等待；`show` 中 paired=0 表示当前未建立 RF 会话。
- 禁止写 `0x3F000` 及以上的芯片配置区（UID 位于 `0x3F018`）。
