# AGENTS.md

只写硬约束与路由；描述看 `README.md`/`docs/`。

## 不要碰

- `Drivers/` 不改；`Core/` 只写 `USER CODE BEGIN/END` 段内；`build/` 是产物目录。
- 手写层禁止 include `main.h`/`tim.h`/`usart.h`/`gpio.h`/`FreeRTOS.h`，唯一例外 `Bsp/Inc/mx_board.h`。
- 依赖只能向下 `App→Service→Bsp→Osal→CubeMX`，`Config/` 横切。反向 include 编译即失败。

## 必须遵守

- 新增 `.c` 加进 `CMakeLists.txt` 该层 `add_library(<层> OBJECT ...)`，漏了静默丢弃。
- 先 `Bsp_MotorPulse_StartAll()` 再 `SetFreq()`；同侧永久同频；停车 `SetFreq(ch,0)`。
- ISR（`Protocol_FeedByte()`、`bsp_rc`）禁 `LOG()`、`Osal_DelayMs()`、阻塞调用。
- 接口以 `*/Inc/*.h` 为准（README §7 滞后：缺 `Motion_*Abort/Tick`、`App_Rc_*`）。
- 改 `.ioc` 时钟后同步 `cfg_motor.h` 的 `CFG_TIM*_CLK_HZ`，否则转速静默错。
- 未定位故障：软件已停但 TIM1/TIM2 仍出脉冲 → 上电/下载前垫起四轮。

## 怎么构建

工具链不在 PATH，禁止直接 `ninja`/`cmake`。唯一入口：

- `powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1`（`-Clean` 全量）
- `bash tools/syntax-check.sh` 须 `RESULT: OK`，但只查 6 个文件

验证靠上板（`.map`、示波器、调试器）；`.ps1` 必须 UTF-8 带 BOM。

## 深入看这里

- 架构、13 条契约、常见坑、P1–P4 → `README.md` §8、§8.5、§5、§9.1
- 卡点/串口/调试器 → `docs/HANDOVER-2026-10-21.md` §4–§6
