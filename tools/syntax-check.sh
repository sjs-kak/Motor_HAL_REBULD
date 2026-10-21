#!/usr/bin/env bash
# 语法检查（不链接）：绕开 cmake/ninja，直接调 arm-none-eabi-gcc
set -u
# 工程根目录由脚本自身位置推导（tools/ 的上一级），避免换目录后路径失效
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
GCC="C:/Users/Lenovo/AppData/Local/stm32cube/bundles/gnu-tools-for-stm32/14.3.1+st.2/bin/arm-none-eabi-gcc.exe"
# 工具链版本会随 STM32Cube 扩展升级；硬编码路径失效时自动在 bundles 里找最新的一份
if [ ! -x "$GCC" ]; then
  GCC="$(ls -d "$LOCALAPPDATA"/stm32cube/bundles/gnu-tools-for-stm32/*/bin/arm-none-eabi-gcc.exe 2>/dev/null | sort -V | tail -1)"
fi
if [ -z "${GCC:-}" ] || [ ! -x "$GCC" ]; then
  echo "arm-none-eabi-gcc not found (install the STM32Cube VS Code extension)" >&2
  exit 1
fi

INC=(
  "-I$ROOT/App/Inc"
  "-I$ROOT/Bsp/Inc"
  "-I$ROOT/Config/Inc"
  "-I$ROOT/Osal/Inc"
  "-I$ROOT/Service/Inc"
  "-I$ROOT/Core/Inc"
  "-I$ROOT/Drivers/CMSIS/Device/ST/STM32F4xx/Include"
  "-I$ROOT/Drivers/CMSIS/Include"
  "-I$ROOT/Drivers/STM32F4xx_HAL_Driver/Inc"
  "-I$ROOT/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy"
)

SRC=(
  "App/Src/motion.c"
  "App/Src/app_init.c"
  "Bsp/Src/bsp_motor_pulse.c"
  "Bsp/Src/bsp_uart.c"
  "Service/Src/protocol.c"
  "Core/Src/main.c"
)

fail=0
for f in "${SRC[@]}"; do
  echo "===== $f"
  "$GCC" -fsyntax-only -std=gnu11 -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard \
    -DDEBUG -DSTM32F407xx -DUSE_HAL_DRIVER \
    -Wall -Wextra \
    "${INC[@]}" "$ROOT/$f" 2>&1 | head -40
  if [ "${PIPESTATUS[0]}" -ne 0 ]; then fail=1; fi
done
echo "-----"
if [ "$fail" -ne 0 ]; then echo "RESULT: FAIL"; else echo "RESULT: OK"; fi
