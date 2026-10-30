cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Motor_HAL_REBULD)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})
# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# =====================================================================
# 手写分层（本文件 CubeMX 只生成一次，之后随便改）
#
# 依赖方向（只看 #include）：
#     App → Service → Bsp → Osal → CubeMX(Core/ + Drivers/)
#     Config 横切：所有层都可读它，它不依赖任何人
#
# 层间约束不靠"自觉"，靠 include 路径：
#   下层拿不到上层头文件目录 → 下层一旦 #include 上层，编译期直接报错。
#
# 为什么用 OBJECT 库而不是 STATIC 库：
#   静态库对链接顺序敏感、容易循环依赖、要靠 --start-group 才能过；
#   OBJECT 库把所有 .o 一次性交给同一条链接命令，且 --gc-sections 已开，
#   不会带进没用到的代码。
#   ⚠️ 每个 OBJECT 库在链接列表里只能出现一次，否则报重复符号。
#
# 手写代码全部收在下面五个 OBJECT 库里；可执行文件只负责"链接"它们。
# =====================================================================

# stm32cubemx 是 CubeMX 生成的 INTERFACE 库：只有 include 路径 + 宏定义
# (STM32F407xx / USE_HAL_DRIVER / DEBUG)，没有任何 .o 文件。
# 挂到各层上既能拿到这些宏，又不会把源文件重复编译。
# ⚠️ 不要改成去链接 STM32_Drivers —— 那是有 .o 的 OBJECT 库，链两次就是重复符号。

# ---- Config：板级事实与参数（谁都能读，它谁都不依赖）----
add_library(cfg OBJECT
    Config/Src/cfg_board.c
)
target_include_directories(cfg PUBLIC
    ${CMAKE_SOURCE_DIR}/Config/Inc
)

# ---- Osal：全工程唯一允许直接调 HAL_Delay / HAL_GetTick 的地方 ----
add_library(osal OBJECT
    Osal/Src/osal.c
)
target_include_directories(osal PUBLIC
    ${CMAKE_SOURCE_DIR}/Osal/Inc
)
target_link_libraries(osal PUBLIC cfg stm32cubemx)

# ---- Bsp：唯一懂"这块板子"的一层（唯一允许 include tim.h/usart.h/gpio.h）----
add_library(bsp OBJECT
    Bsp/Src/bsp_fault.c
    Bsp/Src/bsp_motor_pulse.c
    Bsp/Src/bsp_rc.c
    Bsp/Src/bsp_uart.c
)
target_include_directories(bsp PUBLIC
    ${CMAKE_SOURCE_DIR}/Bsp/Inc
)
target_link_libraries(bsp PUBLIC osal)

# ---- Service：与业务无关的工具箱（帧协议 / 日志 / i-BUS 遥控解码）----
add_library(service OBJECT
    Service/Src/protocol.c
    Service/Src/log.c
    Service/Src/ibus.c
)
target_include_directories(service PUBLIC
    ${CMAKE_SOURCE_DIR}/Service/Inc
)
target_link_libraries(service PUBLIC bsp)

# ---- App：唯一知道"这是一台表型检测车"的一层 ----
add_library(app OBJECT
    App/Src/app_init.c
    App/Src/app_rc.c
    App/Src/motion.c
    App/Src/rc_control.c
)
target_include_directories(app PUBLIC
    ${CMAKE_SOURCE_DIR}/App/Inc
)
target_link_libraries(app PUBLIC service)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# 手写源文件不在这里列：它们在上面五个 OBJECT 库里。
# 以后若出现"不属于任何层"的一次性代码，再往这里加：
#   target_sources(${CMAKE_PROJECT_NAME} PRIVATE xxx.c)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    App/Inc
    Config/Inc
)
# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    cfg
    osal
    bsp
    service
    app

    # Add user defined libraries
)
