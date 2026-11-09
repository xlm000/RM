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
set(CMAKE_PROJECT_NAME PHOENIX_2027_Infantry_guodong_up)

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

# 基础库根目录（本项目以源码目录形式放在工程下，作为 git submodule 时默认同名）
set(PHOENIX_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/phoenix_embedded_base_code)

# 用户代码根目录
set(USER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/UserFile)

# ============================================================================
# 2. phoenix_embedded_base_code 基础库源文件
#    按需增删；用不到的模块注释掉即可（注释掉的模块同时也别加它的头文件路径）
# ============================================================================
set(PHOENIX_LIB_SRC
        # ---- Algorithm 算法层 ----
        ${PHOENIX_LIB_DIR}/Algorithm/Chassis_calc/alg_chassis_calc.c
        ${PHOENIX_LIB_DIR}/Algorithm/Crc/alg_crc.c
        ${PHOENIX_LIB_DIR}/Algorithm/Fliters/alg_fliter.c
        ${PHOENIX_LIB_DIR}/Algorithm/Kalman/Kalman.c
        ${PHOENIX_LIB_DIR}/Algorithm/Madgwick/MadgwickAHRS.c
        ${PHOENIX_LIB_DIR}/Algorithm/Mahony/alg_mahony.c
        ${PHOENIX_LIB_DIR}/Algorithm/Pid/alg_pid.c
        ${PHOENIX_LIB_DIR}/Algorithm/SMC/alg_smc.c
        # ${PHOENIX_LIB_DIR}/Algorithm/LQR/alg_lqr.c

        # ---- Application 应用层 ----
        ${PHOENIX_LIB_DIR}/Application/app_ins_task.c

        # ---- Bsp 板级支持层 ----
        ${PHOENIX_LIB_DIR}/Bsp/Can/bsp_can.c
        ${PHOENIX_LIB_DIR}/Bsp/DWT/bsp_dwt.c
        # ${PHOENIX_LIB_DIR}/Bsp/Exti/bsp_exti.c
        ${PHOENIX_LIB_DIR}/Bsp/Fdcan/bsp_fdcan.c
        ${PHOENIX_LIB_DIR}/Bsp/GPIO/bsp_gpio.c
        ${PHOENIX_LIB_DIR}/Bsp/I2c/bsp_iic.c
        ${PHOENIX_LIB_DIR}/Bsp/Log/bsp_log.c
        ${PHOENIX_LIB_DIR}/Bsp/PWM/bsp_pwm.c
        ${PHOENIX_LIB_DIR}/Bsp/SPI/bsp_spi.c
        # ${PHOENIX_LIB_DIR}/Bsp/Tim/bsp_tim.c
        ${PHOENIX_LIB_DIR}/Bsp/Uart/bsp_uart.c

        # ---- Module 设备模块层 ----
        ${PHOENIX_LIB_DIR}/Module/BMI088/dev_bmi088.c
        ${PHOENIX_LIB_DIR}/Module/Buzzer/bsp_buzzer.c
        # ${PHOENIX_LIB_DIR}/Module/Buzzer/music_sheet.c        # 用到乐谱播放时打开
        # ${PHOENIX_LIB_DIR}/Module/Com_System/Com_System.c
        # ${PHOENIX_LIB_DIR}/Module/Com_System/Queue.c
        ${PHOENIX_LIB_DIR}/Module/Daemon/dev_daemon.c
        ${PHOENIX_LIB_DIR}/Module/Dr16/dev_dr16.c
        ${PHOENIX_LIB_DIR}/Module/Et08a/dev_et08a.c
        ${PHOENIX_LIB_DIR}/Module/Ist8310/dev_ist_8310.c
        ${PHOENIX_LIB_DIR}/Module/Led/dev_led.c
        ${PHOENIX_LIB_DIR}/Module/Minipc/dev_minipc.c
        ${PHOENIX_LIB_DIR}/Module/Motor/Bm/dev_motor_bm.c
        ${PHOENIX_LIB_DIR}/Module/Motor/Dji/dev_motor_dji.c
        ${PHOENIX_LIB_DIR}/Module/Motor/Dm/dev_motor_dm.c
        ${PHOENIX_LIB_DIR}/Module/Motor/Lk/dev_motor_lk.c
        ${PHOENIX_LIB_DIR}/Module/Motor/RobStride/dev_motor_rs.c
        ${PHOENIX_LIB_DIR}/Module/Motor/Unitree/dev_motor_unitree.c
        ${PHOENIX_LIB_DIR}/Module/Referee/dev_referee.c
        # ${PHOENIX_LIB_DIR}/Module/Referee/dev_referee_ui.c
        ${PHOENIX_LIB_DIR}/Module/Servo/dev_servo.c
        # ${PHOENIX_LIB_DIR}/Module/SEGGER/dev_segger.c          # J-Link RTT 的额外封装，一般用不到

        # ---- J-Link RTT 日志（bsp_log.c 依赖，必须保留）----
        ${PHOENIX_LIB_DIR}/Module/SEGGER/Jlink_RTT/SEGGER_RTT.c
        ${PHOENIX_LIB_DIR}/Module/SEGGER/Jlink_RTT/SEGGER_RTT_printf.c
)

# ============================================================================
# 3. phoenix_embedded_base_code 基础库头文件路径
# ============================================================================
set(PHOENIX_LIB_INC
        # ---- Algorithm 算法层 ----
        ${PHOENIX_LIB_DIR}/Algorithm/Chassis_calc
        ${PHOENIX_LIB_DIR}/Algorithm/Crc
        ${PHOENIX_LIB_DIR}/Algorithm/Fliters
        ${PHOENIX_LIB_DIR}/Algorithm/Kalman
        ${PHOENIX_LIB_DIR}/Algorithm/Madgwick
        ${PHOENIX_LIB_DIR}/Algorithm/Mahony
        ${PHOENIX_LIB_DIR}/Algorithm/Pid
        ${PHOENIX_LIB_DIR}/Algorithm/SMC
        # ${PHOENIX_LIB_DIR}/Algorithm/LQR

        # ---- Application 应用层 ----
        ${PHOENIX_LIB_DIR}/Application

        # ---- Bsp 板级支持层 ----
        ${PHOENIX_LIB_DIR}/Bsp/Can
        ${PHOENIX_LIB_DIR}/Bsp/DWT
        # ${PHOENIX_LIB_DIR}/Bsp/Exti
        ${PHOENIX_LIB_DIR}/Bsp/Fdcan
        ${PHOENIX_LIB_DIR}/Bsp/GPIO
        ${PHOENIX_LIB_DIR}/Bsp/I2c
        ${PHOENIX_LIB_DIR}/Bsp/Log
        ${PHOENIX_LIB_DIR}/Bsp/PWM
        ${PHOENIX_LIB_DIR}/Bsp/SPI
        # ${PHOENIX_LIB_DIR}/Bsp/Tim
        ${PHOENIX_LIB_DIR}/Bsp/Uart

        # ---- Module 设备模块层 ----
        ${PHOENIX_LIB_DIR}/Module/BMI088
        ${PHOENIX_LIB_DIR}/Module/Buzzer
        # ${PHOENIX_LIB_DIR}/Module/Com_System
        ${PHOENIX_LIB_DIR}/Module/Daemon
        ${PHOENIX_LIB_DIR}/Module/Dr16
        ${PHOENIX_LIB_DIR}/Module/Et08a
        ${PHOENIX_LIB_DIR}/Module/Ist8310
        ${PHOENIX_LIB_DIR}/Module/Led
        ${PHOENIX_LIB_DIR}/Module/Minipc
        ${PHOENIX_LIB_DIR}/Module/Motor/Bm
        ${PHOENIX_LIB_DIR}/Module/Motor/Dji
        ${PHOENIX_LIB_DIR}/Module/Motor/Dm
        ${PHOENIX_LIB_DIR}/Module/Motor/Lk
        ${PHOENIX_LIB_DIR}/Module/Motor/RobStride
        ${PHOENIX_LIB_DIR}/Module/Motor/Unitree
        ${PHOENIX_LIB_DIR}/Module/Referee
        ${PHOENIX_LIB_DIR}/Module/Servo
        ${PHOENIX_LIB_DIR}/Module/SEGGER
        ${PHOENIX_LIB_DIR}/Module/SEGGER/Jlink_RTT
)

# ============================================================================
# 4. UserFile 用户代码
#    约定：自己的代码全部放在 UserFile/ 下，按 Algorithm/Application/Bsp/Module 分层，
#    新增 .c/.h 后直接重新 build 即可，不用再回来改这个文件。
#    （CONFIGURE_DEPENDS 会让 ninja 在文件增删后自动重新 configure）
# ============================================================================
file(GLOB_RECURSE USER_SRC CONFIGURE_DEPENDS
        ${USER_DIR}/*.c
)

# UserFile 下所有子目录都加入头文件搜索路径，直接 #include "xxx.h" 即可
file(GLOB_RECURSE USER_ENTRIES CONFIGURE_DEPENDS LIST_DIRECTORIES true ${USER_DIR}/*)
set(USER_INC ${USER_DIR})
foreach(_entry ${USER_ENTRIES})
    if(IS_DIRECTORY "${_entry}")
        list(APPEND USER_INC "${_entry}")
    endif()
endforeach()

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
        ${PHOENIX_LIB_SRC}
        ${USER_SRC}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        ${PHOENIX_LIB_INC}
        ${USER_INC}

        ${CMAKE_CURRENT_SOURCE_DIR}/USB_DEVICE/App
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

    # Add user defined libraries
)
