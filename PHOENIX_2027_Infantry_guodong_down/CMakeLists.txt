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
set(CMAKE_PROJECT_NAME PHOENIX_2027_Infantry_guodong_down)

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

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
        phoenix_embedded_base_code/Algorithm/Chassis_calc/alg_chassis_calc.c
        phoenix_embedded_base_code/Algorithm/Crc/alg_crc.c
        phoenix_embedded_base_code/Algorithm/Fliters/alg_fliter.c
        phoenix_embedded_base_code/Algorithm/Kalman/Kalman.c
        phoenix_embedded_base_code/Algorithm/Mahony/alg_mahony.c
        phoenix_embedded_base_code/Algorithm/Pid/alg_pid.c
        phoenix_embedded_base_code/Algorithm/SMC/alg_smc.c

        phoenix_embedded_base_code/Application/app_ins_task.c

        phoenix_embedded_base_code/Bsp/Can/bsp_can.c
        phoenix_embedded_base_code/Bsp/DWT/bsp_dwt.c
        # phoenix_embedded_base_code/Bsp/Exti/bsp_exti.c
        phoenix_embedded_base_code/Bsp/Fdcan/bsp_fdcan.c
        phoenix_embedded_base_code/Bsp/GPIO/bsp_gpio.c
        phoenix_embedded_base_code/Bsp/I2c/bsp_iic.c
        phoenix_embedded_base_code/Bsp/Log/bsp_log.c
        phoenix_embedded_base_code/Bsp/PWM/bsp_pwm.c
        phoenix_embedded_base_code/Bsp/SPI/bsp_spi.c
        # phoenix_embedded_base_code/Bsp/Tim/bsp_tim.c
        phoenix_embedded_base_code/Bsp/Uart/bsp_uart.c

        phoenix_embedded_base_code/Module/BMI088/dev_bmi088.c
        phoenix_embedded_base_code/Module/Buzzer/bsp_buzzer.c
        # phoenix_embedded_base_code/Module/Com_System/Com_System.c
        # phoenix_embedded_base_code/Module/Com_System/Queue.c
        phoenix_embedded_base_code/Module/Daemon/dev_daemon.c
        phoenix_embedded_base_code/Module/Dr16/dev_dr16.c
        phoenix_embedded_base_code/Module/Et08a/dev_et08a.c
        phoenix_embedded_base_code/Module/Ist8310/dev_ist_8310.c
        phoenix_embedded_base_code/Module/Led/dev_led.c
        # phoenix_embedded_base_code/Module/Minipc/dev_minipc.c
        phoenix_embedded_base_code/Module/Motor/Bm/dev_motor_bm.c
        phoenix_embedded_base_code/Module/Motor/Dji/dev_motor_dji.c
        phoenix_embedded_base_code/Module/Motor/Dm/dev_motor_dm.c
        phoenix_embedded_base_code/Module/Motor/Lk/dev_motor_lk.c
        phoenix_embedded_base_code/Module/Motor/RobStride/dev_motor_rs.c
        phoenix_embedded_base_code/Module/Motor/Unitree/dev_motor_unitree.c
        phoenix_embedded_base_code/Module/Referee/dev_referee.c
        # phoenix_embedded_base_code/Module/Referee/dev_referee_ui.c
        # phoenix_embedded_base_code/Module/SEGGER/dev_segger.c
        # phoenix_embedded_base_code/Module/SEGGER/Jlink_RTT/SEGGER_RTT.c
        phoenix_embedded_base_code/Module/Servo/dev_servo.c

)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        DSP_LIB/Include

        phoenix_embedded_base_code/Algorithm/Chassis_calc
        phoenix_embedded_base_code/Algorithm/Crc
        phoenix_embedded_base_code/Algorithm/Fliters
        phoenix_embedded_base_code/Algorithm/Kalman
        phoenix_embedded_base_code/Algorithm/Madgwick
        phoenix_embedded_base_code/Algorithm/Mahony
        phoenix_embedded_base_code/Algorithm/Pid
        phoenix_embedded_base_code/Algorithm/SMC

        phoenix_embedded_base_code/Application

        phoenix_embedded_base_code/Bsp/Can
        phoenix_embedded_base_code/Bsp/DWT
        # phoenix_embedded_base_code/Bsp/Exti
        phoenix_embedded_base_code/Bsp/Fdcan
        phoenix_embedded_base_code/Bsp/GPIO
        phoenix_embedded_base_code/Bsp/I2c
        phoenix_embedded_base_code/Bsp/Log
        phoenix_embedded_base_code/Bsp/PWM
        phoenix_embedded_base_code/Bsp/SPI
        #vphoenix_embedded_base_code/Bsp/Tim
        phoenix_embedded_base_code/Bsp/Uart

        phoenix_embedded_base_code/Module/BMI088
        phoenix_embedded_base_code/Module/Buzzer
        # phoenix_embedded_base_code/Module/Com_System
        phoenix_embedded_base_code/Module/Daemon
        phoenix_embedded_base_code/Module/Dr16
        phoenix_embedded_base_code/Module/Et08a
        phoenix_embedded_base_code/Module/Ist8310
        phoenix_embedded_base_code/Module/Led
        # phoenix_embedded_base_code/Module/Minipc
        phoenix_embedded_base_code/Module/Motor/Bm
        phoenix_embedded_base_code/Module/Motor/Dji
        phoenix_embedded_base_code/Module/Motor/Dm
        phoenix_embedded_base_code/Module/Motor/Lk
        phoenix_embedded_base_code/Module/Motor/RobStride
        phoenix_embedded_base_code/Module/Motor/Unitree
        phoenix_embedded_base_code/Module/Referee
        phoenix_embedded_base_code/Module/SEGGER
        phoenix_embedded_base_code/Module/SEGGER/Jlink_RTT
        phoenix_embedded_base_code/Module/Servo

        UserFile
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
        -DARM_MATH_CM4
        -DARM_MATH_MATRIX_CHECK
        -DARM_MATH_ROUNDING
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
