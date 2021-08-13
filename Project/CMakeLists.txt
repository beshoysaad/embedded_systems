cmake_minimum_required(VERSION 3.3)

# include configuration
include(Config.cmake)

set(CMAKE_TOOLCHAIN_FILE libs/Toolchain.cmake)

project(EmbeddedSystems18)

include(libs/Arduino.cmake)
include(libs/Pi.cmake)
include(libs/Zumo.cmake)

include_directories("hdr")

# Scout Executable
add_pi_executable(scout
        src/scout/main.cpp
        #src/scout/line.cpp
        #src/scout/referee.cpp
        #src/scout/relay.cpp
        #src/scout/ping.cpp
        #src/scout/homing.cpp
        #src/scout/photophobic.cpp
        #src/scout/controller.cpp
        src/scout/light.cpp
        src/PID_v1.cpp
        src/softspi.cpp
        src/radio.cpp
        #src/radio_hal.cpp
        #src/motor_hal.cpp
        )

# Collector Executable
add_zumo_executable(collector
        src/collector/main.cpp
        #src/collector/line.cpp
        #src/collector/referee.cpp
        #src/collector/relay.cpp
        #src/collector/pong.cpp
        #src/collector/homing.cpp
        #src/collector/proximity.cpp
        src/softspi.cpp
        src/radio.cpp
        )