cmake_minimum_required(VERSION 3.16)

# Set the supported ESP-IDF version
set(SUPPORTED_TARGETS esp32)

# Include ESP-IDF project
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Define project
project(handsignal_esp32)