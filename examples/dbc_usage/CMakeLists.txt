cmake_minimum_required(VERSION 3.16)

# Provide path to the ESP-IDF CMake framework
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Point to the root directory so the example can find the can_driver component
set(EXTRA_COMPONENT_DIRS "../../")

project(can_dbc_example)
