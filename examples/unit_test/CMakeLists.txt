cmake_minimum_required(VERSION 3.16)

# Point ESP-IDF at this repository so can_driver is available as a component.
set(EXTRA_COMPONENT_DIRS "../..")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(can_unit_tests)
