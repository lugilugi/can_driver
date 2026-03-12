cmake_minimum_required(VERSION 3.16)

# This tells the project to look one directory up for the component
set(EXTRA_COMPONENT_DIRS "../../")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(can_driver_basic_test)