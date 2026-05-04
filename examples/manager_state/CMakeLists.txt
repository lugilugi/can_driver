cmake_minimum_required(VERSION 3.16)

# Look for can_driver in the repository root.
set(EXTRA_COMPONENT_DIRS "../..")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(manager_state_example)
