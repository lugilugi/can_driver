cmake_minimum_required(VERSION 3.16)

# This tells ESP-IDF to look for extra components in the repo root,
# which is where your actual component lives.
set(EXTRA_COMPONENT_DIRS "../../..")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(basic_test)