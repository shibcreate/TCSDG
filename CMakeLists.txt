# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for CMake to work correctly
cmake_minimum_required(VERSION 3.5)

set(EXTRA_COMPONENT_DIRS "components/ra01s")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(TCSMiniLoRa)

set(PARTITION_TABLE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/partitions.csv)