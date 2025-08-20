set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE BOOL "Enable compile_commands.json")
cmake_minimum_required(VERSION 3.16)
project(inspector_plugin LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(mujoco REQUIRED)

add_library(inspector SHARED
  src/inspector.cc
  src/register.cc)

target_include_directories(inspector PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(inspector PRIVATE mujoco::mujoco)

set_target_properties(inspector PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

