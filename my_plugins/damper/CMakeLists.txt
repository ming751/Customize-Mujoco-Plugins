# 最好放在最顶层 CMakeLists.txt 的开头附近
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE BOOL "Enable compile_commands.json")
cmake_minimum_required(VERSION 3.16)
project(damper_plugin LANGUAGES CXX)

# 设置 C++ 标准为 C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_PREFIX_PATH)
  # 将 CMAKE_PREFIX_PATH 转换为绝对路径，并存入新变量 ABS_PREFIX_PATH
  # ABSOLUTE 会相对于 CMAKE_CURRENT_SOURCE_DIR (当前处理的CMakeLists.txt所在目录) 来解析
  get_filename_component(ABS_PREFIX_PATH "${CMAKE_PREFIX_PATH}" ABSOLUTE)
  message(STATUS "Resolved absolute CMAKE_PREFIX_PATH is: ${ABS_PREFIX_PATH}")
else()
  message(STATUS "CMAKE_PREFIX_PATH is not set.")
endif()
# 告诉 CMake 去哪找 MuJoCo
# 设置mujoco的安装路径（相对于当前CMakeLists.txt的路径）
get_filename_component(MUJOCO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../release" ABSOLUTE)
message(STATUS "Looking for MuJoCo at: ${MUJOCO_ROOT}")

# 方法 1：直接指定路径查找mujoco
find_package(mujoco REQUIRED CONFIG PATHS "${MUJOCO_ROOT}/lib/cmake/mujoco" NO_DEFAULT_PATH)

add_library(damper SHARED
    spring_damper.cc
    spring_damper.h
    register.cc)

# 链接mujoco库
target_link_libraries(damper PRIVATE mujoco::mujoco)

# 设置包含目录（通常由target自动处理，但为了确保可以显式设置）
target_include_directories(damper PRIVATE
    "${MUJOCO_ROOT}/include")

set_target_properties(damper PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}) # 生成在 plugin/

# 安装配置：将插件安装到 release/bin/mujoco_plugin/
# install(TARGETS damper
#     LIBRARY DESTINATION "${CMAKE_CURRENT_SOURCE_DIR}/../../release/bin/mujoco_plugin"
#     COMPONENT plugins)