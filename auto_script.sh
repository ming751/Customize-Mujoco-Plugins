#!/bin/bash

# MuJoCo 插件自动化构建和安装脚本
# 用途：编译 my_plugins 下的所有插件，并安装到 release/bin/mujoco_plugin/

set -e  # 遇到错误立即退出

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 获取脚本所在目录的绝对路径
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$SCRIPT_DIR"

# 源文件和构建目录
MUJOCO_SRC_DIR="$PROJECT_ROOT/mujoco"
MUJOCO_BUILD_DIR="$MUJOCO_SRC_DIR/build"
PLUGINS_ROOT_DIR="$PROJECT_ROOT/my_plugins"

# 发布和安装目录
RELEASE_DIR="$PROJECT_ROOT/release"
PLUGIN_INSTALL_DIR="$RELEASE_DIR/bin/mujoco_plugin"
MUJOCO_INSTALL_LIB_PATH="$RELEASE_DIR/lib/libmujoco.so" # 用于检查是否已安装

# --- 日志函数 ---
log_info() {
    echo -e "${BLUE}[INFO] $1${NC}"
}

log_success() {
    echo -e "${GREEN}[SUCCESS] $1${NC}"
}

log_warning() {
    echo -e "${YELLOW}[WARNING] $1${NC}"
}

log_error() {
    echo -e "${RED}[ERROR] $1${NC}" >&2
    exit 1
}

# --- 帮助信息 ---
usage() {
    echo "用法: $0 [clean]"
    echo "  <无参数>   执行完整的构建和安装流程。"
    echo "  clean      清理所有构建目录和发布目录。"
}

# --- 功能函数 ---

# 检查依赖项是否存在
check_deps() {
    log_info "检查环境依赖..."
    local missing_deps=0
    for cmd in cmake git g++ ccache; do
        if ! command -v "$cmd" &> /dev/null; then
            log_warning "依赖 '$cmd' 未找到。"
            missing_deps=1
        fi
    done
    if [ "$missing_deps" -eq 1 ]; then
        log_error "请先安装以上缺失的依赖项再运行此脚本。"
    fi
    log_success "依赖检查通过。"
}

# 清理构建产物
clean() {
    log_info "开始清理构建产物..."
    rm -rf "$MUJOCO_BUILD_DIR"
    # 清理所有插件的 build 目录
    if [ -d "$PLUGINS_ROOT_DIR" ]; then
        for dir in "$PLUGINS_ROOT_DIR"/*; do
            [ -d "$dir" ] || continue
            rm -rf "$dir/build"
        done
    fi
    rm -rf "$RELEASE_DIR"
    log_success "清理完成。"
}

# 编译和安装 MuJoCo
build_mujoco() {
    log_info "开始处理 MuJoCo..."
    if [ -f "$MUJOCO_INSTALL_LIB_PATH" ]; then
        log_success "MuJoCo 已安装，跳过构建。"
        return
    fi

    log_info "配置 MuJoCo (Release模式)..."
    # -S 指定源码目录, -B 指定构建目录
    cmake -S "$MUJOCO_SRC_DIR" -B "$MUJOCO_BUILD_DIR" \
          -DCMAKE_BUILD_TYPE=Debug \
          -DCMAKE_C_COMPILER_LAUNCHER=ccache \
          -DCMAKE_CXX_COMPILER_LAUNCHER=ccache

    log_info "编译 MuJoCo..."
    # --build 指定构建目录, -j 使用所有可用核心并行编译
    cmake --build "$MUJOCO_BUILD_DIR" -j"$(nproc)"

    log_info "安装 MuJoCo 到发布目录..."
    # --install 指定构建目录, --prefix 指定安装路径
    cmake --install "$MUJOCO_BUILD_DIR" --prefix "$RELEASE_DIR"
    log_success "MuJoCo 处理完成。"
}

# 编译 my_plugins 下所有插件
build_plugins() {
    log_info "开始构建所有插件..."
    local cmake_prefix_path="-DCMAKE_PREFIX_PATH=$RELEASE_DIR"
    if [ ! -d "$PLUGINS_ROOT_DIR" ]; then
        log_warning "未找到插件根目录: $PLUGINS_ROOT_DIR"
        return
    fi
    for plugin_dir in "$PLUGINS_ROOT_DIR"/*; do
        [ -d "$plugin_dir" ] || continue
        if [ ! -f "$plugin_dir/CMakeLists.txt" ]; then
            log_warning "跳过(无CMakeLists): $plugin_dir"
            continue
        fi
        local build_dir="$plugin_dir/build"
        log_info "配置插件: $plugin_dir"
        cmake -S "$plugin_dir" -B "$build_dir" \
            -DCMAKE_BUILD_TYPE=Release \
            -DCMAKE_C_COMPILER_LAUNCHER=ccache \
            -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
            "$cmake_prefix_path"
        log_info "编译插件: $plugin_dir"
        cmake --build "$build_dir" -j"$(nproc)"
        log_success "编译完成: $plugin_dir"
    done
}

# 安装所有插件（复制 .so 到安装目录）
install_plugins() {
    log_info "开始安装所有插件..."
    mkdir -p "$PLUGIN_INSTALL_DIR"
    local any_found=0
    for plugin_dir in "$PLUGINS_ROOT_DIR"/*; do
        [ -d "$plugin_dir" ] || continue
        local build_dir="$plugin_dir/build"
        if [ ! -d "$build_dir" ]; then
            log_warning "未构建，跳过安装: $plugin_dir"
            continue
        fi
        local so_files
        so_files=$(find "$build_dir" -maxdepth 1 -type f -name "*.so" || true)
        if [ -z "$so_files" ]; then
            log_warning "未找到 .so: $build_dir"
            continue
        fi
        for so in $so_files; do
            cp "$so" "$PLUGIN_INSTALL_DIR/"
            any_found=1
            log_info "已安装: $(basename "$so")"
        done
    done
    if [ "$any_found" -eq 1 ]; then
        log_success "所有可用插件已安装到: $PLUGIN_INSTALL_DIR"
    else
        log_warning "未安装任何插件 (.so 未找到)"
    fi
}




# --- 主函数 ---
main() {
    echo -e "${BLUE}=== MuJoCo 插件自动化构建脚本 ===${NC}"
    echo "项目根目录: $PROJECT_ROOT"
    echo "-------------------------------------"

    if [ $# -gt 1 ] || ([ $# -eq 1 ] && [ "$1" != "clean" ]); then
        usage
        exit 1
    fi

    if [ "${1:-}" = "clean" ]; then
        clean
    else
        # 默认执行完整流程
        check_deps
        build_mujoco
        build_plugins
        install_plugins
    fi

    echo "-------------------------------------"
    log_success "所有任务已完成！"
}

# --- 脚本入口 ---
# 将所有命令行参数传递给主函数
main "$@"

