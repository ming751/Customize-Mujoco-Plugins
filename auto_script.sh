#!/bin/bash

# Damper插件自动化构建和安装脚本
# 用途：编译damper插件并安装到release/bin/mujoco_plugin/目录

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

echo -e "${BLUE}=== Damper插件自动化构建脚本 ===${NC}"
echo "项目根目录: $PROJECT_ROOT"

# 检查必要的目录和文件
echo -e "${YELLOW}检查环境...${NC}"

# 安装mujoco
echo -e "${YELLOW}安装mujoco中...${NC}"
#ls
cd "$PROJECT_ROOT/mujoco"
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target mujoco -j
cmake --install build --prefix ../../release   # 安装到自定义前缀


