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

if [ ! -d "$PROJECT_ROOT/release" ]; then
    echo -e "${RED}错误: 找不到release目录，请确保已经编译安装了mujoco${NC}"
    exit 1
fi

if [ ! -f "$PROJECT_ROOT/release/lib/libmujoco.so" ]; then
    echo -e "${RED}错误: 找不到libmujoco.so，请确保mujoco已正确安装${NC}"
    exit 1
fi

if [ ! -d "$PROJECT_ROOT/my_plugins/damper" ]; then
    echo -e "${RED}错误: 找不到damper插件源代码目录${NC}"
    exit 1
fi

# 创建mujoco_plugin目录（如果不存在）
mkdir -p "$PROJECT_ROOT/release/bin/mujoco_plugin"

# 进入damper插件目录
cd "$PROJECT_ROOT/my_plugins/damper"

echo -e "${YELLOW}开始构建damper插件...${NC}"

# 清理旧的构建文件
if [ -d "build" ]; then
    echo "清理旧的构建文件..."
    rm -rf build
fi

# 创建构建目录
mkdir build
cd build

# 配置CMake
echo -e "${YELLOW}配置CMake...${NC}"
cmake .. -DCMAKE_BUILD_TYPE=Release

# 构建
echo -e "${YELLOW}编译插件...${NC}"
cmake --build . -j$(nproc)

# 安装
echo -e "${YELLOW}安装插件到 release/bin/mujoco_plugin/...${NC}"
cmake --install .

# 验证安装
if [ -f "$PROJECT_ROOT/release/bin/mujoco_plugin/libdamper.so" ]; then
    echo -e "${GREEN}✅ 成功！插件已安装到: $PROJECT_ROOT/release/bin/mujoco_plugin/libdamper.so${NC}"
    
    # 显示插件信息
    echo -e "${BLUE}插件信息:${NC}"
    ls -lh "$PROJECT_ROOT/release/bin/mujoco_plugin/libdamper.so"
    
    # 检查插件依赖
    echo -e "${BLUE}插件依赖检查:${NC}"
    ldd "$PROJECT_ROOT/release/bin/mujoco_plugin/libdamper.so" | grep -E "(mujoco|not found)" || true
else
    echo -e "${RED}❌ 安装失败！找不到生成的插件文件${NC}"
    exit 1
fi

echo -e "${GREEN}=== 构建完成 ===${NC}"
echo -e "${BLUE}使用方法:${NC}"
echo "在你的XML文件中添加:"
echo '<plugin plugin="$PROJECT_ROOT/release/bin/mujoco_plugin/libdamper.so"/>'
echo ""
echo "或者设置环境变量:"
echo "export MUJOCO_PLUGIN_PATH=\"$PROJECT_ROOT/release/bin/mujoco_plugin\""
