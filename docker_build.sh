#!/bin/bash
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
echo "脚本目录: ${SCRIPT_DIR}"

cd ${SCRIPT_DIR}/docker

echo "🐳 启动 Docker 开发环境..."
docker compose up -d --build 

#docker compose up --build --force-recreate -d # 强制重新构建   -d 表示后台运行
# 作用: 切换到docker目录，因为docker-compose.yaml在这里。
# 作用: 启动docker容器，并构建镜像。

# -- 新增步骤：修复 .ccache 目录的权限 --
echo "🔧 正在修复 .ccache 目录权限..."
docker compose exec --user root dev-environment chown -R developer:developer /workspace/.ccache  #这里是服务名字


echo "⏳ 等待容器启动..."
sleep 3

echo "📦 在容器中执行构建脚本..."
docker exec -it ming-dev-ubuntu-focal  /workspace/auto_script.sh #这里是容器名字

# docker exec: 在运行的容器中执行命令
# -it: 交互式终端
# my-cpp-dev: 容器名称
# /workspace/auto_build.sh --all: 要执行的命令

echo ""
echo "🎉 Docker 构建完成!"
echo ""
echo "💡 提示:"
echo "   - 使用 'docker exec -it ming-dev-ubuntu-focal bash' 进入容器"
echo "   - 使用 'docker compose -f ${SCRIPT_DIR}/docker/docker-compose.yaml down' 停止容器"