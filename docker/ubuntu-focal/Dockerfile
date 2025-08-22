# 基于 Ubuntu 20.04 LTS
FROM ubuntu:focal

# 设置非交互式环境（避免安装时的交互提示）
ENV DEBIAN_FRONTEND=noninteractive

# 设置时区
ENV TZ=Asia/Shanghai

# 创建非root用户（安全最佳实践）
RUN useradd -m -s /bin/bash developer && \
    usermod -aG sudo developer

# 更新包列表并安装开发工具
RUN apt-get update && apt-get install -y \
    # 基础构建工具
    build-essential \
    cmake \
    ninja-build \
    pkg-config \
    # 版本控制
    git \
    # 调试和分析工具
    gdb \
    valgrind \
    # 代码质量工具
    clang \
    clang-format \
    clang-tidy \
    # 编译缓存
    ccache \
    # 系统工具
    sudo \
    # 实用工具
    curl \
    wget \
    vim \
    nano \
    tree \
    htop \
    # X11和OpenGL开发库
    libx11-dev \
    libxrandr-dev \
    libxinerama-dev \
    libxcursor-dev \
    libxi-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    # 清理缓存
    && rm -rf /var/lib/apt/lists/*

# 设置 ccache 路径（加速重复编译）
ENV PATH=/usr/lib/ccache:$PATH
ENV CCACHE_DIR=/workspace/.ccache

# 配置 Git（可选）
RUN git config --global init.defaultBranch main

# 验证安装是否成功
RUN gcc --version && \
    g++ --version && \
    cmake --version && \
    clang --version

# 设置工作目录
WORKDIR /workspace

# 切换到非root用户
USER developer

# 设置默认命令
CMD ["/bin/bash"]