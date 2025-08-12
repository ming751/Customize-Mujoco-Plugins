# 自定义Mujoco插件实践
一个完整的Mujoco Plugins 自定义仓库，用于定制属于你的Plugin, 做到控制和使用进行解耦



## 教程
1. 拉取主仓库以及子仓库
```bash
git clone --recurse-submodules git@github.com:ming751/Customize-Mujoco-Plugins.git
# git submodule update --init --recursive
```

2. 安装依赖
```bash
sudo apt update
sudo apt install build-essential cmake git ccache -y
```

3. 一键运行安装mujoco以及插件
```bash
sudo chmod a+x auto_script.sh
bash ./auto_script.sh
```

4. 验证插件是否正常安装以及运行
```bash
export PATH="$(pwd)/release/bin:$PATH"
export LD_LIBRARY_PATH="$(pwd)/release/lib:$LD_LIBRARY_PATH"
simulate test_spring_damper.xml
```
TODO：docker 不同系统安装

<!-- 2. 编译安装mujoco
```bash
cd ~/mujoco
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target mujoco -j
cmake --install build --prefix ../../release   # 安装到自定义前缀
```
3. 使用cache加速重复编译安装
```bash
sudo apt update
sudo apt install ccache -y
``` -->


<!-- 4. 编译插件
```bash
cd ~/my_plugins/damper
cmake -B build -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_PREFIX_PATH="../../release"
cmake --build build -j
```

需要将./simulate 和 mujoco_plugin 放在同一个文件夹
（也可以把 MUJOCO_PLUGIN_DIR 环境变量指向 ~/run/mujoco_plugin，
但官方的可执行文件默认就会搜索同级目录中的 mujoco_plugin。）


# assemble run dir
RUN=run
mkdir -p $RUN/bin $RUN/mujoco_plugin
cp $MUJOCO_HOME/bin/simulate $RUN/bin/
cp $MUJOCO_HOME/lib/libmujoco.so $RUN/bin/
cp build/plugin/libdamper.so     $RUN/mujoco_plugin/

# run
cd $RUN
export LD_LIBRARY_PATH=$PWD/bin
./bin/simulate ../test_spring_damper.xml -->