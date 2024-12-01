#!/bin/bash

# 动态添加 flexiv_rdk_path 到 PYTHONPATH
Install_dir=$(dirname "$(readlink -f "$0")")
project_dir=$(dirname "$Install_dir")
flexiv_rdk_path="$project_dir/Install/flexiv_rdk/lib_py"

# 检查并永久添加到 ~/.bashrc
if ! grep -q "$flexiv_rdk_path" ~/.bashrc; then
    echo "export PYTHONPATH=\$PYTHONPATH:$flexiv_rdk_path" >> ~/.bashrc
    echo "flexiv_rdk_path added to PYTHONPATH in ~/.bashrc"
else
    echo "flexiv_rdk_path is already in ~/.bashrc"
fi