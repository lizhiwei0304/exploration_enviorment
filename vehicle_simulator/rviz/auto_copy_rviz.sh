#!/bin/bash

# 删除所有已存在的vehiclei.rviz文件
rm -f vehicle*.rviz

# 检查原始文件是否存在
if [ ! -f "simulator.rviz" ]; then
    echo "Error: 'simulator.rviz' 文件不存在."
    exit 1
fi

# 提示用户输入副本数量
read -p "请输入要生成的副本数量: " num_copies

# 检查用户输入是否为正整数
if ! [[ "$num_copies" =~ ^[1-9][0-9]*$ ]]; then
    echo "Error: 请输入一个正整数."
    exit 1
fi

# 循环生成副本并替换内容
for ((i=1; i<=num_copies; i++)); do
    # 复制文件并重命名
    cp simulator.rviz vehicle$i.rviz

    # 替换文件内容
    sed -i "s/vehicle1/vehicle$i/g" vehicle$i.rviz
    sed -i "s/Vehicle1/Vehicle$i/g" vehicle$i.rviz

    echo "成功生成并替换 vehicle$i.rviz 文件中的字符串."
done

