#!/bin/bash

# 定义基础输出目录
BASE_OUTPUT_DIR="/home/pnx/ros2_bag"

# 获取当前时间并格式化为 YYYY-MM-DD_HH-MM-SS，用于文件夹命名
# 使用 date '+%Y-%m-%d_%H-%M-%S' 可以生成一个易于阅读的时间戳 [10, 15]
TIMESTAMP=$(date '+%Y-%m-%d_%H-%M-%S')

# 构建完整的输出路径
OUTPUT_PATH="$BASE_OUTPUT_DIR/$TIMESTAMP"

# 检查基础输出目录是否存在，如果不存在则创建 [1]
if [ ! -d "$BASE_OUTPUT_DIR" ]; then
  echo "基础目录 $BASE_OUTPUT_DIR 不存在，正在创建..."
  mkdir -p "$BASE_OUTPUT_DIR"
  if [ $? -ne 0 ]; then
    echo "创建基础目录失败！请检查权限或路径是否正确。"
    exit 1
  fi
  echo "基础目录 $BASE_OUTPUT_DIR 创建成功。"
fi

# 检查是否成功创建了完整输出路径的父目录（即基础目录）
if [ ! -d "$BASE_OUTPUT_DIR" ]; then
    echo "基础目录 $BASE_OUTPUT_DIR 仍然不存在，无法继续。退出脚本。"
    exit 1
fi


# 构建 ros2 bag record 命令
# 指定录制话题和分段时长参数
# ROS2_RECORD_CMD="/opt/ros/humble/bin/ros2 bag record -o $OUTPUT_PATH --max-bag-duration 60 sensor/RealSense/point_cloud sensor/camera/images /computer_state /player_command /tf"
ROS2_RECORD_CMD="/opt/ros/humble/bin/ros2 bag record -o $OUTPUT_PATH --max-bag-duration 60 -a"

echo "录制将保存到: $OUTPUT_PATH"
echo "正在执行命令: $ROS2_RECORD_CMD"

# 执行 ros2 bag record 命令
# 使用 exec 可以替换当前 shell 进程为 ros2 bag 命令进程，
# 这样当你停止 ros2 bag 时，脚本也会退出
exec $ROS2_RECORD_CMD

# 注意：如果 ros2 bag record 成功执行，exec 会替换当前进程，
# 所以下面的 echo 语句通常不会被执行。
echo "ros2 bag record 命令执行完毕或出错。"