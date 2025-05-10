#!/bin/bash

# 检查脚本是否以 root 权限运行
if [[ $EUID -ne 0 ]]; then
   echo "错误：此脚本必须以 root 权限运行 (使用 sudo)."
   exit 1
fi

# 设置 -e：如果任何命令失败，脚本将立即退出
set -e

SYSTEMD_SERVICE_DIR="/etc/systemd/system/"

# --- 获取脚本所在的绝对路径 ---
# $0 是脚本的名称（可能包含相对路径）
# dirname "$0" 获取脚本所在的目录
# cd "$(dirname "$0")" 进入该目录
# pwd 获取当前的绝对路径
# 或者使用更简洁和健壮的方式（推荐）： readlink -f "$0" 解析符号链接并获取脚本的绝对路径，然后 dirname 获取目录
SCRIPT_PATH="$(readlink -f "$0")"
SCRIPT_DIR="$(dirname "$SCRIPT_PATH")"

echo "正在查找脚本所在目录 ($SCRIPT_DIR) 中的所有 .service 文件..."

# 查找脚本目录下的所有 .service 文件，并存储到数组中
# 使用 nullglob 选项，以便在没有匹配文件时，数组为空而不是包含 "*.service" 字符串
shopt -s nullglob
# 这里的模式现在是相对于 SCRIPT_DIR 的路径
service_files=("$SCRIPT_DIR"/*.service)
shopt -u nullglob # 关闭 nullglob 选项

# 检查是否找到了 .service 文件
# 注意：如果 service_files 为空，那么 ${#service_files[@]} 是 0
# 如果没有匹配到，但 nullglob 未开启，它会是 ("$SCRIPT_DIR/*.service")，数量是 1，这是错误的
# nullglob 选项确保了正确性
if [ ${#service_files[@]} -eq 0 ]; then
  echo "未在脚本目录 ($SCRIPT_DIR) 找到任何 .service 文件。"
  exit 0
fi

# 由于数组中现在是完整路径，打印时显示完整路径
echo "找到以下 .service 文件：${service_files[@]}"

# 1. 复制文件到 systemd 目录
echo "正在复制 .service 文件到 $SYSTEMD_SERVICE_DIR..."
# 遍历数组，每个元素都是一个 service 文件的完整路径
for service_file_path in "${service_files[@]}"; do
  # 提取文件名（例如，从 /path/to/my_app.service 中得到 my_app.service）
  service_name=$(basename "$service_file_path")
  echo "复制 $service_name..."
  # 复制时使用完整源路径和目标目录+文件名
  cp "$service_file_path" "$SYSTEMD_SERVICE_DIR/$service_name"
  echo "$service_name 复制完成。"
done

# 2. 重新加载 systemd daemon，使其识别新复制的服务文件
echo "正在重新加载 systemd daemon..."
systemctl daemon-reload
echo "systemd daemon 重新加载完成。"

# 3. 设置服务为开机自启动 (enable)
echo "正在设置服务为开机自启动..."
# 再次遍历文件路径（或者直接使用文件名数组也可以，但用路径数组更一致）
for service_file_path in "${service_files[@]}"; do
  # 提取文件名作为服务名称
  service_name=$(basename "$service_file_path")
  echo "启用服务 $service_name..."
  systemctl enable "$service_name"
  echo "服务 $service_name 已启用。"
done

echo "所有找到的 .service 文件已复制到 $SYSTEMD_SERVICE_DIR 并设置为开机自启动。"
echo "你现在可以使用 'sudo systemctl start <service_name>.service' 命令手动启动服务。"