#!/bin/bash
# 修复so文件的rpath

# 获取脚本的上级目录（PROJECT_DIR）
SCRIPT_DIR="$(dirname "$(realpath "$0")")"
PROJECT_DIR="$(dirname "${SCRIPT_DIR}")"

# 检查输入参数
if [ $# -ne 1 ]; then
    echo "Usage: $0 <path/to/library.so>"
    exit 1
fi

LIB_PATH="$1"
if [ ! -f "${LIB_PATH}" ]; then
    echo "Error: Library file ${LIB_PATH} not found!"
    exit 1
fi

# 检查依赖命令是否存在
command -v ldd >/dev/null 2>&1 || { echo >&2 "Error: ldd required but not found."; exit 1; }
command -v patchelf >/dev/null 2>&1 || { echo >&2 "Error: patchelf required but not found."; exit 1; }
command -v mlocate >/dev/null 2>&1 || { echo >&2 "Error: mlocate required but not found."; exit 1; }

# 获取 not found 的库列表
missing_libs=$(ldd "${LIB_PATH}" | awk '/not found/{gsub(/\(.*/, ""); print $1}')

# 处理每个缺失的库
for lib in $missing_libs; do
    echo "Processing missing library: ${lib}"
    
    # 使用 mlocate 查找库文件
    found_paths=$(locate -b "${lib}" | grep "^${PROJECT_DIR}")
    
    if [ -z "${found_paths}" ]; then
        echo "  Warning: Library ${lib} not found under ${PROJECT_DIR}"
        continue
    fi
    
    # 选择第一个找到的有效路径
    target_path=$(echo "${found_paths}" | head -n1)

    # 剥离文件名，只保留目录路径
    target_dir=$(dirname "${target_path}")
    
    # 计算相对路径（基于 ORIGIN）
    lib_dir=$(dirname "${LIB_PATH}")
    relative_path=$(realpath --relative-to="${lib_dir}" "${target_dir}")
    rpath_entry="\$ORIGIN/${relative_path}"
    
    # 检查是否已存在该 rpath
    current_rpath=$(patchelf --print-rpath "${LIB_PATH}")
    if [[ "${current_rpath}" == *"${rpath_entry}"* ]]; then
        echo "  RPATH already contains: ${rpath_entry}"
        continue
    fi
    
    # 添加新的 rpath
    echo "  Adding rpath: ${rpath_entry}"
    if [ -z "${current_rpath}" ]; then
        new_rpath="${rpath_entry}"
    else
        new_rpath="${current_rpath}:${rpath_entry}"
    fi
    
    patchelf --set-rpath "${new_rpath}" "${LIB_PATH}"
done

echo "RPATH update completed for ${LIB_PATH}"