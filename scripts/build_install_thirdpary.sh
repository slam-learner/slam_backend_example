#!/bin/bash
##
## author: ydx
## description: 编译安装第三方库到项目目录下
## date: 2025-06-29

# 记录脚本、项目目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null 2>&1 && pwd )"
PROJECT_DIR="$SCRIPT_DIR/.."

# 如果没有指定编译类型, 则默认为Release
BUILD_TYPE="${1:-Release}"
CLEAN_BUILD="${2:-false}"
echo "Build type of third party: $BUILD_TYPE"

SPLIT_LINE="####################################################################"

# 使用同样的标准对各个库进行编译
COMMON_CXX_STANDARD=17
echo "C++ standard: $COMMON_CXX_STANDARD"

# 编译线程数
NUM_PARALLEL_BUILDS=8
echo "Number of build thread: $NUM_CORES"

# Eigen默认使用16字节的对齐, 但当AVX指令集可用时, Eigen会使用32字节的对齐
# 如果各个库或者项目使用了不同的对齐方式, 可能会导致内存访问错误
# 而 Ceres 库以及 OpenGV 库在release模式会强制性传递 arch=native 选项
# 在现代的CPU上, arch=native 选项会启用 SSE4.2 和 AVX 指令集也就是会使用32字节的对齐
# 为了防止Eigen库引入一些微妙的bug, 需要在各处编译时都加入arc=native选项
# 参见：https://eigen.tuxfamily.org/dox/TopicPreprocessorDirectives.html
if [ "$(uname -m)" = "x86_64" ]; then
    CMAKE_MARCH="{CXX_MARCH:-native}"
fi

if [ ! -z "$CXX_MARCH" ]; then
    EXTRA_CXX_FLAGS="$EXTRA_CXX_FLAGS -march=$CXX_MARCH"
fi

EXTRA_CXX_FLAGS="${EXTRA_CXX_FLAGS} ${CXX_FLAGS} -Wno-deprecated-enum-enum-conversion"

INSTALL_DIR="$PROJECT_DIR/install"

# 通用的CMake参数
# -DCMAKE_BUILD_TYPE=${BUILD_TYPE} 指定编译类型
# -DCMAKE_CXX_STANDARD=${COMMON_CXX_STANDARD} 指定C++标准
# -DCMAKE_CXX_FLAGS="${EXTRA_CXX_FLAGS}" 指定额外的C++编译选项
# -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} 指定安装目录
# -DCMAKE_PREFIX_PATH=${INSTALL_DIR} 指定查找依赖库的路径, find_package()会在这个路径下查找依赖库
# -DCMAKE_CXX_EXTENSIONS=OFF 禁用C++扩展
# -DCMAKE_POSITION_INDEPENDENT_CODE=ON 生成位置无关的代码
# -DBUILD_SHARED_LIBS=ON 使用动态库
COMMON_CMAKE_ARGS=(
    -DCMAKE_BUILD_TYPE=${BUILD_TYPE}
    -DCMAKE_CXX_STANDARD=${COMMON_CXX_STANDARD}
    -DCMAKE_CXX_FLAGS="${EXTRA_CXX_FLAGS}"
    -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR}
    -DCMAKE_PREFIX_PATH=${INSTALL_DIR}
    -DCMAKE_CXX_EXTENSIONS=OFF
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON
    -DBUILD_SHARED_LIBS=ON
)

# 检查ccache是否可用, 如果可用则使用ccache加速编译
if command -v ccache > /dev/null 2>&1; then
    echo "ccache found! Using ccache to speed up compilation!"
    COMMON_CMAKE_ARGS+=(
            -DCMAKE_C_COMPILER_LAUNCHER=ccache
            -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
            "${COMMON_CMAKE_ARGS[@]}"
        )
else
    echo "ccache not found! Compiling without ccache!"
fi

mkdir -p "$INSTALL_DIR"

THIRD_SOURCE_DIR="$PROJECT_DIR/external"
##################################
## ceres
if false; then
    echo ""
    echo "$SPLIT_LINE"
    echo "$SPLIT_LINE"
    echo ""

    echo "Building gtsam..."
    cd "$THIRD_SOURCE_DIR/ceres-solver"
    if [ "$CLEAN_BUILD" = true ]; then
        rm -rf build
    fi
    mkdir -p build
    cd build
    cmake .. "${COMMON_CMAKE_ARGS[@]}" -DCMAKE_PREFIX_PATH=$PROJECT_DIR/thirdparty/eigen3/share/eigen3/cmake;$PROJECT_DIR/thirdparty/gflags/lib/cmake/gflags;$PROJECT_DIR/thirdparty/glog/lib/cmake/glog
    make -j$NUM_PARALLEL_BUILDS
    make install
fi

##################################
## g2o
if true; then
    echo ""
    echo "$SPLIT_LINE"
    echo "$SPLIT_LINE"
    echo ""

    echo "Building gtsam..."
    cd "$THIRD_SOURCE_DIR/g2o"
    if [ "$CLEAN_BUILD" = true ]; then
        rm -rf build
    fi
    mkdir -p build
    cd build
    cmake .. "${COMMON_CMAKE_ARGS[@]}" -DCMAKE_PREFIX_PATH=$PROJECT_DIR/thirdparty/eigen3/share/eigen3/cmake;$PROJECT_DIR/thirdparty/spdlog/lib/cmake/spdlog
    make -j$NUM_PARALLEL_BUILDS
    make install
fi

##################################
## gtsam
if true; then
    echo ""
    echo "$SPLIT_LINE"
    echo "$SPLIT_LINE"
    echo ""

    echo "Building gtsam..."
    cd "$THIRD_SOURCE_DIR/gtsam"
    if [ "$CLEAN_BUILD" = true ]; then
        rm -rf build
    fi
    mkdir -p build
    cd build
    cmake .. "${COMMON_CMAKE_ARGS[@]}"
    make -j$NUM_PARALLEL_BUILDS
    make install
fi

echo ""
echo "$SPLIT_LINE"
echo "$SPLIT_LINE"
echo ""
echo "All third party libraries are built and installed to $INSTALL_DIR!"
