#!/bin/bash
#==============================================================================
# emd-gaf 交叉编译脚本
# 用法: ./build.sh [build|clean|native]
#   build  - 交叉编译 (默认, 清理后编译, aarch64)
#   clean  - 仅清理构建目录
#   native - 本机编译 (x86_64, 用于测试)
#
# 产物:
#   build/libemd_gaf.so     — 动态库 (机器人主程序链接使用)
#   build/libemd_gaf.a      — 静态库
#   build/emd-gaf           — 原有独立可执行文件 (保留)
#   build/read_sensor       — 示例程序 (链接 libemd_gaf)
#==============================================================================
set -e

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
TOOLCHAIN="$PROJECT_DIR/toolchain.cmake"
CMAKE_CROSS="/opt/gcc-arm-12.4-x86_64-aarch64-linux-gnu/bin/cmake"
READELF="/opt/gcc-arm-12.4-x86_64-aarch64-linux-gnu/bin/aarch64-buildroot-linux-gnu-readelf"

BUILD_DIR="$PROJECT_DIR/build"

#==============================================================================
# 清理函数
#==============================================================================
_clean() {
    echo "清理构建目录..."
    rm -rf "$BUILD_DIR"
    echo " ✓ build/ 已删除"
}

CMD="${1:-build}"

case "$CMD" in
    clean)
        _clean
        exit 0
        ;;
    build|native)
        ;;
    *)
        echo "用法: $0 [build|clean|native]"
        echo "  build  - 交叉编译 (aarch64, 清理后编译)"
        echo "  clean  - 仅清理构建目录"
        echo "  native - 本机编译 (x86_64, 用于测试)"
        exit 1
        ;;
esac

#==============================================================================
# 编译前清理
#==============================================================================
_clean

#==============================================================================
# 编译
#==============================================================================
if [ "$CMD" = "native" ]; then
    echo "=========================================="
    echo " emd-gaf 本机编译 (x86_64)"
    echo "=========================================="

    mkdir -p "$BUILD_DIR"
    cmake -S "$PROJECT_DIR" -B "$BUILD_DIR"
    cmake --build "$BUILD_DIR" -j"$(nproc)"

    echo ""
    echo "=========================================="
    echo " 编译完成!"
    echo "=========================================="
    echo ""

    ls -lh "$BUILD_DIR"/libemd_gaf* "$BUILD_DIR"/emd-gaf "$BUILD_DIR"/read_sensor 2>/dev/null

    echo ""
    echo "产物:"
    echo "  库:   $BUILD_DIR/libemd_gaf.so"
    echo "  示例: $BUILD_DIR/read_sensor"
    echo "  独立: $BUILD_DIR/emd-gaf"
    echo ""
    echo "运行示例:"
    echo "  sudo ./build/read_sensor -i /dev/i2c-3 -g gpiochip4 -l 2 -m 5"
else
    echo "=========================================="
    echo " emd-gaf 交叉编译"
    echo " Target: aarch64 (RV1126B)"
    echo "=========================================="

    mkdir -p "$BUILD_DIR"
    $CMAKE_CROSS -S "$PROJECT_DIR" -B "$BUILD_DIR" \
        -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN"

    $CMAKE_CROSS --build "$BUILD_DIR" -j"$(nproc)"

    echo ""
    echo "=========================================="
    echo " 编译完成!"
    echo "=========================================="
    echo ""

    ls -lh "$BUILD_DIR"/libemd_gaf* "$BUILD_DIR"/emd-gaf "$BUILD_DIR"/read_sensor 2>/dev/null

    echo ""
    echo "产物:"
    for f in "$BUILD_DIR"/libemd_gaf.so "$BUILD_DIR"/libemd_gaf.a "$BUILD_DIR"/emd-gaf "$BUILD_DIR"/read_sensor; do
        if [ -f "$f" ]; then
            echo "  $f"
            $READELF -h "$f" 2>/dev/null | grep -E "Machine|Class|Type" || true
        fi
    done

    echo ""
    echo "部署:"
    echo "  scp build/libemd_gaf.so     root@rv1126b:/usr/lib/"
    echo "  scp build/read_sensor       root@rv1126b:/usr/bin/"
    echo "  scp build/emd-gaf           root@rv1126b:/usr/bin/"
    echo ""
    echo "运行:"
    echo "  ssh root@rv1126b"
    echo "  rmmod inv-mpu-icm45600"
    echo "  read_sensor -i /dev/i2c-3 -g gpiochip4 -l 2 -m 5"
fi
