#!/bin/bash
#==============================================================================
# emd-gaf 交叉编译脚本
# 用法: ./build.sh [build|clean]
#   build - 交叉编译 (默认, 先清理再编译)
#   clean - 仅清理构建目录
#==============================================================================
set -e

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
TOOLCHAIN="$PROJECT_DIR/toolchain.cmake"
CMAKE="/opt/gcc-arm-12.4-x86_64-aarch64-linux-gnu/bin/cmake"
READELF="/opt/gcc-arm-12.4-x86_64-aarch64-linux-gnu/bin/aarch64-buildroot-linux-gnu-readelf"

BUILD_DIR="$PROJECT_DIR/build"
TARGET="emd-gaf"

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
    build)
        ;;
    *)
        echo "用法: $0 [build|clean]"
        echo "  build - 交叉编译 (默认, 清理后编译)"
        echo "  clean - 仅清理构建目录"
        exit 1
        ;;
esac

#==============================================================================
# 编译前清理
#==============================================================================
_clean

#==============================================================================
# 交叉编译
#==============================================================================
echo "=========================================="
echo " emd-gaf 交叉编译"
echo " Target: aarch64 (RV1126B)"
echo "=========================================="

mkdir -p "$BUILD_DIR"
$CMAKE -S "$PROJECT_DIR" -B "$BUILD_DIR" \
    -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN"

$CMAKE --build "$BUILD_DIR" -j"$(nproc)"

#==============================================================================
# 结果
#==============================================================================
echo ""
echo "=========================================="
echo " 编译完成!"
echo "=========================================="
echo ""

ls -lh "$BUILD_DIR/$TARGET"

echo ""
echo "架构信息:"
$READELF -h "$BUILD_DIR/$TARGET" 2>/dev/null | grep -E "Machine|Class|Type" || true

echo ""
echo "产物: $BUILD_DIR/$TARGET"
echo ""
echo "部署:"
echo "  scp $BUILD_DIR/$TARGET root@rv1126b:/usr/bin/"
echo ""
echo "运行:"
echo "  ssh root@rv1126b"
echo "  rmmod inv-mpu-icm45600"
echo "  emd-gaf -i /dev/i2c-3 -g gpiochip4 -l 2 -m 5"
