#!/bin/bash
#==============================================================================
# emd-gaf 交叉编译脚本
# 用法: ./build.sh [build|clean|rebuild]
#   build   - 交叉编译 (默认)
#   clean   - 仅清理构建目录
#   rebuild - 清理后重新编译
#==============================================================================
set -e

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
TOOLCHAIN="$PROJECT_DIR/toolchain.cmake"
CMAKE="/opt/gcc-arm-12.4-x86_64-aarch64-linux-gnu/bin/cmake"

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
    rebuild)
        _clean
        ;;
    build)
        ;;
    *)
        echo "用法: $0 [build|clean|rebuild]"
        echo "  build   - 交叉编译 (默认)"
        echo "  clean   - 仅清理构建目录"
        echo "  rebuild - 清理后重新编译"
        exit 1
        ;;
esac

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
# 输出
#==============================================================================
echo ""
echo "=========================================="
echo " 编译完成"
echo "=========================================="
ls -lh "$BUILD_DIR/$TARGET"
file "$BUILD_DIR/$TARGET"
echo ""
echo "产物: $BUILD_DIR/$TARGET"
echo "部署: scp $BUILD_DIR/$TARGET root@rv1126b:/usr/bin/"
echo ""
echo "运行:"
echo "  ssh root@rv1126b"
echo "  rmmod inv-mpu-icm45600  # 卸载 IIO 驱动"
echo "  emd-gaf -i /dev/i2c-3 -g gpiochip4 -l 2 -m 5"
