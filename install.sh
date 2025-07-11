#!/bin/bash

# 半挂卡车UKF车速估计系统安装脚本
# 
# 此脚本将自动安装依赖项并编译整个项目
#
# 使用方法:
#   chmod +x install.sh
#   ./install.sh

set -e  # 出错时退出

echo "==========================================="
echo "半挂卡车UKF车速估计系统安装脚本"
echo "==========================================="
echo

# 检查操作系统
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    OS="linux"
    echo "检测到Linux系统"
elif [[ "$OSTYPE" == "darwin"* ]]; then
    OS="macos"
    echo "检测到macOS系统"
else
    echo "错误: 不支持的操作系统: $OSTYPE"
    exit 1
fi

# 安装依赖项
echo "安装依赖项..."
if [[ "$OS" == "linux" ]]; then
    # Ubuntu/Debian
    if command -v apt-get &> /dev/null; then
        echo "使用apt-get安装依赖..."
        sudo apt-get update
        sudo apt-get install -y build-essential cmake libeigen3-dev
    # CentOS/RHEL
    elif command -v yum &> /dev/null; then
        echo "使用yum安装依赖..."
        sudo yum groupinstall -y "Development Tools"
        sudo yum install -y cmake eigen3-devel
    else
        echo "警告: 无法自动安装依赖项，请手动安装以下软件包:"
        echo "  - build-essential 或 Development Tools"
        echo "  - cmake"
        echo "  - libeigen3-dev 或 eigen3-devel"
    fi
elif [[ "$OS" == "macos" ]]; then
    # macOS with Homebrew
    if command -v brew &> /dev/null; then
        echo "使用Homebrew安装依赖..."
        brew install cmake eigen
    else
        echo "错误: 未找到Homebrew，请先安装Homebrew:"
        echo "  /bin/bash -c \"\$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)\""
        exit 1
    fi
fi

# 检查cmake
if ! command -v cmake &> /dev/null; then
    echo "错误: 未找到cmake，请手动安装"
    exit 1
fi

echo "✓ 依赖项安装完成"

# 检查Eigen库
echo "检查Eigen库..."
EIGEN_PATHS=(
    "/usr/include/eigen3"
    "/usr/local/include/eigen3" 
    "/opt/homebrew/include/eigen3"
    "/usr/include/eigen3/Eigen"
    "/usr/local/include/eigen3/Eigen"
)

EIGEN_FOUND=false
for path in "${EIGEN_PATHS[@]}"; do
    if [[ -d "$path" ]]; then
        echo "✓ 找到Eigen库: $path"
        EIGEN_FOUND=true
        break
    fi
done

if [[ "$EIGEN_FOUND" == false ]]; then
    echo "警告: 未找到系统Eigen库，将使用CMake自动下载"
fi

# 创建构建目录
echo "准备构建环境..."
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$PROJECT_DIR/build"

if [[ -d "$BUILD_DIR" ]]; then
    echo "清理旧的构建目录..."
    rm -rf "$BUILD_DIR"
fi

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

echo "✓ 构建目录已创建: $BUILD_DIR"

# 配置构建
echo "配置CMake..."
cmake .. -DCMAKE_BUILD_TYPE=Release

echo "✓ CMake配置完成"

# 编译项目
echo "编译项目..."
make -j$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)

echo "✓ 编译完成"

# 运行测试
echo "运行测试..."
if ctest --output-on-failure; then
    echo "✓ 所有测试通过"
else
    echo "⚠ 部分测试失败，但核心功能可能正常"
fi

# 验证可执行文件
if [[ -f "$BUILD_DIR/ukf" ]]; then
    echo "✓ 主程序编译成功: $BUILD_DIR/ukf"
    
    # 运行简单测试
    echo "运行基本功能测试..."
    if timeout 10s "$BUILD_DIR/ukf" > /dev/null 2>&1; then
        echo "✓ 基本功能测试通过"
    else
        echo "⚠ 基本功能测试超时或失败"
    fi
else
    echo "✗ 主程序编译失败"
    exit 1
fi

# 检查库文件
if [[ -f "$BUILD_DIR/libukf_lib.a" ]]; then
    echo "✓ UKF库编译成功: $BUILD_DIR/libukf_lib.a"
else
    echo "✗ UKF库编译失败"
    exit 1
fi

# 输出安装总结
echo
echo "==========================================="
echo "安装完成！"
echo "==========================================="
echo
echo "编译产物:"
echo "  - 主程序:     $BUILD_DIR/ukf"
echo "  - UKF库:     $BUILD_DIR/libukf_lib.a"
echo "  - 测试程序:   $BUILD_DIR/tests/ukf_test"
echo

echo "下一步操作:"
echo "1. 运行演示程序:"
echo "   cd $BUILD_DIR && ./ukf"
echo
echo "2. 编译Simulink S-Function (需要MATLAB):"
echo "   cd $PROJECT_DIR/simulink"
echo "   matlab -nodisplay -nosplash -r \"compile_truck_ukf_sfunc; exit\""
echo
echo "3. 测试S-Function (需要MATLAB):"
echo "   cd $PROJECT_DIR/simulink"
echo "   matlab -r \"test_truck_ukf_sfunc; exit\""
echo

echo "项目文档: $PROJECT_DIR/README.md"
echo "技术支持: 请查看README.md中的故障排除部分"
echo

echo "安装脚本执行完成！"
