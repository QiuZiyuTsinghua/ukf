# 无迹卡尔曼滤波器 (UKF) 实现

这个项目实现了一个无迹卡尔曼滤波器，用于非线性系统的状态估计。

## 依赖项

- C++17 兼容的编译器
- CMake (>= 3.14)
- Eigen 库 (如未安装，将自动下载)
- Google Test (将自动下载)

## 构建项目

```bash
mkdir build
cd build
cmake ..
make
```

注意：首次构建时，CMake 会自动下载并编译 Eigen 和 Google Test 依赖库，可能需要一些时间。

## 项目构建说明

项目使用了模块化的构建系统：
- `ukf_lib` - 包含UKF核心实现的库
- `ukf` - 主可执行文件，链接到ukf_lib
- `ukf_test` - 测试可执行文件，链接到ukf_lib和Google Test

这种结构确保了代码在主应用和测试之间的一致性。

## 运行程序

```bash
./ukf
```

## 运行测试

```bash
make test
```

## 项目结构

- `include/` - 头文件
- `src/` - 源代码
- `tests/` - 测试代码
- `CMakeLists.txt` - CMake 构建配置
