# 半挂卡车UKF车速估计系统

## 项目概述

本项目实现了一个基于无迹卡尔曼滤波器(UKF)的半挂卡车车速估计系统，专门用于Simulink与TruckSim的联合仿真。系统通过融合轮速传感器、加速度计和陀螺仪等多传感器数据，提供精确的车辆状态估计。

## 目录结构

```
ukf/
├── README.md                          # 本文档
├── CMakeLists.txt                     # CMake构建配置
├── include/                           # 头文件目录
│   ├── ukf.h                         # 通用UKF类声明
│   └── truck_ukf.h                   # 半挂卡车UKF类声明
├── src/                              # 源代码目录
│   ├── main.cpp                      # 主程序入口
│   ├── ukf.cpp                       # 通用UKF实现
│   └── truck_ukf.cpp                 # 半挂卡车UKF实现
├── simulink/                         # Simulink集成文件
│   ├── truck_ukf_sfunc.cpp           # S-Function源代码
│   ├── compile_truck_ukf_sfunc.m     # MEX编译脚本
│   └── test_truck_ukf_sfunc.m        # S-Function测试脚本
├── tests/                            # 单元测试
│   ├── CMakeLists.txt
│   └── ukf_test.cpp
└── build/                            # 构建输出目录
    ├── ukf                           # 可执行文件
    ├── libukf_lib.a                  # 静态库
    └── tests/
        └── ukf_test                  # 测试可执行文件
```

## 功能特性

### 1. 车辆动力学模型

系统采用7维状态向量描述半挂卡车运动：

**状态向量**: `[x, y, v, yaw, yaw_rate, ax, ay]`
- `x, y`: 车辆位置坐标 (m)
- `v`: 纵向速度 (m/s) - **主要估计目标**
- `yaw`: 航向角 (rad)
- `yaw_rate`: 航向角速度 (rad/s)
- `ax, ay`: 纵向和侧向加速度 (m/s²)

**动力学方程**:
```
ẋ = v·cos(yaw)
ẏ = v·sin(yaw)
v̇ = (F_drive - F_brake - F_drag)/m
ψ̇ = yaw_rate
ψ̈ = (F_front·L_f)/(I_z)
äx = f(throttle, brake, drag, mass)
äy = f(steering, slip_angles, tire_forces)
```

其中：
- `F_drive`: 驱动力 (N)
- `F_brake`: 制动力 (N)  
- `F_drag`: 空气阻力 = 0.5·ρ·Cd·A·v² (N)
- `m`: 车辆质量 (kg)
- `L_f`: 前轴到重心距离 (m)
- `I_z`: 转动惯量 (kg·m²)

### 2. 传感器测量模型

**测量向量**: `[wheel_speed, acc_x, acc_y, yaw_rate_sensor]`

```
z_wheel_speed = v + n_wheel
z_acc_x = ax + n_acc_x
z_acc_y = ay + n_acc_y  
z_yaw_rate = yaw_rate + n_yaw_rate
```

其中 `n_*` 表示各传感器的测量噪声。

### 3. UKF算法流程

1. **初始化**: 设置初始状态、协方差矩阵和噪声参数
2. **预测步骤**:
   - 生成sigma点
   - 通过动力学模型传播sigma点
   - 计算预测状态均值和协方差
3. **更新步骤**:
   - 预测测量值
   - 计算卡尔曼增益
   - 更新状态估计和协方差

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
