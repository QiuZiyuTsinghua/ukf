# 项目构建和使用指南

## 快速开始

### 1. 编译C++库
```bash
cd /home/ubuntu/code/cpp/ukf
./install.sh
```

### 2. 运行演示程序
```bash
cd build
./ukf
```

### 3. 编译Simulink S-Function (需要MATLAB)

```matlab
cd simulink
compile_truck_ukf_sfunc()
```

### 4. 测试S-Function
```matlab
test_truck_ukf_sfunc()
```

## 文件说明

### 核心文件
- `include/truck_ukf.h`: 半挂卡车UKF类声明
- `src/truck_ukf.cpp`: UKF实现，包含车辆动力学模型
- `simulink/truck_ukf_sfunc.cpp`: Simulink S-Function接口

### Simulink集成
- `compile_truck_ukf_sfunc.m`: MEX编译脚本
- `test_truck_ukf_sfunc.m`: 创建测试模型的脚本

### S-Function参数
在Simulink中设置参数: `[3.8, 15000, 10.0, 0.6, 0.0, 0.01, 0]`
分别对应: 轴距(m), 质量(kg), 迎风面积(m²), 风阻系数, 初始速度(m/s), 采样时间(s), 日志开关

### 输入信号 (7个)
1. 轮速传感器 (m/s)
2. 加速度计X (m/s²)
3. 加速度计Y (m/s²)
4. 陀螺仪 (rad/s)
5. 油门开度 (0-1)
6. 制动压力 (0-1)
7. 方向盘转角 (rad)

### 输出信号 (7个)
1. 估计车速 (m/s)
2. 估计位置X (m)
3. 估计位置Y (m)
4. 估计航向角 (rad)
5. 估计加速度X (m/s²)
6. 估计加速度Y (m/s²)
7. 速度不确定度 (m/s)

## TruckSim连接方案

1. 从TruckSim输出车辆传感器数据
2. 在Simulink中添加传感器噪声模拟
3. 连接到UKF S-Function进行状态估计
4. 输出估计结果用于控制算法或数据分析

完整的集成流程和详细说明请参考README.md文档。
