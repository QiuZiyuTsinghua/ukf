#include "truck_ukf.h"
#include <iostream>
#include <iomanip>
#include <cmath>

int main() {
    std::cout << "半挂卡车UKF车速估计演示\n";
    std::cout << "========================\n\n";
    
    // 创建UKF实例
    TruckUKF ukf;
    
    // 设置车辆参数
    ukf.setVehicleParameters(3.8, 15000.0, 10.0, 0.6);
    
    // 初始化滤波器
    ukf.initialize(5.0);  // 初始速度 5 m/s
    
    std::cout << "UKF已初始化，初始速度: 5.0 m/s\n";
    std::cout << "开始仿真循环...\n\n";
    
    // 仿真参数
    double dt = 0.01;  // 采样时间 10ms
    int num_steps = 1000;  // 仿真步数
    
    // 模拟输入信号
    double throttle = 0.3;
    double brake = 0.0;
    double steering = 0.0;
    
    // 输出表头
    std::cout << std::setw(8) << "Time(s)" 
              << std::setw(12) << "WheelSpd(m/s)" 
              << std::setw(12) << "EstSpd(m/s)"
              << std::setw(12) << "AccX(m/s²)"
              << std::setw(12) << "AccY(m/s²)"
              << std::setw(12) << "YawRate(rad/s)"
              << std::setw(12) << "Uncert(m/s)" << std::endl;
    std::cout << std::string(88, '-') << std::endl;
    
    for (int i = 0; i < num_steps; i++) {
        double time = i * dt;
        
        // 生成模拟传感器数据
        double true_speed = 5.0 + 2.0 * time;  // 线性加速
        double wheel_speed = true_speed + 0.1 * sin(time * 10);  // 轮速噪声
        double acc_x = 2.0 + 0.2 * sin(time * 5);  // 纵向加速度
        double acc_y = 0.1 * sin(time * 3);  // 侧向加速度 
        double yaw_rate = 0.05 * sin(time * 2);  // 航向角速度
        
        // UKF预测步骤
        ukf.predict(dt, throttle, brake, steering);
        
        // UKF更新步骤
        ukf.update(wheel_speed, acc_x, acc_y, yaw_rate);
        
        // 获取估计结果
        double estimated_speed = ukf.getEstimatedSpeed();
        Eigen::VectorXd state = ukf.getState();
        Eigen::MatrixXd covariance = ukf.getCovariance();
        double uncertainty = sqrt(covariance(2, 2));
        
        // 每100步输出一次结果
        if (i % 100 == 0) {
            std::cout << std::setw(8) << std::fixed << std::setprecision(2) << time
                      << std::setw(12) << std::setprecision(3) << wheel_speed
                      << std::setw(12) << std::setprecision(3) << estimated_speed
                      << std::setw(12) << std::setprecision(3) << state(5)  // ax
                      << std::setw(12) << std::setprecision(3) << state(6)  // ay
                      << std::setw(12) << std::setprecision(4) << state(4)  // yaw_rate
                      << std::setw(12) << std::setprecision(4) << uncertainty
                      << std::endl;
        }
    }
    
    std::cout << "\n仿真完成!\n";
    std::cout << "最终估计状态:\n";
    
    Eigen::VectorXd final_state = ukf.getState();
    std::cout << "  位置 X: " << std::setprecision(2) << final_state(0) << " m\n";
    std::cout << "  位置 Y: " << std::setprecision(2) << final_state(1) << " m\n";
    std::cout << "  速度:   " << std::setprecision(3) << final_state(2) << " m/s\n";
    std::cout << "  航向角: " << std::setprecision(3) << final_state(3) << " rad\n";
    std::cout << "  角速度: " << std::setprecision(4) << final_state(4) << " rad/s\n";
    std::cout << "  加速度X: " << std::setprecision(3) << final_state(5) << " m/s²\n";
    std::cout << "  加速度Y: " << std::setprecision(3) << final_state(6) << " m/s²\n";
    
    std::cout << "\n程序运行成功！\n";
    std::cout << "现在可以编译Simulink S-Function进行集成仿真。\n";
    
    return 0;
}
