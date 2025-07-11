#include <iostream>
#include <vector>
#include "ukf.h"

int main() {
    std::cout << "Unscented Kalman Filter Implementation" << std::endl;
    
    // 创建UKF实例
    UKF ukf;
    ukf.initialize();
    
    std::cout << "UKF initialized successfully!" << std::endl;
    
    // 模拟一些测量数据
    std::vector<Eigen::VectorXd> measurements;
    
    // 假设我们跟踪一个物体，这个物体以匀速直线运动
    for (int i = 0; i < 10; i++) {
        Eigen::VectorXd meas(2);
        meas << 0.5 * i, 0.5 * i;  // 物体沿着对角线移动
        measurements.push_back(meas);
    }
    
    // 运行滤波器
    double dt = 0.1;  // 时间步长
    for (const auto& meas : measurements) {
        // 预测
        ukf.predict(dt);
        
        // 更新
        ukf.update(meas);
        
        // 获取结果
        Eigen::VectorXd state = ukf.getState();
        Eigen::MatrixXd cov = ukf.getCovariance();
        
        std::cout << "State: " << state.transpose() << std::endl;
        std::cout << "Position uncertainty: " 
                  << std::sqrt(cov(0,0)) << ", " 
                  << std::sqrt(cov(1,1)) << std::endl;
    }
    
    return 0;
}
