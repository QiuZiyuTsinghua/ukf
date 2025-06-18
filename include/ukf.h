#ifndef UKF_H
#define UKF_H

#include <vector>
#include <Eigen/Dense>

class UKF {
public:
    UKF();
    virtual ~UKF();
    
    // 初始化滤波器
    void initialize();
    
    // 预测步骤
    void predict(double delta_t);
    
    // 更新步骤
    void update(const Eigen::VectorXd& measurement);
    
private:
    // 状态向量
    Eigen::VectorXd x_;
    
    // 状态协方差矩阵
    Eigen::MatrixXd P_;
    
    // 过程噪声协方差矩阵
    Eigen::MatrixXd Q_;
    
    // 测量噪声协方差矩阵
    Eigen::MatrixXd R_;
    
    // 其他需要的参数
    double lambda_;
    int n_x_;
    int n_aug_;
    std::vector<double> weights_;
};

#endif // UKF_H
