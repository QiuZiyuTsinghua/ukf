#ifndef UKF_H
#define UKF_H

#include <vector>
#include <Eigen/Dense>

class UKF {
public:
    UKF();
    ~UKF();
    
    // 初始化滤波器
    void initialize();
    
    // 预测步骤
    void predict(double delta_t);
    
    // 更新步骤
    void update(const Eigen::VectorXd& measurement);
    
    // 获取当前状态估计
    Eigen::VectorXd getState() const;
    
    // 获取当前状态协方差
    Eigen::MatrixXd getCovariance() const;
    
private:
    // 生成sigma点
    void generateSigmaPoints(Eigen::MatrixXd& Xsig_aug);
    
    // 预测sigma点
    void predictSigmaPoints(const Eigen::MatrixXd& Xsig_aug, Eigen::MatrixXd& Xsig_pred, double delta_t);
    
    // 预测状态均值和协方差
    void predictMeanAndCovariance(const Eigen::MatrixXd& Xsig_pred);
    
    // 预测测量值
    void predictMeasurement(const Eigen::MatrixXd& Xsig_pred, Eigen::VectorXd& z_pred, Eigen::MatrixXd& S, Eigen::MatrixXd& Zsig);
    
    // 更新状态
    void updateState(const Eigen::VectorXd& z, const Eigen::VectorXd& z_pred, const Eigen::MatrixXd& S, const Eigen::MatrixXd& Zsig, const Eigen::MatrixXd& Xsig_pred);
    
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
