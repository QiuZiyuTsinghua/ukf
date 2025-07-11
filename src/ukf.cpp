#include "ukf.h"
#include <iostream>
#include <cmath>

UKF::UKF() {
    // 默认状态维度
    n_x_ = 5;
    
    // 增广状态维度
    n_aug_ = 7;
    
    // 扩展参数
    lambda_ = 3 - n_aug_;
    
    // 初始化权重
    weights_.resize(2 * n_aug_ + 1);
    weights_[0] = lambda_ / (lambda_ + n_aug_);
    for (int i = 1; i < 2 * n_aug_ + 1; i++) {
        weights_[i] = 0.5 / (lambda_ + n_aug_);
    }
    
    // 初始化状态向量和协方差
    x_ = Eigen::VectorXd(n_x_);
    P_ = Eigen::MatrixXd(n_x_, n_x_);
    Q_ = Eigen::MatrixXd(2, 2);
    R_ = Eigen::MatrixXd(2, 2);
}

UKF::~UKF() {
    // 析构函数
}

void UKF::initialize() {
    // 初始化状态
    x_.setZero();
    
    // 初始化协方差矩阵
    P_.setIdentity();
    P_ *= 1.0;
    
    // 初始化过程噪声
    Q_ << 0.1, 0,
          0, 0.1;
    
    // 初始化测量噪声
    R_ << 0.1, 0,
          0, 0.1;
          
    std::cout << "UKF initialized with state dimension: " << n_x_ << std::endl;
}

Eigen::VectorXd UKF::getState() const {
    return x_;
}

Eigen::MatrixXd UKF::getCovariance() const {
    return P_;
}

void UKF::generateSigmaPoints(Eigen::MatrixXd& Xsig_aug) {
    // 创建增广状态向量
    Eigen::VectorXd x_aug = Eigen::VectorXd(n_aug_);
    x_aug.head(n_x_) = x_;
    x_aug(n_x_) = 0;
    x_aug(n_x_ + 1) = 0;
    
    // 创建增广协方差矩阵
    Eigen::MatrixXd P_aug = Eigen::MatrixXd(n_aug_, n_aug_);
    P_aug.fill(0.0);
    P_aug.topLeftCorner(n_x_, n_x_) = P_;
    P_aug.bottomRightCorner(2, 2) = Q_;
    
    // 计算平方根矩阵
    Eigen::MatrixXd L = P_aug.llt().matrixL();
    
    // 创建sigma点矩阵
    Xsig_aug.col(0) = x_aug;
    double sqrt_lambda_n_aug = std::sqrt(lambda_ + n_aug_);
    for (int i = 0; i < n_aug_; i++) {
        Xsig_aug.col(i + 1) = x_aug + sqrt_lambda_n_aug * L.col(i);
        Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt_lambda_n_aug * L.col(i);
    }
}

void UKF::predictSigmaPoints(const Eigen::MatrixXd& Xsig_aug, Eigen::MatrixXd& Xsig_pred, double delta_t) {
    // 对每个sigma点进行预测
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        // 提取状态值
        double px = Xsig_aug(0, i);
        double py = Xsig_aug(1, i);
        double v = Xsig_aug(2, i);
        double yaw = Xsig_aug(3, i);
        double yawd = Xsig_aug(4, i);
        double nu_a = Xsig_aug(5, i);
        double nu_yawdd = Xsig_aug(6, i);
        
        // 预测状态
        double px_pred, py_pred;
        
        // 避免除以零
        if (std::fabs(yawd) > 0.001) {
            px_pred = px + v / yawd * (std::sin(yaw + yawd * delta_t) - std::sin(yaw));
            py_pred = py + v / yawd * (std::cos(yaw) - std::cos(yaw + yawd * delta_t));
        } else {
            px_pred = px + v * delta_t * std::cos(yaw);
            py_pred = py + v * delta_t * std::sin(yaw);
        }
        
        double v_pred = v;
        double yaw_pred = yaw + yawd * delta_t;
        double yawd_pred = yawd;
        
        // 添加噪声
        px_pred += 0.5 * delta_t * delta_t * std::cos(yaw) * nu_a;
        py_pred += 0.5 * delta_t * delta_t * std::sin(yaw) * nu_a;
        v_pred += delta_t * nu_a;
        yaw_pred += 0.5 * delta_t * delta_t * nu_yawdd;
        yawd_pred += delta_t * nu_yawdd;
        
        // 填入预测矩阵
        Xsig_pred(0, i) = px_pred;
        Xsig_pred(1, i) = py_pred;
        Xsig_pred(2, i) = v_pred;
        Xsig_pred(3, i) = yaw_pred;
        Xsig_pred(4, i) = yawd_pred;
    }
}

void UKF::predictMeanAndCovariance(const Eigen::MatrixXd& Xsig_pred) {
    // 预测状态均值
    x_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        x_ += weights_[i] * Xsig_pred.col(i);
    }
    
    // 预测状态协方差
    P_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        Eigen::VectorXd x_diff = Xsig_pred.col(i) - x_;
        
        // 角度标准化
        while (x_diff(3) > M_PI) x_diff(3) -= 2.0 * M_PI;
        while (x_diff(3) < -M_PI) x_diff(3) += 2.0 * M_PI;
        
        P_ += weights_[i] * x_diff * x_diff.transpose();
    }
}

void UKF::predictMeasurement(const Eigen::MatrixXd& Xsig_pred, Eigen::VectorXd& z_pred, Eigen::MatrixXd& S, Eigen::MatrixXd& Zsig) {
    // 假设测量是位置 (px, py)
    int n_z = 2;
    
    // 变换sigma点到测量空间
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        double px = Xsig_pred(0, i);
        double py = Xsig_pred(1, i);
        
        Zsig(0, i) = px;
        Zsig(1, i) = py;
    }
    
    // 计算测量预测均值
    z_pred.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        z_pred += weights_[i] * Zsig.col(i);
    }
    
    // 计算测量预测协方差
    S.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        Eigen::VectorXd z_diff = Zsig.col(i) - z_pred;
        S += weights_[i] * z_diff * z_diff.transpose();
    }
    
    // 添加测量噪声
    S += R_;
}

void UKF::updateState(const Eigen::VectorXd& z, const Eigen::VectorXd& z_pred, const Eigen::MatrixXd& S, const Eigen::MatrixXd& Zsig, const Eigen::MatrixXd& Xsig_pred) {
    // 计算状态与测量的互协方差矩阵
    int n_z = z.size();
    Eigen::MatrixXd Tc = Eigen::MatrixXd(n_x_, n_z);
    Tc.fill(0.0);
    
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        Eigen::VectorXd x_diff = Xsig_pred.col(i) - x_;
        
        // 角度标准化
        while (x_diff(3) > M_PI) x_diff(3) -= 2.0 * M_PI;
        while (x_diff(3) < -M_PI) x_diff(3) += 2.0 * M_PI;
        
        Eigen::VectorXd z_diff = Zsig.col(i) - z_pred;
        
        Tc += weights_[i] * x_diff * z_diff.transpose();
    }
    
    // 计算卡尔曼增益
    Eigen::MatrixXd K = Tc * S.inverse();
    
    // 更新状态
    Eigen::VectorXd z_diff = z - z_pred;
    
    // 更新状态均值和协方差
    x_ = x_ + K * z_diff;
    P_ = P_ - K * S * K.transpose();
}

void UKF::predict(double delta_t) {
    std::cout << "Prediction step with dt = " << delta_t << std::endl;
    
    // 创建sigma点矩阵
    Eigen::MatrixXd Xsig_aug = Eigen::MatrixXd(n_aug_, 2 * n_aug_ + 1);
    generateSigmaPoints(Xsig_aug);
    
    // 预测sigma点
    Eigen::MatrixXd Xsig_pred = Eigen::MatrixXd(n_x_, 2 * n_aug_ + 1);
    predictSigmaPoints(Xsig_aug, Xsig_pred, delta_t);
    
    // 预测状态均值和协方差
    predictMeanAndCovariance(Xsig_pred);
}

void UKF::update(const Eigen::VectorXd& measurement) {
    std::cout << "Update step with measurement dimension: " << measurement.size() << std::endl;
    
    // 测量维度（假设是位置）
    int n_z = measurement.size();
    
    // 创建sigma点矩阵
    Eigen::MatrixXd Xsig_aug = Eigen::MatrixXd(n_aug_, 2 * n_aug_ + 1);
    generateSigmaPoints(Xsig_aug);
    
    // 预测sigma点
    Eigen::MatrixXd Xsig_pred = Eigen::MatrixXd(n_x_, 2 * n_aug_ + 1);
    // 使用很小的delta_t，因为我们只需要当前时刻的sigma点
    predictSigmaPoints(Xsig_aug, Xsig_pred, 0.0001);
    
    // 预测测量
    Eigen::VectorXd z_pred = Eigen::VectorXd(n_z);
    Eigen::MatrixXd S = Eigen::MatrixXd(n_z, n_z);
    Eigen::MatrixXd Zsig = Eigen::MatrixXd(n_z, 2 * n_aug_ + 1);
    
    predictMeasurement(Xsig_pred, z_pred, S, Zsig);
    
    // 更新状态
    updateState(measurement, z_pred, S, Zsig, Xsig_pred);
}
