#include "truck_ukf.h"
#include <iostream>
#include <cmath>

TruckUKF::TruckUKF() {
    // 状态维度 [x, y, v, yaw, yaw_rate, ax, ay]
    n_x_ = 7;
    
    // 增广状态维度 (添加过程噪声)
    n_aug_ = 9;
    
    // 测量维度 [wheel_speed, acc_x, acc_y, yaw_rate_sensor]
    n_z_ = 4;
    
    // UKF参数
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
    Q_ = Eigen::MatrixXd(2, 2);  // 过程噪声 [ax_noise, ay_noise]
    R_ = Eigen::MatrixXd(n_z_, n_z_);
    
    Xsig_pred_ = Eigen::MatrixXd(n_x_, 2 * n_aug_ + 1);
    
    // 车辆默认参数 (半挂卡车)
    wheelbase_ = 3.8;      // 轴距 (米)
    mass_ = 15000.0;       // 质量 (kg)
    front_area_ = 10.0;    // 迎风面积 (m²)
    drag_coeff_ = 0.6;     // 风阻系数
    
    is_initialized_ = false;
}

TruckUKF::~TruckUKF() {
    // 析构函数
}

void TruckUKF::initialize(double initial_speed) {
    // 初始化状态 [x, y, v, yaw, yaw_rate, ax, ay]
    x_ << 0.0, 0.0, initial_speed, 0.0, 0.0, 0.0, 0.0;
    
    // 初始化协方差矩阵
    P_.setIdentity();
    P_(0, 0) = 1.0;    // x position uncertainty
    P_(1, 1) = 1.0;    // y position uncertainty
    P_(2, 2) = 0.5;    // velocity uncertainty
    P_(3, 3) = 0.1;    // yaw uncertainty
    P_(4, 4) = 0.1;    // yaw rate uncertainty
    P_(5, 5) = 0.5;    // ax uncertainty
    P_(6, 6) = 0.5;    // ay uncertainty
    
    // 过程噪声协方差 [ax_noise, ay_noise]
    Q_ << 0.5, 0.0,
          0.0, 0.5;
    
    // 测量噪声协方差 [wheel_speed, acc_x, acc_y, yaw_rate]
    R_.setZero();
    R_(0, 0) = 0.1;    // wheel speed noise
    R_(1, 1) = 0.2;    // accelerometer x noise
    R_(2, 2) = 0.2;    // accelerometer y noise
    R_(3, 3) = 0.05;   // gyroscope noise
    
    is_initialized_ = true;
    
    std::cout << "Truck UKF initialized with initial speed: " << initial_speed << " m/s" << std::endl;
}

void TruckUKF::setVehicleParameters(double wheelbase, double mass, double front_area, double drag_coeff) {
    wheelbase_ = wheelbase;
    mass_ = mass;
    front_area_ = front_area;
    drag_coeff_ = drag_coeff;
}

double TruckUKF::getEstimatedSpeed() const {
    return x_(2);  // 返回速度状态
}

Eigen::VectorXd TruckUKF::getState() const {
    return x_;
}

Eigen::MatrixXd TruckUKF::getCovariance() const {
    return P_;
}

void TruckUKF::predict(double delta_t, double throttle, double brake, double steering_angle) {
    if (!is_initialized_) {
        std::cerr << "UKF not initialized!" << std::endl;
        return;
    }
    
    // 1. 生成sigma点
    Eigen::MatrixXd Xsig_aug = Eigen::MatrixXd(n_aug_, 2 * n_aug_ + 1);
    generateSigmaPoints(Xsig_aug);
    
    // 2. 预测sigma点
    predictSigmaPoints(Xsig_aug, Xsig_pred_, delta_t, throttle, brake, steering_angle);
    
    // 3. 预测状态均值和协方差
    predictMeanAndCovariance(Xsig_pred_);
}

void TruckUKF::update(double wheel_speed, double acc_x, double acc_y, double yaw_rate_measured) {
    if (!is_initialized_) {
        std::cerr << "UKF not initialized!" << std::endl;
        return;
    }
    
    // 测量向量
    Eigen::VectorXd z = Eigen::VectorXd(n_z_);
    z << wheel_speed, acc_x, acc_y, yaw_rate_measured;
    
    // 预测测量值
    Eigen::VectorXd z_pred = Eigen::VectorXd(n_z_);
    Eigen::MatrixXd S = Eigen::MatrixXd(n_z_, n_z_);
    Eigen::MatrixXd Zsig = Eigen::MatrixXd(n_z_, 2 * n_aug_ + 1);
    
    predictMeasurement(Xsig_pred_, z_pred, S, Zsig);
    
    // 更新状态
    updateState(z, z_pred, S, Zsig, Xsig_pred_);
}

void TruckUKF::generateSigmaPoints(Eigen::MatrixXd& Xsig_aug) {
    // 创建增广状态向量
    Eigen::VectorXd x_aug = Eigen::VectorXd(n_aug_);
    x_aug.head(n_x_) = x_;
    x_aug(n_x_) = 0;     // ax noise mean
    x_aug(n_x_ + 1) = 0; // ay noise mean
    
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

Eigen::VectorXd TruckUKF::vehicleDynamicsModel(const Eigen::VectorXd& state, double delta_t,
                                              double throttle, double brake, double steering_angle,
                                              double noise_ax, double noise_ay) {
    // 提取状态变量
    double x = state(0);
    double y = state(1);
    double v = state(2);
    double yaw = state(3);
    double yaw_rate = state(4);
    double ax = state(5);
    double ay = state(6);
    
    // 车辆动力学参数
    const double g = 9.81;
    const double air_density = 1.225;
    
    // 计算驱动力和制动力
    double max_drive_force = 8000.0;  // 最大驱动力 (N)
    double max_brake_force = 15000.0; // 最大制动力 (N)
    
    double drive_force = throttle * max_drive_force;
    double brake_force = brake * max_brake_force;
    
    // 计算空气阻力
    double air_resistance = 0.5 * air_density * drag_coeff_ * front_area_ * v * v;
    
    // 计算轮胎侧向力 (简化模型)
    double front_slip_angle = steering_angle - std::atan2(ay + yaw_rate * wheelbase_ / 2.0, v);
    double rear_slip_angle = -std::atan2(ay - yaw_rate * wheelbase_ / 2.0, v);
    
    double cornering_stiffness = 50000.0; // 轮胎侧偏刚度 (N/rad)
    double front_lateral_force = cornering_stiffness * front_slip_angle;
    double rear_lateral_force = cornering_stiffness * rear_slip_angle;
    
    // 计算加速度
    double net_force = drive_force - brake_force - air_resistance;
    double ax_new = net_force / mass_ + noise_ax;
    double ay_new = (front_lateral_force + rear_lateral_force) / mass_ + noise_ay;
    
    // 计算航向角加速度
    double yaw_accel = (front_lateral_force * wheelbase_ / 2.0) / (mass_ * wheelbase_ * wheelbase_ / 12.0);
    
    // 状态更新
    Eigen::VectorXd x_new = Eigen::VectorXd(n_x_);
    
    // 位置更新
    x_new(0) = x + v * cos(yaw) * delta_t;
    x_new(1) = y + v * sin(yaw) * delta_t;
    
    // 速度更新
    x_new(2) = v + ax * delta_t;
    if (x_new(2) < 0) x_new(2) = 0; // 防止倒车
    
    // 航向角更新
    x_new(3) = yaw + yaw_rate * delta_t;
    
    // 航向角速度更新
    x_new(4) = yaw_rate + yaw_accel * delta_t;
    
    // 加速度更新
    x_new(5) = ax_new;
    x_new(6) = ay_new;
    
    return x_new;
}

Eigen::VectorXd TruckUKF::measurementModel(const Eigen::VectorXd& state) {
    // 测量模型: [wheel_speed, acc_x, acc_y, yaw_rate_sensor]
    Eigen::VectorXd z_pred = Eigen::VectorXd(n_z_);
    
    double v = state(2);
    double yaw_rate = state(4);
    double ax = state(5);
    double ay = state(6);
    
    // 轮速 (假设没有滑移)
    z_pred(0) = v;
    
    // 加速度计测量 (包含重力影响)
    z_pred(1) = ax;
    z_pred(2) = ay;
    
    // 陀螺仪测量
    z_pred(3) = yaw_rate;
    
    return z_pred;
}

void TruckUKF::predictSigmaPoints(const Eigen::MatrixXd& Xsig_aug, Eigen::MatrixXd& Xsig_pred,
                                 double delta_t, double throttle, double brake, double steering_angle) {
    // 对每个sigma点进行预测
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        Eigen::VectorXd state = Xsig_aug.col(i).head(n_x_);
        double noise_ax = Xsig_aug(n_x_, i);
        double noise_ay = Xsig_aug(n_x_ + 1, i);
        
        Xsig_pred.col(i) = vehicleDynamicsModel(state, delta_t, throttle, brake, 
                                               steering_angle, noise_ax, noise_ay);
    }
}

void TruckUKF::predictMeanAndCovariance(const Eigen::MatrixXd& Xsig_pred) {
    // 预测状态均值
    x_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        x_ = x_ + weights_[i] * Xsig_pred.col(i);
    }
    
    // 预测状态协方差
    P_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        Eigen::VectorXd x_diff = Xsig_pred.col(i) - x_;
        
        // 角度归一化
        while (x_diff(3) > M_PI) x_diff(3) -= 2.0 * M_PI;
        while (x_diff(3) < -M_PI) x_diff(3) += 2.0 * M_PI;
        
        P_ = P_ + weights_[i] * x_diff * x_diff.transpose();
    }
}

void TruckUKF::predictMeasurement(const Eigen::MatrixXd& Xsig_pred, Eigen::VectorXd& z_pred,
                                 Eigen::MatrixXd& S, Eigen::MatrixXd& Zsig) {
    // 将sigma点转换为测量空间
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        Zsig.col(i) = measurementModel(Xsig_pred.col(i));
    }
    
    // 计算预测测量均值
    z_pred.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        z_pred = z_pred + weights_[i] * Zsig.col(i);
    }
    
    // 计算测量协方差矩阵S
    S.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        Eigen::VectorXd z_diff = Zsig.col(i) - z_pred;
        S = S + weights_[i] * z_diff * z_diff.transpose();
    }
    
    // 添加测量噪声
    S = S + R_;
}

void TruckUKF::updateState(const Eigen::VectorXd& z, const Eigen::VectorXd& z_pred,
                          const Eigen::MatrixXd& S, const Eigen::MatrixXd& Zsig,
                          const Eigen::MatrixXd& Xsig_pred) {
    // 计算交叉相关矩阵Tc
    Eigen::MatrixXd Tc = Eigen::MatrixXd(n_x_, n_z_);
    Tc.fill(0.0);
    
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        Eigen::VectorXd z_diff = Zsig.col(i) - z_pred;
        Eigen::VectorXd x_diff = Xsig_pred.col(i) - x_;
        
        // 角度归一化
        while (x_diff(3) > M_PI) x_diff(3) -= 2.0 * M_PI;
        while (x_diff(3) < -M_PI) x_diff(3) += 2.0 * M_PI;
        
        Tc = Tc + weights_[i] * x_diff * z_diff.transpose();
    }
    
    // 计算卡尔曼增益K
    Eigen::MatrixXd K = Tc * S.inverse();
    
    // 更新状态均值和协方差
    Eigen::VectorXd z_diff = z - z_pred;
    
    x_ = x_ + K * z_diff;
    P_ = P_ - K * S * K.transpose();
}
