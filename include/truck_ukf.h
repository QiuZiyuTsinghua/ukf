#ifndef TRUCK_UKF_H
#define TRUCK_UKF_H

#include <Eigen/Dense>
#include <vector>

/**
 * @brief 半挂卡车UKF车速估计器
 * 
 * 状态向量: [x, y, v, yaw, yaw_rate, ax, ay] (7维)
 * - x, y: 位置
 * - v: 纵向速度 (主要估计目标)
 * - yaw: 航向角
 * - yaw_rate: 航向角速度
 * - ax, ay: 加速度
 * 
 * 测量向量: [wheel_speed, acc_x, acc_y, yaw_rate_sensor] (4维)
 */
class TruckUKF {
public:
    TruckUKF();
    ~TruckUKF();
    
    // 初始化滤波器
    void initialize(double initial_speed = 0.0);
    
    // 预测步骤
    void predict(double delta_t, double throttle, double brake, double steering_angle);
    
    // 更新步骤 - 输入测量值
    void update(double wheel_speed, double acc_x, double acc_y, double yaw_rate_measured);
    
    // 获取估计的车速
    double getEstimatedSpeed() const;
    
    // 获取完整状态估计
    Eigen::VectorXd getState() const;
    
    // 获取状态协方差
    Eigen::MatrixXd getCovariance() const;
    
    // 设置车辆参数
    void setVehicleParameters(double wheelbase, double mass, double front_area, double drag_coeff);
    
private:
    // UKF核心函数
    void generateSigmaPoints(Eigen::MatrixXd& Xsig_aug);
    void predictSigmaPoints(const Eigen::MatrixXd& Xsig_aug, Eigen::MatrixXd& Xsig_pred, 
                           double delta_t, double throttle, double brake, double steering_angle);
    void predictMeanAndCovariance(const Eigen::MatrixXd& Xsig_pred);
    void predictMeasurement(const Eigen::MatrixXd& Xsig_pred, Eigen::VectorXd& z_pred, 
                           Eigen::MatrixXd& S, Eigen::MatrixXd& Zsig);
    void updateState(const Eigen::VectorXd& z, const Eigen::VectorXd& z_pred, 
                    const Eigen::MatrixXd& S, const Eigen::MatrixXd& Zsig, 
                    const Eigen::MatrixXd& Xsig_pred);
    
    // 车辆动力学模型
    Eigen::VectorXd vehicleDynamicsModel(const Eigen::VectorXd& state, double delta_t,
                                        double throttle, double brake, double steering_angle,
                                        double noise_ax, double noise_ay);
    
    // 测量模型
    Eigen::VectorXd measurementModel(const Eigen::VectorXd& state);
    
    // 状态向量 [x, y, v, yaw, yaw_rate, ax, ay]
    Eigen::VectorXd x_;
    
    // 状态协方差矩阵
    Eigen::MatrixXd P_;
    
    // 过程噪声协方差矩阵
    Eigen::MatrixXd Q_;
    
    // 测量噪声协方差矩阵
    Eigen::MatrixXd R_;
    
    // UKF参数
    double lambda_;
    int n_x_;        // 状态维度
    int n_aug_;      // 增广状态维度
    int n_z_;        // 测量维度
    std::vector<double> weights_;
    
    // 车辆参数
    double wheelbase_;       // 轴距
    double mass_;           // 质量
    double front_area_;     // 迎风面积
    double drag_coeff_;     // 风阻系数
    
    // 预测的sigma点
    Eigen::MatrixXd Xsig_pred_;
    
    // 标志位
    bool is_initialized_;
};

#endif // TRUCK_UKF_H
