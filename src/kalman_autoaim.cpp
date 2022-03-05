//
// Created by TioeAre on 2022年2月28日.
//
///sin用的是弧度值*pi/180
///atan2返回的弧度制*180/pi
#include <kalman_autoaim.h>

#define PI 3.1415926535
//bool IF_DEBUG = true;
bool IF_DEBUG = false;

EKF_autoaim::EKF_autoaim() {
    {
        hat_x.resize(4);
        hat_x.setZero();
        measure_x.resize(4);
        measure_x.setZero();
        control_A.resize(4, 4);
        control_A.setZero();
        control_H.resize(4, 4);
        kalman_gain.resize(4, 4);
        kalman_gain.setZero();
        kalman_gain_restraint.resize(4, 4);
        kalman_gain_restraint.setZero();
        error_P.resize(4, 4);
        error_Q.resize(4, 4);
        error_R.resize(4, 4);
        error_R_restraint.resize(1, 1);
        error_v.resize(4, 4);
        error_w.resize(4, 4);
        yaw_pre = 0;
        pitch_pre = 1.7453;
        distance_pre = -500;
        if_change = true;
        time_ = 1;
    }
    {
        control_H << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;
        error_P << 0.0464093, -0.314546, -0.133175, 0.0577939,
                -0.314546, 4.21262, 0.66347, -0.032676,
                -0.133175, 0.663471, 1.20085, 0.0114497,
                0.0577939, -0.0326761, 0.0114497, 4.00076;//此处调参
        error_Q << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;//此处调参
        error_R << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;//此处调参
        error_v << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;//此处调参
        error_w << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;//此处调参
        error_R_restraint << 1;//此处调参
    }
}

/// 注意视觉定义的坐标系与陀螺仪的坐标系稍有不同
void EKF_autoaim::transform(const double &pitch, const double &yaw) {
    //转化为弧度值
    double pitch_ = pitch / 180 * PI;
    double yaw_ = yaw / 180 * PI;
    double roll_ = 0;
    if (if_change) {
        hat_x(0) = cos(roll_ / 2) * cos(pitch_ / 2) * cos(yaw_ / 2) - sin(roll_ / 2) * sin(pitch_ / 2) * sin(yaw_ / 2);
        hat_x(1) = cos(roll_ / 2) * cos(pitch_ / 2) * sin(yaw_ / 2) + sin(roll_ / 2) * sin(pitch_ / 2) * cos(yaw_ / 2);
        hat_x(2) = cos(roll_ / 2) * sin(pitch_ / 2) * cos(yaw_ / 2) - sin(roll_ / 2) * cos(pitch_ / 2) * sin(yaw_ / 2);
        hat_x(3) = cos(roll_ / 2) * sin(pitch_ / 2) * sin(yaw_ / 2) + sin(roll_ / 2) * cos(pitch_ / 2) * cos(yaw_ / 2);
    }
    measure_x(0) = cos(roll_ / 2) * cos(pitch_ / 2) * cos(yaw_ / 2) - sin(roll_ / 2) * sin(pitch_ / 2) * sin(yaw_ / 2);
    measure_x(1) = cos(roll_ / 2) * cos(pitch_ / 2) * sin(yaw_ / 2) + sin(roll_ / 2) * sin(pitch_ / 2) * cos(yaw_ / 2);
    measure_x(2) = cos(roll_ / 2) * sin(pitch_ / 2) * cos(yaw_ / 2) - sin(roll_ / 2) * cos(pitch_ / 2) * sin(yaw_ / 2);
    measure_x(3) = cos(roll_ / 2) * sin(pitch_ / 2) * sin(yaw_ / 2) + sin(roll_ / 2) * cos(pitch_ / 2) * cos(yaw_ / 2);
}

void EKF_autoaim::get_measure(const double &pitch, const double &yaw, const double &distance) {
    if_change = false;
    if (distance - distance_pre > 500) {
        if_change = true;
    }
    transform(pitch, yaw);
    distance_pre = distance;
}

/// 角速度是按弧度制算的角速度
void EKF_autoaim::get_A(const double &pitch, const double &yaw) {
    //转化为弧度值
    double omega_x = (pitch / 180 * PI - pitch_pre / 180 * PI) / time_;
    double omega_y = 0;
    double omega_z = (yaw / 180 * PI - yaw_pre / 180 * PI) / time_;
    MatrixXd omega;
    omega.resize(4, 4);
    omega << 0, -omega_z, -omega_x, -omega_y,
            omega_z, 0, omega_y, -omega_x,
            omega_x, -omega_y, 0, omega_z,
            omega_y, omega_x, -omega_z, 0;
    control_A = MatrixXd::Identity(4, 4) + 0.5 * omega * time_;
    if (IF_DEBUG) {
        cout << "control_A:\n" << control_A << endl;
    }
}

void EKF_autoaim::get_x_() {
    hat_x = control_A * hat_x;
}

void EKF_autoaim::get_p_() {
    error_P = control_A * error_P * control_A.transpose() + error_w * error_Q * error_w.transpose();
}

void EKF_autoaim::update(const double &pitch_, const double &yaw_, const double &distance_) {
    get_measure(pitch_, yaw_, distance_);
    if (IF_DEBUG) {
        cout << "measure_x:\n" << measure_x << endl;
        cout << "pre-hat_x:\n" << hat_x << endl;
    }
    if (!if_change) {   ///判断是否为第一次滤波或目标装甲板是否发生转变是否
        get_A(pitch_, yaw_);
        get_x_();
        get_p_();
        if (IF_DEBUG) {
            cout << "hat_x_:\n" << hat_x << " \nerror_p:\n" << error_P << endl << endl;
        }
        MatrixXd k_1(2, 2);
        k_1 = error_P * control_H.transpose();
        MatrixXd k_2(2, 2);
        k_2 = control_H * error_P * control_H.transpose() + error_v * error_R * error_v.transpose();
        kalman_gain = k_1 * k_2.inverse();
        hat_x = hat_x + kalman_gain * (measure_x - control_H * hat_x);
        error_P = (MatrixXd::Identity(4, 4) - kalman_gain * control_H) * error_P;
        if (IF_DEBUG) {
            cout << "1-hat_x:\n" << hat_x << " \nerror_p:\n" << error_P << " \nkalman_gain:\n" << kalman_gain << endl
                 << endl;
        }
        /// 对卡尔曼滤波进行约束
        restraint_update();
        hat_x = hat_x + kalman_gain_restraint *
                        (1 - (hat_x(0) * hat_x(0) + hat_x(1) * hat_x(1) + hat_x(2) * hat_x(2) + hat_x(3) * hat_x(3)));
        if (IF_DEBUG) {
            cout << "2-hat_x:\n" << hat_x << " \nerror_p:\n" << error_P << " \nkalman_gain:\n" << kalman_gain << endl
                 << endl;
        }
        yaw_pre = yaw_;
        pitch_pre = pitch_;
    } else {
        yaw_pre = yaw_;
        pitch_pre = pitch_;
    }
}

void EKF_autoaim::restraint_update() {
    VectorXd control_A_;
    control_A_.resize(4);
    control_A_ << (2 * hat_x(0)), (2 * hat_x(1)), (2 * hat_x(2)), (2 * hat_x(3));
    kalman_gain_restraint = (error_P * control_A_)
                            * ((control_A_.transpose() * error_P * control_A_) + error_R_restraint).inverse();
    error_P = (MatrixXd::Identity(4, 4) - kalman_gain_restraint * control_A_.transpose()) * error_P;
}

void EKF_autoaim::return_pitch_yaw(double &pitch, double &yaw) {
    if (!if_change) {
        MatrixXd return_ = control_A * hat_x;
        if (IF_DEBUG) {
            cout << "return_:\n" << return_ << endl;
        }

        yaw = atan2(2 * return_(0) * return_(1) - (return_(2) * return_(3)),
                    (return_(0) * return_(0) - return_(1) * return_(1) - return_(2) * return_(2) +
                     return_(3) * return_(3)));
        pitch = asin(2 * (return_(0) * return_(2) + return_(1) * return_(3)));
        yaw = yaw * 180 / PI;
        pitch = pitch * 180 / PI;
    }
}