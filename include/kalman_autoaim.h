//
// Created by TioeAre on 2022年2月28日.
//

#ifndef KALMAN_AUTOAIM_H_
#define KALMAN_AUTOAIM_H_

#include <iostream>
#include <stdlib.h>
#include "Dense"

using namespace std;
using namespace Eigen;

class EKF_autoaim {
private:
    VectorXd hat_x;            //后验四元数
    VectorXd measure_x;        //测量出的四元数
    MatrixXd control_A;        //状态转移矩阵A 会随着预测过程改变
    MatrixXd control_H;        //控制矩阵H 为计算过程中的不变值
    MatrixXd kalman_gain;    //卡尔曼增益
    MatrixXd kalman_gain_restraint; //约束卡尔曼滤波中的卡尔曼增益
    MatrixXd error_P;        //后验误差协方差矩阵
    MatrixXd error_Q;        //过程误差协方差矩阵Q 为计算过程中的不变值
    MatrixXd error_R;        //测量误差协方差矩阵R 为计算过程中的不变值
    MatrixXd error_R_restraint;//状态约束误差R
    MatrixXd error_v;        //测量误差v的雅各比矩阵系数,为计算过程中的不变值
    MatrixXd error_w;        //系统误差w的雅各比矩阵系数,为计算过程中的不变值
    double time_;    //滤波间隔时间
    double yaw_pre; //上一刻的yaw轴角度，应接受角度制的yaw
    double pitch_pre;   //上一刻的pitch轴角度，应接受角度制的pitch
    double distance_pre;    //上一时刻目标装甲板的距离
    bool if_change;  //是否为第一次滤波或目标是否改变

    /*
    * @brief 将欧拉角转为四元数
    * @input 接受当前pitch,yaw的角度制
    * */
    void transform(const double &pitch, const double &yaw);

    /*
     * @brief 为观测值赋值
     * @input 接受当前pitch,yaw的角度制
     * */
    void get_measure(const double &pitch, const double &yaw, const double &distance);

    /*
    * @brief 更新状态转移矩阵A
    */
    void get_A(const double &pitch, const double &yaw);

    /*
    * @brief 获取先验值
    */
    void get_x_();

    /*
    * @brief 获取协方差矩阵先验
    */
    void get_p_();

    /*
    * @brief 约束滤波器
    */
    void restraint_update();

public:
    EKF_autoaim();

    ~EKF_autoaim() {};

    /*
     * @brief 滤波器更新
     * @input 接受当前pitch,yaw的角度制，distance为目标装甲板的距离以判断是否变更目标
     * */
    void update(const double &pitch, const double &yaw, const double &distance);

    /*
    * @brief 返回预测值
    * @input pitch,yaw:从外部接受当前角度值，实际作用充当返回值
    */
    void return_pitch_yaw(double &pitch, double &yaw);
};

#endif //KALMAN_AUTOAIM_H_