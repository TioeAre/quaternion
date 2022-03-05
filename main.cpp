#include <iostream>
#include <fstream>
#include <random>
#include <chrono>
#include "kalman_autoaim.h"

#define PATH_DIR "D:/C/quaternion/"
#define PI_ 3.1415926535
extern bool IF_DEBUG;
using namespace std;
using namespace Eigen;

int main() {
    double x_ = 170, y_ = 200, z_ = 0, distance = 0, pitch = 0, yaw = 0;
    int i = 0;
    EKF_autoaim ekf;
    fstream out;
    fstream out_true;
    fstream out_yaw;
    fstream out_pitch;
    fstream out_pre_yaw;
    fstream out_pre_pitch;
    out.open(PATH_DIR"data/predict.txt");
    out_true.open(PATH_DIR"data/true.txt");

    out_pitch.open(PATH_DIR"data/true_pitch.txt");
    out_yaw.open(PATH_DIR"data/true_yaw.txt");
    out_pre_pitch.open(PATH_DIR"data/predict_pitch.txt");
    out_pre_yaw.open(PATH_DIR"data/predict_yaw.txt");
    while (i < 500) {
        //生成真实值数据
        i++;
        x_ += 5;
        y_ += 10;
        distance = sqrt(x_ * x_ + y_ * y_ + z_ * z_);
        pitch = atan2(z_, y_) * 180 / PI_;
        yaw = atan2(x_, y_) * 180 / PI_;
        if (IF_DEBUG) {
            cout << "true i:" << i << " x:" << x_ << " y:" << y_ << " z:" << z_ << " yaw_now:" << yaw << " pitch_now:"
                 << pitch << " distance_now:" << distance << endl;
        }
        out_true << yaw << "," << pitch << endl;
        out_pitch << pitch << endl;
        out_yaw << yaw << endl;
        //手动添加误差
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine generator(seed);
        std::normal_distribution<double> distribution(0.0, 20.0);
        x_ += distribution(generator);
        y_ += distribution(generator);
        z_ += distribution(generator);
        distance = sqrt(x_ * x_ + y_ * y_ + z_ * z_);
        pitch = atan2(z_, y_) * 180 / PI_;
        yaw = atan2(x_, y_) * 180 / PI_;
        if (IF_DEBUG) {
            cout << "error x:" << x_ << " y:" << y_ << " z:" << z_ << " yaw_now:" << yaw << " pitch_now:" << pitch
                 << endl;
        }
        //滤波器更新
        ekf.update(pitch, yaw, distance);
        ekf.return_pitch_yaw(pitch, yaw);
        //输出预测值到文件
        out << yaw << "," << pitch << endl;
        out_pre_pitch << pitch << endl;
        out_pre_yaw << yaw << endl;
    }
    return 0;
}
