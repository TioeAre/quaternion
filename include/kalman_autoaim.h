//
// Created by TioeAre on 2022��2��28��.
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
    VectorXd hat_x;            //������Ԫ��
    VectorXd measure_x;        //����������Ԫ��
    MatrixXd control_A;        //״̬ת�ƾ���A ������Ԥ����̸ı�
    MatrixXd control_H;        //���ƾ���H Ϊ��������еĲ���ֵ
    MatrixXd kalman_gain;    //����������
    MatrixXd kalman_gain_restraint; //Լ���������˲��еĿ���������
    MatrixXd error_P;        //�������Э�������
    MatrixXd error_Q;        //�������Э�������Q Ϊ��������еĲ���ֵ
    MatrixXd error_R;        //�������Э�������R Ϊ��������еĲ���ֵ
    MatrixXd error_R_restraint;//״̬Լ�����R
    MatrixXd error_v;        //�������v���Ÿ��Ⱦ���ϵ��,Ϊ��������еĲ���ֵ
    MatrixXd error_w;        //ϵͳ���w���Ÿ��Ⱦ���ϵ��,Ϊ��������еĲ���ֵ
    double time_;    //�˲����ʱ��
    double yaw_pre; //��һ�̵�yaw��Ƕȣ�Ӧ���ܽǶ��Ƶ�yaw
    double pitch_pre;   //��һ�̵�pitch��Ƕȣ�Ӧ���ܽǶ��Ƶ�pitch
    double distance_pre;    //��һʱ��Ŀ��װ�װ�ľ���
    bool if_change;  //�Ƿ�Ϊ��һ���˲���Ŀ���Ƿ�ı�

    /*
    * @brief ��ŷ����תΪ��Ԫ��
    * @input ���ܵ�ǰpitch,yaw�ĽǶ���
    * */
    void transform(const double &pitch, const double &yaw);

    /*
     * @brief Ϊ�۲�ֵ��ֵ
     * @input ���ܵ�ǰpitch,yaw�ĽǶ���
     * */
    void get_measure(const double &pitch, const double &yaw, const double &distance);

    /*
    * @brief ����״̬ת�ƾ���A
    */
    void get_A(const double &pitch, const double &yaw);

    /*
    * @brief ��ȡ����ֵ
    */
    void get_x_();

    /*
    * @brief ��ȡЭ�����������
    */
    void get_p_();

    /*
    * @brief Լ���˲���
    */
    void restraint_update();

public:
    EKF_autoaim();

    ~EKF_autoaim() {};

    /*
     * @brief �˲�������
     * @input ���ܵ�ǰpitch,yaw�ĽǶ��ƣ�distanceΪĿ��װ�װ�ľ������ж��Ƿ���Ŀ��
     * */
    void update(const double &pitch, const double &yaw, const double &distance);

    /*
    * @brief ����Ԥ��ֵ
    * @input pitch,yaw:���ⲿ���ܵ�ǰ�Ƕ�ֵ��ʵ�����ó䵱����ֵ
    */
    void return_pitch_yaw(double &pitch, double &yaw);
};

#endif //KALMAN_AUTOAIM_H_