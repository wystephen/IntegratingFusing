//
// Created by steve on 17-7-8.
//

#include <iostream>
#include <cmath>


#include "CSVReader.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <EKFSimple.h>
#include <EKFR.h>

#include "sophus/so3.h"
#include "sophus/se3.h"


#include "MYEKF.h"
#include "EKF.hpp"
#include "SettingPara.h"
#include "EKFEigen.h"
//#include <pangolin/pangolin.h>

int main() {
    // INPUT FILE
    std::string dir_name = "/home/steve/Code/Mini_IMU/Scripts/IMUWB/93/";

    CppExtent::CSVReader imuReader(dir_name + "imu.txt");
    CppExtent::CSVReader zuptReader(dir_name + "sim_zupt.csv");

    auto imuM = imuReader.GetMatrix();
    auto zuptM = zuptReader.GetMatrix();

    Eigen::MatrixXd imu_data(imuM.GetRows(), imuM.GetCols());
    Eigen::MatrixXd zupt_data(zuptM.GetRows(), zuptM.GetCols());


    for (int i(0); i < imuM.GetRows(); ++i) {
        for (int j(0); j < imuM.GetCols(); ++j) {
            imu_data(i, j) = double(*(imuM(i, j)));
            if (0 < j < 4) {
                imu_data(i, j) *= 9.81;
            } else if (4 <= j < 7) {
                imu_data(i, j) *= (M_PI / 180.0);
            }
        }

        zupt_data(i, 0) = *(zuptM(i, 0));
    }


    long data_size = imu_data.rows();

//    Eigen::Matrix<double,data_size,1> timestamp = imu_data.block(0,0,data_size,1);
//    Eigen::Matrix<double,data_size,3> acc_s = imu_data.block(0,1,data_size,3);
//    Eigen::Matrix<double,data_size,3> gyro_s = imu_data.block(0,3,data_size,3);
    Eigen::MatrixXd timestamp(data_size, 1);
    Eigen::MatrixXd acc_s(data_size, 3);
    Eigen::MatrixXd gyro_s(data_size, 3);
    timestamp = imu_data.block(0, 0, data_size, 1);
    acc_s = imu_data.block(0, 1, data_size, 3);// acc in sensor frame
    gyro_s = imu_data.block(0, 3, data_size, 3);

    double g = 9.8;



    /// Initialize parameters
    // try to use average value
    double pitch = -std::asin(acc_s(0, 0) / g);
    double roll = std::atan(acc_s(1, 0) / acc_s(2, 0));
    double yaw = 0.0;

    double cp = std::cos(pitch);
    double sp = std::sin(pitch);

    double cr = std::cos(roll);
    double sr = std::sin(roll);

    double cy = std::cos(yaw);
    double sy = std::sin(yaw);

    Eigen::Matrix3d C, C_prev;

    C << cp * cy, (cr * sp * cy - cr * sy), cr * sp * cy + sr * sy,
            cp * sy, sr * sp * sy + cr * 7cy, cr * sp * sy - sr * cy,
            -sp, sr * cp, cr * cp;
    C_prev = C;

    Eigen::MatrixXd heading(1,data_size);
    heading(0,0) = yaw;


    Eigen::MatrixXd acc_n(data_size,3);// acc in navigation frame
    acc_n.block(0,0,1,3) = (C*acc_s.block(0,0,1,3).transpose()).transpose();


    Eigen::MatrixXd vel_n(data_size,3);

    Eigen::MatrixXd pose_n(data_size,3);

    Eigen::MatrixXd distance(data_size,1);

    // Error covariance
    Eigen::MatrixXd P(9,9);
    P.setZero();

    // Sigma noise
    double sigma_omega = 1e-2;
    double sigma_a = 1e-2;

    // ZUPT
    Eigen::MatrixXd H(9,9);

    for(int i(6);i<9;++i)
    {
        H(i,i) = 1.0;
    }

    double sigma_v = 1e-2;

    Eigen::MatrixXd R(3,3);
    for(int i(0);i<3;++i)
    {
        R(i,i)  = sigma_v*sigma_v;
    }

    double gyro_threshold = 0.6;

    for(int t(1);t<data_size;++t)
    {
        double dt = timestamp(t)-timestamp(t-1);

        //Skew-symmetric matrix for angular rates

        Eigen::Vector3d gyro_s1 = gyro_s.block(t,0,1,3).transpose();

        Eigen::Matrix3d ang_rate_matrix;

        ang_rate_matrix<< 0.0,-gyro_s(2),gyro_s1(1),
        gyro_s1(2),0.0,-gyro_s1(0),
        -gyro_s1(1),gyro_s1(0),0.0;

        C = C_prev*((2*Eigen::Matrix3d::Identity()+(ang_rate_matrix * dt))*
                (2.0*Eigen::Matrix3d::Identity()-(ang_rate_matrix * dt)).inv());

        acc_n.block()




    }














    // OUTPUT FILE
    std::ofstream out_file("./ResultData/out_result.txt");
}