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

    // Special initial value

    Eigen::Matrix<double, 9, 3> K;
    K.setIdentity();


    // INPUT FILE
    std::string dir_name = "/home/steve/Code/Mini-IMU/Scripts/IMUWB/46/";

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
                imu_data(i, j) *= (M_PI / 180.0f);
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
    gyro_s = imu_data.block(0, 4, data_size, 3);

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
    C.setZero();
    C_prev.setZero();

    C << cp * cy, (cr * sp * cy - cr * sy), cr * sp * cy + sr * sy,
            cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy,
            -sp, sr * cp, cr * cp;
    C_prev = C;

    Eigen::MatrixXd heading(1, data_size);
    heading(0, 0) = yaw;


    Eigen::MatrixXd acc_n(data_size, 3);// acc in navigation frame
    acc_n.block(0, 0, 1, 3) = (C * acc_s.block(0, 0, 1, 3).transpose()).transpose();
//    if(acc_n.block(0,0,1,3).norm()>260)
//    {
//        std::cout << " acc n :" << acc_n.block(0,)
//    }


    Eigen::MatrixXd vel_n(data_size, 3);
    vel_n.setZero();

    Eigen::MatrixXd pose_n(data_size, 3);
    pose_n.setZero();

    Eigen::MatrixXd distance(data_size, 1);
    distance.setZero();


    // Error covariance
    Eigen::Matrix<double, 9, 9> P;
    P.setIdentity();

    // Sigma noise
    double sigma_omega = 1e-2;
    double sigma_a = 1e-2;

    // ZUPT
    Eigen::MatrixXd H(3, 9);
    H.setZero();

    for (int i(6); i < 9; ++i) {
        H(i - 6, i) = 1.0;
    }

    double sigma_v = 1e-2;

    Eigen::MatrixXd R(3, 3);
    for (int i(0); i < 3; ++i) {
        R(i, i) = sigma_v * sigma_v;
    }

    double gyro_threshold = 0.6;

    for (int t(1); t < data_size; ++t) {
        double dt = timestamp(t) - timestamp(t - 1);

        //Skew-symmetric matrix for angular rates

        Eigen::Vector3d gyro_s1 = gyro_s.block(t, 0, 1, 3).transpose();
        Eigen::Matrix3d ang_rate_matrix;
        ang_rate_matrix.setZero();
        if (gyro_s1.norm() > 1e-4) {


            ang_rate_matrix << 0.0f, -gyro_s(2), gyro_s1(1),
                    gyro_s1(2), 0.0f, -gyro_s1(0),
                    -gyro_s1(1), gyro_s1(0), 0.0f;

            C = C * ((2.0 * Eigen::Matrix3d::Identity() + (dt * ang_rate_matrix)) *
                     (2.0 * Eigen::Matrix3d::Identity() - (dt * ang_rate_matrix)).inverse());
        }


        if (std::isnan(C.sum()) || (C * C.transpose()-Eigen::Matrix3d::Identity()).norm() > 0.2) {
            std::cout << "t: " << t
                      << "\n ang rate : " << ang_rate_matrix
                      << "\n C: " << C
                      << "\n C * C^T: " << C * C.transpose()
                      << std::endl;
            C = C_prev;
        }

        acc_n.block(t, 0, 1, 3) = (C * acc_s.block(t, 0, 1, 3).transpose().eval()).transpose();
//        if (acc_n.block(t, 0, 1, 3).norm() > 300) {
//            std::cout << "  acc_n error \n C : " << C << "\n accn : " << acc_n.block(t, 0, 1, 3) << " \n acc s: "
//                      << acc_s.block(t, 0, 1, 3) << std::endl;
//        }

        vel_n.block(t, 0, 1, 3) = vel_n.block(t - 1, 0, 1, 3) +
                                  dt / 2.0 * ((acc_n.block(t, 0, 1, 3) - Eigen::Vector3d(0, 0, g).transpose()) +
                                              (acc_n.block(t - 1, 0, 1, 3) - Eigen::Vector3d(0, 0, g).transpose()));

        pose_n.block(t, 0, 1, 3) = pose_n.block(t - 1, 0, 1, 3) +
                                   dt / 2.0 * (vel_n.block(t, 0, 1, 3) + vel_n.block(t - 1, 0, 1, 3));


        Eigen::Matrix3d S;
        S << 0.0, -acc_n(t, 2), acc_n(t, 1),
                acc_n(t, 2), 0.0, -acc_n(t, 0),
                -acc_n(t, 1), acc_n(t, 0), 0.0;
        if (std::abs(acc_n.block(t, 0, 1, 3).norm() - acc_s.block(t, 0, 1, 3).norm()) > 1.0
            || std::isnan(acc_n.block(t, 0, 1, 3).norm())) {
            std::cout << "t: " << t
                      << "\nC : " << C
                      << "\n C*C^T:" << C * C.transpose()

                      << " \nacc_n: " << acc_n.block(t, 0, 1, 3)
                      << "\n acc_s : " << acc_s.block(t, 0, 1, 3) << std::endl
                      << std::endl;
        }

        Eigen::Matrix<double, 9, 9> F;
        F.setZero();

        F.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
        F.block(3, 3, 3, 3) = Eigen::Matrix3d::Identity();
        F.block(6, 6, 3, 3) = Eigen::Matrix3d::Identity();

        F.block(3, 6, 3, 3) = dt * Eigen::Matrix3d::Identity();
        F.block(6, 0, 3, 3) = -dt * S;
        std::cout << "F:" << F << std::endl;

        Eigen::Matrix<double, 9, 9> Q;
        Q.setZero();

        Q.block(0, 0, 3, 3) = sigma_omega * sigma_omega * dt * Eigen::Matrix3d::Identity();
        Q.block(6, 6, 3, 3) = sigma_a * sigma_a * dt * Eigen::Matrix3d::Identity();

        // propagate the error covariance matrix
        auto P_presave = P;
        P = F * P * F.transpose().eval() + Q;

        if (std::isnan(P.sum())) {
            std::cout << "error at t= " << t
                      << F << "\n Q: " << Q << std::endl;


            P = P_presave;
            std::cout << "P:" << P << std::endl;
        }

        C_prev = C;


        ///// Zero-velocity updates
//        std::cout<< gyro_s.block(t,0,1,3) << std::endl;
        if (gyro_s.block(t, 0, 1, 3).norm() < gyro_threshold) {

            // First detecter for data error
            if (acc_n.block(t, 0, 1, 3).norm() > 10.0) {
                std::cout << "t: " << t << " acc_n: " << acc_n.block(t, 0, 1, 3) << std::endl;
            }


            std::cout << "Begin zero velocity Update : " << t << std::endl;

//            std::cout << (P * H.transpose()) << std::endl;
//            std::cout << "--------" << std::endl;
//            std::cout << (H * P * H.transpose()).rows() << std::endl;

            K = (P * H.transpose()) * (H * P * H.transpose() + R);
            if (std::isnan(K.sum())) {
                std::cout << " t: " << t <<
                          "\n K: " << K << std::endl;
            }

            std::cout << "k:" << K << std::endl;
            Eigen::Matrix<double, 9, 1> delta_x = (K * vel_n.block(t, 0, 1, 3).transpose().eval());
//            std::cout << "deltax :" << delta_x << std::endl;
//            std::cout<< "current vel n :" << vel_n.block(t,0,1,3) << std::endl;


            std::cout << " after computer delta x " << std::endl;
            // update the error covariance matrix
            P = (Eigen::Matrix<double, 9, 9>::Identity() - K * H) * P;
            if (std::isnan(P.sum())) {

                std::cout << "t: " << t
                          << "\n K: " << K <<
                          "H: " << H << std::endl;
                P = P_presave;

            }


            Eigen::Vector3d attitude_error = delta_x.block(0, 0, 3, 1);
            Eigen::Vector3d pos_error = delta_x.block(3, 0, 3, 1);
            Eigen::Vector3d vel_error = delta_x.block(6, 0, 3, 1);

            Eigen::Matrix3d ang_matrix;
            ang_matrix << 0.0, -attitude_error(2), attitude_error(1),
                    attitude_error(2), 0.0, -attitude_error(0),
                    -attitude_error(1), attitude_error(0), 0.0;
            ang_matrix *= -1.0;

            std::cout << " after computer C " << std::endl;

            C = (2.0 * Eigen::Matrix3d::Identity() + ang_matrix) *
                (2.0 * Eigen::Matrix3d::Identity() - ang_matrix).inverse()
                * C;
            if (std::isnan(C.sum())) {
                std::cout << C << "\n" << "t: " << t << "\nline:"
                          << __FILE__ << " " << __LINE__ << std::endl;
                std::cout << "acc_n : " << acc_n.block(t, 0, 1, 3) << " acc i :" << acc_s.block(t, 0, 1, 3)
                          << std::endl;
                std::cout << "vel_n x :" << vel_n.block(t, 0, 1, 3) << std::endl;
                C = C_prev;
            }

            vel_n.block(t, 0, 1, 3) = vel_n.block(t, 0, 1, 3) + vel_error.transpose();
            pose_n.block(t, 0, 1, 3) = pose_n.block(t, 0, 1, 3) + pos_error.transpose();


        }

        C_prev = C;

    }


    for (int i(0); i < pose_n.rows() - 1; ++i) {
        if (gyro_s.block(i, 0, 1, 3).norm() < gyro_threshold)// &&
//                gyro_s.block(i+1,0,1,3).norm() > gyro_threshold)
        {
            std::cout << pose_n.block(i, 0, 1, 3) << std::endl;
        }
    }





    // OUTPUT FILE
    std::ofstream out_file("./ResultData/out_result.txt");
    std::ofstream out_vel("./ResultData/out_vel.txt");
    std::ofstream out_acc("./ResultData/out_acc.txt");

    for (int i(0); i < pose_n.rows(); ++i) {
        for (int j(0); j < pose_n.cols(); ++j) {
            out_file << pose_n(i, j);
            if (j < pose_n.cols() - 1) {
                out_file << ";";
            } else {

                out_file << std::endl;
            }
        }
    }

    for (int i(0); i < vel_n.rows(); ++i) {
        for (int j(0); j < vel_n.cols(); ++j) {
            out_vel << vel_n(i, j);
            if (j < vel_n.cols() - 1) {
                out_vel << ";";
            } else {
                out_vel << std::endl;
            }
        }
    }

    for (int i(0); i < acc_n.rows(); ++i) {
        for (int j(0); j < acc_n.cols(); ++j) {
            out_acc << acc_n(i, j);
            if (j < acc_n.cols() - 1) {
                out_acc << ";";
            } else {
                out_acc << std::endl;
            }
        }
    }


}