//
// Created by steve on 17-6-2.
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

#include <matplotlib_interface.h>

namespace plt=matplotlibcpp;

int main() {

/**
 * ...new data
 */
    std::string dir_name = "/home/steve/Code/Mini_IMU/Scripts/IMUWB/91/";
//    std::string dir_name = "/home/steve/Code/Mini_IMU/Scripts/IMUWB/93/";

    CppExtent::CSVReader imuReader(dir_name + "imu.txt");
    CppExtent::CSVReader zuptReader(dir_name + "sim_zupt.csv");

    auto imuM = imuReader.GetMatrix();
    auto zuptM = zuptReader.GetMatrix();

    // Load data

    Eigen::MatrixXd imu_data(imuM.GetRows(), imuM.GetCols());
    Eigen::MatrixXd zupt_data(zuptM.GetRows(), zuptM.GetCols());


    for (int i(0); i < imuM.GetRows(); ++i) {
        for (int j(0); j < imuM.GetCols(); ++j) {
            imu_data(i, j) = double(*(imuM(i, j)));
            if (0 < j&& j < 4) {
                imu_data(i, j) *= 9.81;
            } else if (4 <= j && j< 7) {
                imu_data(i,j) *= (M_PI/180.0f);
            }
        }

        zupt_data(i, 0) = *(zuptM(i, 0));
    }
    /**
     * End new data
     */
//
////    std::string dir_name = "/home/steve/locate/3";
//    std::string dir_name = "/home/steve/XsensData/1";
////    std::string dir_name = "/home/steve/XsensData/";
//
////    CppExtent::CSVReader imuReader(dir_name + "imu.txt");
////    CppExtent::CSVReader imuReader(dir_name + "ImuData.data.csv");
////    CppExtent::CSVReader imuReader(dir_name + "sim_imu.csv");
//    CppExtent::CSVReader imuReader(dir_name + "Imu.csv");
////    CppExtent::CSVReader zuptReader(dir_name + "sim_zupt.csv");
////    CppExtent::CSVReader zuptReader(dir_name + "Zupt.data.csv");
////    CppExtent::CSVReader zuptReader(dir_name + "sim_zupt.csv");
//    CppExtent::CSVReader zuptReader(dir_name + "Zupt.csv");
//
//    auto imuM = imuReader.GetMatrix();
//    auto zuptM = zuptReader.GetMatrix();
//
//    Eigen::MatrixXd imu_data(imuM.GetRows(), imuM.GetCols());
//    Eigen::MatrixXd zupt_data(zuptM.GetRows(), zuptM.GetCols());

//
//    for (int i(0); i < imuM.GetRows(); ++i) {
//        for (int j(0); j < imuM.GetCols(); ++j) {
//            imu_data(i, j) = double(*(imuM(i, j)));
////            if (0 < j < 4) {
////                imu_data(i, j) *= 9.81;
////            } else if (4 <= j < 7) {
////                imu_data(i, j) *= (M_PI / 180.0f);
////            }
//        }
//
//        zupt_data(i, 0) = *(zuptM(i, 0));
//    }


    /**
     * MyEKF
     */

    std::ofstream out_file("./ResultData/out_result.txt");
    std::ofstream out_axis("./ResultData/out_axis.txt");


    SettingPara init_para(true);

    init_para.init_pos1_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    init_para.init_heading1_ = 0.0;// -2.0;//M_PI / 2.0;


    init_para.sigma_acc_ = 1.5 * Eigen::Vector3d(1,1,1);
    init_para.sigma_gyro_= 2.5 * Eigen::Vector3d(1,1,1) * M_PI / 180.0;

    init_para.sigma_vel_ = 0.005 * Eigen::Vector3d(1.0,1.0,1.0);

    init_para.sigma_initial_pos1_ *= 1e-3;
//    init_para.sigma_initial_vel1_ *=
    init_para.sigma_initial_att1_ = Eigen::Vector3d(0.1,0.1,0.1) * M_PI / 180.0;

//    init_para.Ts_ = 0.005f;//1.0/ 200.0;
//    init_para.Ts_ = 1.0f/100.0f;//1.0/ 200.0;
    init_para.Ts_ = 1.0f / 200.0f;
    
    init_para.gravity_ = 9.6;

    MyEkf myekf(init_para);

    std::vector<double> gx,gy,gz,zupt_v;

    myekf.InitNavEq(imu_data.block(1, 1, 40, 6));
    for (int i(0); i < imu_data.rows(); ++i) {
//        if(i>1)
//        {
//            myekf.para_.Ts_ = imu_data(i,0)-imu_data(i-1,0);
//        }
        std::cout << "i:" << i << std::endl;
        zupt_v.push_back(zupt_data(i,0));

        Eigen::VectorXd vec = myekf.GetPosition(imu_data.block(i, 1, 1, 6).transpose(),
                                                zupt_data(i, 0));
//        std::cout << imu_data.block(i,1,1,6) << std::endl;

        /// Output axis for each step(the first moment of detected zero-velocity state)
//        auto tt = myekf.OutputAxis();
//        for (int index(0); index < tt.rows(); ++index) {
//            out_axis << tt(index);
//            if (index < tt.rows() - 1) {
//                out_axis << ",";
//            } else {
//                out_axis << std::endl;
//            }
//        }

        if ((zupt_data.rows()-2)>i&& i > 1 && zupt_data(i, 0) > 0.5 && zupt_data(i + 1, 0) < 0.5) {
            out_file << vec(0) << " " << vec(1) << " " << vec(2) << std::endl;
//            std::cout << vec(0) << " " << vec(1) << " " << vec(2) << " "
//                      << vec(3) << " " << vec(4) << " " << vec(5) << std::endl;
            gx.push_back(vec(0));
            gy.push_back(vec(1));
            gz.push_back(vec(2));
        }
    }



    out_file.close();
    out_axis.close();

    plt::plot(gx,gy,"r-+");
//    plt::plot(zupt_v,"b-*");
    plt::show();


}