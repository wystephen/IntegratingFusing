//
// Created by steve on 17-6-2.
//

#include <iostream>
#include <cmath>


#include "CSVReader.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include "sophus/so3.h"
#include "sophus/se3.h"


#include "MYEKF.h"
#include "EKF.hpp"
#include "SettingPara.h"


int main() {
    std::string dir_name = "/home/steve/tmp/test/51/";

    CppExtent::CSVReader imuReader(dir_name + "imu.txt");
    CppExtent::CSVReader zuptReader(dir_name + "sim_zupt.csv");

    auto imuM = imuReader.GetMatrix();
    auto zuptM = zuptReader.GetMatrix();

    // Load data

    Eigen::MatrixXd imu_data(imuM.GetRows(), imuM.GetCols());
    Eigen::MatrixXd zupt_data(zuptM.GetRows(), zuptM.GetCols());


    for (int i(0); i < imuM.GetRows(); ++i) {
        for (int j(0); j < imuM.GetCols(); ++j) {
            imu_data(i, j) = *(imuM(i, j));
            if (0 < j < 4) {
                imu_data(i, j) *= 9.81;
            } else if (4 <= j < 7) {
                imu_data(i, j) *= (M_PI / 180.0);
            }
        }
        zupt_data(i, 0) = *(zuptM(i, 0));
    }




    /**
     * MyEKF
     */

    std::ofstream out_file("./ResultData/out_result.txt");


    SettingPara init_para(true);

    init_para.init_pos1_ = Eigen::Vector3d(0.0, 0.0, 0.0);
//    init_para.init_heading1_ = 0.0 + 20 / 180.0 * M_PI;
    init_para.init_heading1_ = M_PI / 2.0;

//    init_para.sigma_a_ *= 6.0;
//
//    init_para.sigma_g_ *= 6.0;
//
//    init_para.sigma_acc_ *=8.0;
//    init_para.sigma_gyro_ *=8.0;

    init_para.Ts_ = 1.0/ 200.0;

    MyEkf myekf(init_para);

    myekf.InitNavEq(imu_data.block(0, 1, 20, 6));

    for (int i(0); i < imu_data.rows(); ++i) {
//        if(i>1)
//        {
//            init_para.Ts_ = imu_data(i,0)-imu_data(i-1,0);
//        }

//        std::cout << init_para.Ts_ << std::endl;
        Eigen::VectorXd vec = myekf.GetPosition(imu_data.block(i, 1, 1, 6).transpose(),
                                                zupt_data(i, 0));
        std::cout << imu_data.block(i,1,1,6) << std::endl;

        if (i > 1 && zupt_data(i, 0) > 0.5 && zupt_data(i - 1, 0) < 0.5) {
            out_file << vec(0) << " " << vec(1) << " " << vec(2) << std::endl;
        }
    }


}