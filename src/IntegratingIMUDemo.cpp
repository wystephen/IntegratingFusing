//
// Created by steve on 17-6-2.
//

#include <iostream>
#include <cmath>


#include "CSVReader.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "sophus/so3.h"
#include "sophus/se3.h"


#include "MYEKF.h"
#include "SettingPara.h"


int main() {
    std::string dir_name = "/home/steve/tmp/test/51/";

    CSVReader imuReader(dir_name + "sim_imu.csv");
    CSVReader zuptReader(dir_name + "sim_zupt.csv");

    // Load data

    Eigen::MatrixXd imu_data(imuReader.GetMatrix().GetRows(), imuReader.GetMatrix().GetCols());
    Eigen::MatrixXd zupt_data(zuptReader.GetMatrix().GetRows(), zuptReader.GetMatrix().GetCols());

    for (int i(0); i < imuReader.GetMatrix().GetRows(); ++i) {
        for (int j(0); j < imuReader.GetMatrix().GetCols(); ++j) {
            imu_data(i, j) = *(imuReader.GetMatrix()(i, j));
        }
        zupt_data(i, 0) = *(zuptReader.GetMatrix()(i, 0));
    }


    /**
     * MyEKF
     */


    SettingPara init_para(true);

    init_para.init_pos1_ = Eigen::Vector3d(0.0, 0.0, 0.0);
//    init_para.init_heading1_ = 0.0 + 20 / 180.0 * M_PI;
    init_para.init_heading1_ = M_PI / 2.0;

    init_para.Ts_ = 1.0 / 128.0;

    MyEkf myekf(init_para);

    myekf.InitNavEq(imu_data.block(0, 1, 20, 6));




}