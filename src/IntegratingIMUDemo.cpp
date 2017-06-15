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

#include "sophus/so3.h"
#include "sophus/se3.h"


#include "MYEKF.h"
#include "EKF.hpp"
#include "SettingPara.h"
#include "EKFEigen.h"
#include <pangolin/pangolin.h>

int main() {

//    // Creat a windows
//    pangolin::CreateWindowAndBind("Integrating",640,480);
//    glEnable(GL_DEPTH_TEST);
//    pangolin::OpenGlRenderState s_cam(
//            pangolin::ProjectionMatrix(640,480,420,420,320,240,0.2,100),
//            pangolin::ModelViewLookAt(0,-10,0.1,0,0,0,pangolin::AxisNegY)
//    );
//
//    pangolin::Handler3D hander(s_cam);
//    pangolin::View &d_cam = pangolin::CreateDisplay().SetBounds(
//            0.0,1.0,0.0,1.0,-640.0/480.0
//    ).SetHandler(&hander);







    std::string dir_name = "/home/steve/Code/Mini_IMU/Scripts/IMUWB/47/";

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
            if (0 < j < 4) {
                imu_data(i, j) *= 9.81;//*9.81/9.64877;
//              if(j==3)
//              {
//                  imu_data(i,j) *= -1.0f;
//              }
            } else if (4 <= j < 7) {
                imu_data(i, j) *= (180.0 / M_PI);
//                if(j==6)
//                {
//                    imu_data(i,j) *= -1.0f;
//                }
            }
        }
        
        zupt_data(i, 0) = *(zuptM(i, 0));
    }




    /**
     * MyEKF
     */

    std::ofstream out_file("./ResultData/out_result.txt");
    std::ofstream out_axis("./ResultData/out_axis.txt");


    SettingPara init_para(true);

    init_para.init_pos1_ = Eigen::Vector3d(0.0, 0.0, 0.0);
//    init_para.init_heading1_ = 0.0 + 20 / 180.0 * M_PI;
    init_para.init_heading1_ =0.0;// -2.0;//M_PI / 2.0;

    init_para.sigma_a_ *= 5.0;

    init_para.sigma_g_ *= 5.0;

    init_para.sigma_acc_ *=1.0;
    init_para.sigma_gyro_ *=1.0;

    init_para.Ts_ = 0.005f;//1.0/ 200.0;

    EKFSimple myekf(init_para);

    myekf.InitNavEq(imu_data.block(0, 1, 20, 6));

    for (int i(0); i < imu_data.rows(); ++i) {
//        if(i>1)
//        {
//            myekf.para_.Ts_ = imu_data(i,0)-imu_data(i-1,0);
//        }

//        std::cout << init_para.Ts_ << std::endl;
        Eigen::VectorXd vec = myekf.GetPosition(imu_data.block(i, 1, 1, 6).transpose(),
                                                zupt_data(i, 0));
//        std::cout << imu_data.block(i,1,1,6) << std::endl;
        auto tt = myekf.OutputAxis();
        out_axis << tt.transpose() << std::endl;

        if (i > 1 && zupt_data(i, 0) > 0.5 && zupt_data(i - 1, 0) < 0.5) {
            out_file << vec(0) << " " << vec(1) << " " << vec(2) << std::endl;
        }
    }

    out_file.close();
    out_axis.close();
//    while(!pangolin::ShouldQuit())
//    {
//        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//        d_cam.Activate(s_cam);
//
//        // Render OpenGL Cube
////        pangolin::glDrawColouredCube();\
//        //坐标轴的创建
//        pangolin::glDrawAxis(3);
//
//        //点的创建
//        glPointSize(10.0f);
//        glBegin(GL_POINTS);
//        glColor3f(1.0,1.0,1.0);
//        glVertex3f(0.0f,0.0f,0.0f);
//        glVertex3f(1,0,0);
//        glVertex3f(0,2,0);
//        glEnd();
//
//        //把下面的点都做一次旋转变换
//        glPushMatrix();
//        //col major
//        std::vector<GLfloat > Twc = {1,0,0,0, 0,1,0,0 , 0,0,1,0 ,0,0,5,1};
//        glMultMatrixf(Twc.data());
//
//        //直线的创建
//        const float w = 2;
//        const float h = w*0.75;
//        const float z = w*0.6;
//        glLineWidth(2);
//        glColor3f(1.0,0,0);
//        glBegin(GL_LINES);
//
//        glVertex3f(0,0,0);
//        glVertex3f(w,h,z);
//        glVertex3f(0,0,0);
//        glVertex3f(w,-h,z);
//        glVertex3f(0,0,0);
//        glVertex3f(-w,-h,z);
//        glVertex3f(0,0,0);
//        glVertex3f(-w,h,z);
//        glVertex3f(w,h,z);
//        glVertex3f(-w,h,z);
//        glVertex3f(-w,h,z);
//        glVertex3f(-w,-h,z);
//        glVertex3f(-w,-h,z);
//        glVertex3f(w,-h,z);
//        glVertex3f(w,-h,z);
//        glVertex3f(w,h,z);
//        glEnd();
//
//        glPopMatrix();
//
//        // Swap frames and Process Events
//        pangolin::FinishFrame();
//    }


}