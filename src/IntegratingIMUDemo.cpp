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



int main()
{
   std::string dir_name = "/home/steve/tmp/test/51/" ;

   CSVReader imuReader(dir_name+"sim_imu.csv");
   CSVReader zuptReader(dir_name+"sim_zupt.csv");

   // Load data

   Eigen::MatrixXd imu_data(imuReader.GetMatrix().GetRows(),imuReader.GetMatrix().GetCols());
   Eigen::MatrixXd zupt_data(zuptReader.GetMatrix().GetRows(),zuptReader.GetMatrix().GetCols());

   for(int i(0);i<imuReader.GetMatrix().GetRows();++i)
   {
      for(int j(0);j<imuReader.GetMatrix().GetCols();++j)
      {
         imu_data(i,j) = *(imuReader.GetMatrix()(i,j));
      }
      zupt_data(i,0) = *(zuptReader.GetMatrix()(i,0));
   }


   // Prepare State value

   



}