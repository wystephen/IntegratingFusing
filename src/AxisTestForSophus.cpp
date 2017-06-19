//
// Created by steve on 17-6-19.
//

#include <pangolin/pangolin.h>
#include <Eigen/Dense>

#include <sophus/so3.h>

#include <string>
#include <fstream>
#include <iostream>


#include <CSVReader.h>

#include <time_stamp.h>

#include <pangolin/gl/glglut.h>


/**
 * draw axis according to vector for each axis and original point
 * @param p
 * @return
 */
inline bool drawAxis(double *p) {
    glLineWidth(2);

//   glColor3f(1.0,0.0,0.0);
    for (int i(0); i < 3; ++i) {
        if (i == 0) {
            glColor3f(1.0, 0.0, 0.0);
        } else if (1 == i) {
            glColor3f(0.0, 1.0, 0.0);
        } else if (2 == i) {
            glColor3f(0.0, 0.0, 1.0);
        }

        glBegin(GL_LINES);
        glVertex3f(p[0], p[1], p[2]);
        glVertex3f(p[0] + p[3 + i * 3], p[1] + p[4 + i * 3], p[2] + p[5 + i * 3]);
        glEnd();
        glPopMatrix();


    }


}



int main(int argc, char argv[])
{

    // Create OpenGL window in single line
    pangolin::CreateWindowAndBind("Main", 1024, 768);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 840, 840, 640, 480, 0.1, 1000),
            pangolin::ModelViewLookAt(-0, 0.5, -90, 0, 0, 0, pangolin::AxisY)
    );

    const int UI_WIDTH = 180;

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0, -640.0f / 480.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    // Add named Panel and bind to variables beginning 'ui'
    // A Panel is just a View with a default layout and input handling
    pangolin::CreatePanel("ui")
            .SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));

    // var
    pangolin::Var<double> roll("ui.rollx",0.0,-180.0,180.0,false);
    pangolin::Var<double> pitch("ui.pitchy",0.0,-180.0,180.0,false);
    pangolin::Var<double> yaw("ui.yawz",0.0,-180.0,180.0,false);


    while(!pangolin::ShouldQuit())
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);

        double big_axis[] = {0.0, 0.0, 0.0,
                             2.0, 0.0, 0.0,
                             0.0, 2.0, 0.0,
                             0.0, 0.0, 2.0};

        double min_axis[] = {0.0, 0.0, 0.0,
                             10.0, 0.0, 0.0,
                             0.0, 10.0, 0.0,
                             0.0, 0.0, 10.0};
        drawAxis(big_axis);

        Eigen::Matrix3d src_axis=Eigen::Matrix3d::Identity();

        Sophus::SO3 so3 = Sophus::SO3(roll/180.0*M_PI,pitch/180.0*M_PI,yaw/180.0*M_PI);

        src_axis = so3.matrix() * src_axis;

        for(int i(0);i<3;++i)
        {
            for(int j(0);j<3;++j)
            {
                min_axis[i*3+3+j] = src_axis(j,i);
            }
        }

        drawAxis(min_axis);

        pangolin::FinishFrame();

    }

}