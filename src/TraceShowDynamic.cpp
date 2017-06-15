//
// Created by steve on 17-6-15.
//


#include <pangolin/pangolin.h>
#include <Eigen/Dense>

#include <sophus/so3.h>

#include <string>
#include <fstream>
#include <iostream>


#include <CSVReader.h>

#include <time_stamp.h>

inline bool drawAxis(double *p)
{
    glLineWidth(2);

//   glColor3f(1.0,0.0,0.0);
    for(int i(0);i<3;++i)
    {
        if(i==0)
        {
            glColor3f(1.0,0.0,0.0);
        }else if(1 == i)
        {
            glColor3f(0.0,1.0,0.0);
        }else if(2 == i)
        {
            glColor3f(0.0,0.0,1.0);
        }

        glBegin(GL_LINES);
        glVertex3f(p[0],p[1],p[2]);
        glVertex3f(p[0]+p[3+i*3],p[1]+p[4+i*3],p[2]+p[5+i*3]);
        glEnd();
        glPopMatrix();


    }


}


int main(int argc, char * argv[])
{

    std::string axis_file("/home/steve/Code/QuickFusing/ResultData/axis.txt");
    std::string uwb_file("/home/steve/Code/QuickFusing/ResultData/uwb_tmp.txt");
    std::string range_file("/home/steve/Code/QuickFusing/ResultData/range_file.txt");

    if(argc ==0 )
    {
        std::cout << "use default parameter" << std::endl;
    }

    // Load configuration data
    pangolin::ParseVarsFile("app.cfg");

    // Create OpenGL window in single line
    pangolin::CreateWindowAndBind("Main",640,480);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(640,480,420,420,320,240,0.1,1000),
            pangolin::ModelViewLookAt(-0,0.5,-90, 0,0,0, pangolin::AxisY)
    );

    const int UI_WIDTH = 180;

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0, -640.0f/480.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    // Add named Panel and bind to variables beginning 'ui'
    // A Panel is just a View with a default layout and input handling
    pangolin::CreatePanel("ui")
            .SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));



    CppExtent::CSVReader outaxis(axis_file),outuwb(uwb_file),outrange(range_file);

    CppExtent::Matrix<double> axis_mat = outaxis.GetMatrix();
    CppExtent::Matrix<double> uwb_mat = outuwb.GetMatrix();
    CppExtent::Matrix<double> range_mat = outrange.GetMatrix();

    int whole_index(0);
    double last_time = TimeStamp::now();

    while(!pangolin::ShouldQuit())
    {
        if(TimeStamp::now()- last_time > 1.0)
        {
            if(whole_index < axis_mat.GetRows()-1)
            {
                whole_index++;
//            sleep(1);

            }
            last_time = TimeStamp::now();
        }

        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);


        double big_axis[]={0.0,0.0,0.0,
                           100.0,0.0,0.0,
                           0.0,100.0,0.0,
                           0.0,0.0,100.0};

        drawAxis(big_axis);


        for(int i=0;i<whole_index;++i)
        {
            drawAxis(axis_mat(i,0));
        }

        // trace
        glColor3f(1.0,1.0,1.0);
        glBegin(GL_LINES);

        for(int i=0;i<whole_index;++i)
        {
            glVertex3f(*axis_mat(i,0),*axis_mat(i,1),*axis_mat(i,2));
            glVertex3f(*axis_mat(i+1,0),*axis_mat(i+1,1),*axis_mat(i+1,2));
        }
        glEnd();
        glPopMatrix();


        // Range
        for(int i=whole_index-3;i<whole_index;++i)
        {
            if(i<0)
            {
                continue;
            }
            for(int j(0);j<range_mat.GetCols();++j)
            {
                if(*range_mat(i,j)>0.0)
                {
                    Eigen::Vector3d tmp_vec(*axis_mat(i,0)-*uwb_mat(j,0),
                    *axis_mat(i,1)-*uwb_mat(j,1),
                    *axis_mat(i,2)-*uwb_mat(j,2));

                    tmp_vec = tmp_vec / tmp_vec.norm() * (*range_mat(i,j));
//                    std::cout<< "tmp vec range :" << tmp_vec.norm() << ",";

                    glLineWidth(1.0);


//                    glBegin(GL_C);
//                    glBegin(GL_C)
                    glColor3f(1.0,1.0,0.0);
                    glVertex3f(*uwb_mat(j,0),*uwb_mat(j,1),*uwb_mat(j,2));
                    glVertex3f(*uwb_mat(j,0)+tmp_vec(0),
                    *uwb_mat(j,1)+tmp_vec(1),
                    *uwb_mat(j,2) +tmp_vec(2));
//                    std::cout <<*uwb_mat(j,0)<<","<<*uwb_mat(j,1)<<","<<*uwb_mat(j,2)<<",";
//                    std::cout << *uwb_mat(j,0)+tmp_vec(0)<<","<<
//                            *uwb_mat(j,1)+tmp_vec(1)<<","<<
//                            *uwb_mat(j,2) +tmp_vec(2);
                    glEnd();
                    glPopMatrix();

//                    std::cout << "draw range" << std::endl;

                }
            }

        }




        pangolin::FinishFrame();
    }

    return 0;




}