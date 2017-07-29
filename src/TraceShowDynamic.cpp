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

#include <pangolin/gl/glglut.h>


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

int whole_index(0);
bool changed_by_key(false);

void addIndex() {
    changed_by_key = true;
    whole_index++;
    return;
}

void reduceIndex() {
    changed_by_key = true;
    whole_index--;
    return;
}


int main(int argc, char *argv[]) {

    std::string axis_file("/home/steve/Code/QuickFusing/ResultData/axis.txt");
    std::string uwb_file("/home/steve/Code/QuickFusing/ResultData/uwb_tmp.txt");
    std::string range_file("/home/steve/Code/QuickFusing/ResultData/range_file.txt");

    if (argc == 0) {
        std::cout << "use default parameter" << std::endl;
    }

    // Load configuration data
    pangolin::ParseVarsFile("app.cfg");

    // Create OpenGL window in single line
    pangolin::CreateWindowAndBind("Main", 1024, 768);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 1024, 768, 640, 480, 0.1, 1000),
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


    CppExtent::CSVReader outaxis(axis_file), outuwb(uwb_file), outrange(range_file);

    CppExtent::Matrix<double> axis_mat = outaxis.GetMatrix();
    CppExtent::Matrix<double> uwb_mat = outuwb.GetMatrix();
    CppExtent::Matrix<double> range_mat = outrange.GetMatrix();



    // Draw axis
    pangolin::Var<int> a_step_num("ui.Log_scale var", 1, 1, axis_mat.GetRows(), false);
    pangolin::Var<bool> can_play("ui.play_or_not", true, true);
    pangolin::Var<bool> show_range_line("ui.show_range_line", true, true);
    pangolin::Var<bool> show_range_sphere("ui.show_range_sphere", true, true);
    pangolin::Var<bool> show_beacon_sphere("ui.show_beacon_sphere", true, true);

    pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 'w', addIndex);
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 's', reduceIndex);


    double last_time = TimeStamp::now();

    while (!pangolin::ShouldQuit()) {
//        std::cout << a_double_log.Get() << std::endl;
//        a_double_log.
//        a_double_log= 100.0;
        if (changed_by_key) {
            a_step_num = whole_index;
            changed_by_key = false;
        }


        if (!can_play.Get()) {
            whole_index = a_step_num;
        }
        if (whole_index > axis_mat.GetRows() - 1) {
            whole_index = axis_mat.GetRows() - 1;
        }

        if (TimeStamp::now() - last_time > 1.0 && can_play.Get()) {
            if (whole_index < axis_mat.GetRows() - 1) {
                whole_index++;
                a_step_num = whole_index;

//            sleep(1);

            }
            last_time = TimeStamp::now();
        }

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);


        double big_axis[] = {0.0, 0.0, 0.0,
                             100.0, 0.0, 0.0,
                             0.0, 100.0, 0.0,
                             0.0, 0.0, 100.0};

        drawAxis(big_axis);


        for (int i = 0; i < whole_index; ++i) {
            drawAxis(axis_mat(i, 0));
        }

        // trace
        glColor3f(1.0, 1.0, 1.0);
        glBegin(GL_LINES);

        for (int i = 0; i < whole_index; ++i) {
            glVertex3f(*axis_mat(i, 0), *axis_mat(i, 1), *axis_mat(i, 2));
            glVertex3f(*axis_mat(i + 1, 0), *axis_mat(i + 1, 1), *axis_mat(i + 1, 2));
        }
        glEnd();
        glPopMatrix();


        // Range
        for (int i = whole_index - 1; i < whole_index; ++i) {
            if (i < 0) {
                continue;
            }
            for (int j(0); j < range_mat.GetCols(); ++j) {
                if (*range_mat(i, j) > 0.0) {
                    Eigen::Vector3d tmp_vec(*axis_mat(i, 0) - *uwb_mat(j, 0),
                                            *axis_mat(i, 1) - *uwb_mat(j, 1),
                                            *axis_mat(i, 2) - *uwb_mat(j, 2));

                    tmp_vec = tmp_vec / tmp_vec.norm() * (*range_mat(i, j));
//                    std::cout<< "tmp vec range :" << tmp_vec.norm() << ",";

                    glLineWidth(3.0);



                    // show range by line
                    if (show_range_line) {
                        glBegin(GL_LINES);
                        glColor3f(1.0, 1.0, 0.0);
                        glVertex3f(*uwb_mat(j, 0), *uwb_mat(j, 1), *uwb_mat(j, 2));
                        glVertex3f(*uwb_mat(j, 0) + tmp_vec(0),
                                   *uwb_mat(j, 1) + tmp_vec(1),
                                   *uwb_mat(j, 2) + tmp_vec(2));
                        glEnd();
                        glPopMatrix();

                    }


                    if (show_range_sphere) {
                        glPushMatrix();
                        glColor4f(0.8, 0.2, 0.8, 0.1);
                        glTranslated(*uwb_mat(j, 0), *uwb_mat(j, 1), *uwb_mat(j, 2));
                        GLUquadricObj *quadricObj = gluNewQuadric();
                        gluQuadricDrawStyle(quadricObj, GLU_POINT);
                        gluSphere(quadricObj, *range_mat(i, j), 260000, 260000);
                        glPopMatrix();
                    }

                    if(show_beacon_sphere)
                    {
                        glPushMatrix();
                        glColor4f(0.8, 0.8, 0.8, 0.1);
                        glTranslated(*uwb_mat(j, 0), *uwb_mat(j, 1), *uwb_mat(j, 2));
                        GLUquadricObj *quadricObj2 = gluNewQuadric();
                        gluQuadricDrawStyle(quadricObj2, GLU_POINT);
                        gluSphere(quadricObj2, 1.0, 260000, 260000);
                        glPopMatrix();
                    }




                }
            }

        }


        pangolin::FinishFrame();
    }

    return 0;


}