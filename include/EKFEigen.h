//
// Created by steve on 17-6-6.
//

#ifndef INTEGRATINGFUSING_EKFEIGEN_H
#define INTEGRATINGFUSING_EKFEIGEN_H


#include "sophus/so3.h"
#include "sophus/se3.h"

#include "SettingPara.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>


class EKFEigen{
public:
    EKFEigen(SettingPara para){
        para_ = para;

        R_ = Eigen::Matrix3d::Zero();

        P_.resize(9, 9);
        P_.setZero();

        Q_.resize(6, 6);
        Q_.setZero();

        H_.resize(3, 9);
        H_.setZero();


        InitialFilter();

        x_h_.resize(9, 1);
        x_h_.setZero();

        K_.resize(9, 3);
        K_.setIdentity();
    }


    /**
  * Initial parameters in navigation equation.
  * @param u
  * @return
  */
    bool InitNavEq(Eigen::MatrixXd u) {

        double f_u(0.0), f_v(0.0), f_w(0.0);

        f_u = u.col(0).mean();
        f_v = u.col(1).mean();
        f_w = u.col(2).mean();

        double t_norm= 0.0;
        for(int i(0);i<u.rows();++i)
        {
            t_norm += u.block(i,0,1,3).norm();
        }

        t_norm /= u.rows();
        own_g_ = t_norm;
        std::cout << __FILE__ << ":" << __LINE__ << ":" << "own g :" << own_g_ << std::endl;


//        pitch = -asin(acc_s(1,1)/g);
//        roll = atan(acc_s(2,1)/acc_s(3,1));
        double roll(atan(f_v/f_w)), pitch(-asin(f_u/own_g_));

//        Eigen::Vector3d attitude(roll, pitch, para_.init_heading1_);
//
//        Eigen::Quaterniond q;
//        Eigen::AngleAxisd ang(attitude.norm(),attitude);
//        q = ang;
        SO3_rotation_ = Sophus::SO3(roll,pitch,para_.init_heading1_);

        Eigen::Vector3d attitude(SO3_rotation_.log()(0),
        SO3_rotation_.log()(1),
        SO3_rotation_.log()(2));


        Eigen::Vector3d tmp_acc(f_u,f_v,f_w);
        Eigen::Matrix3d tm = SO3_rotation_.matrix();
        std::cout << tmp_acc.transpose() << std::endl;
        std::cout << roll/M_PI*180.0 << " --- " << pitch/M_PI*180.0 << std::endl;
        std::cout << tm << std::endl;
        std::cout << "norm before:"<< tmp_acc.norm() << std::endl;
        tmp_acc = tm * tmp_acc;
        std::cout <<__FILE__<<__LINE__ << "acc:"<<std::endl <<
                                                tmp_acc.transpose() << std::endl;
        std::cout << "norm after :" << tmp_acc.norm() << std::endl;


        x_h_.block(0, 0, 3, 1) = para_.init_pos1_;
        x_h_.block(6, 0, 3, 1) = attitude;


        return true;
    }

    /**
 * Initial Filter .(only run in construct function.)
 * @return
 */
    bool InitialFilter() {

        for (int i(0); i < 3; ++i) {
            P_(i, i) = para_.sigma_initial_pos1_(i) * para_.sigma_initial_pos1_(i);
            P_(i + 3, i + 3) = para_.sigma_initial_vel1_(i) * para_.sigma_initial_vel1_(i);
            P_(i + 6, i + 6) = para_.sigma_initial_att1_(i) * para_.sigma_initial_att1_(i);

            R_(i, i) = para_.sigma_vel_(i) * para_.sigma_vel_(i);

            Q_(i, i) = para_.sigma_acc_(i) * para_.sigma_acc_(i);
            Q_(i + 3, i + 3) = para_.sigma_gyro_(i) * para_.sigma_gyro_(i);

            H_(i, i + 3) = 1.0;
        }


        return true;
    }

    Eigen::VectorXd NavigationEquation(Eigen::VectorXd x_h,
                                       Eigen::VectorXd u,
                                       double dt) {

//        MYCHECK(1);


        Eigen::VectorXd y;
        y.resize(9);
        y.setZero();

        Eigen::Vector3d w_tb(u(3),u(4),u(5));

        if (fabs(w_tb.norm()) > 1e-18) {

            SO3_rotation_ = Sophus::SO3::exp((w_tb)*dt)*SO3_rotation_;
//            SO3_rotation_ =SO3_rotation_* Sophus::SO3::exp(-(w_tb)*dt);
//            Eigen::Matrix3d tmp_matrx = SO3_rotation_.matrix();



            y(6) = SO3_rotation_.log()(0);
            y(7) = SO3_rotation_.log()(1);
            y(8) = SO3_rotation_.log()(2);


        } else {
            /*
             * Need not do any thing.
             */
//            quaterniond_ = q;
        }

        //---------------
        Eigen::Vector3d g_t(0, 0, own_g_);//.81);
//        g_t = g_t.transpose();

        Eigen::Matrix3d Rb2t = SO3_rotation_.matrix();
        Eigen::MatrixXd f_t(Rb2t * (u.block(0, 0, 3, 1)));

        Eigen::Vector3d acc_t(f_t + g_t);

        Eigen::MatrixXd A, B;

        A.resize(6, 6);
        A.setIdentity();

        A(0, 3) = dt;
        A(1, 4) = dt;
        A(2, 5) = dt;

        B.resize(6, 3);
        B.setZero();
        Eigen::Matrix3d tmp;

//        std::cout << B.rows() << " x " << B.cols() << std::endl;
//        tmp.setZero();
//        B.block(0, 0, 3, 3) = tmp;
        B.block(3, 0, 3, 3) = Eigen::Matrix3d::Identity() * dt;

//        MYCHECK(1);
        y.block(0, 0, 6, 1) = A * ( x_h.block(0, 0, 6, 1)) +
                              B * acc_t;

        x_h_ = y;
//        MYCHECK(1);
        return y;
    }

    /**
    *
    * @param q
    * @param u
    * @param dt
    * @return
    */
    bool StateMatrix(Eigen::VectorXd u, double dt) {

//        MYCHECK(1);

        Eigen::Matrix3d Rb2t = SO3_rotation_.matrix();


        Eigen::Vector3d f_t(Rb2t * (u.block(0, 0, 3, 1)));

        Eigen::Matrix3d St;
        St.setZero();

        St(0, 1) = -f_t(2);
        St(0, 2) = f_t(1);

        St(1, 0) = f_t(2);
        St(1, 2) = f_t(0);

        St(2, 0) = -f_t(1);
        St(2, 1) = f_t(0);

        Eigen::MatrixXd Fc;
        Fc.resize(9, 9);
        Fc.setZero();

        Fc.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
        Fc.block(3, 6, 3, 3) = St;

        Eigen::MatrixXd Gc;
        Gc.resize(9, 6);
        Gc.setZero();

        Gc.block(3, 0, 3, 3) = Rb2t;
        Gc.block(6, 3, 3, 3) = -Rb2t;


        Eigen::MatrixXd Id;
        Id.resize(9, 9);
        Id.setIdentity();

        F_ = Id + (Fc * dt);
        G_ = Gc * dt;

        return true;

    }


    /**
     * Plus the error of obvious into the state.
     * @param x_in current state in prior
     * @param dx
     * @param q_in current quantanien
     * @return
     */
    Eigen::VectorXd ComputeInternalState(Eigen::VectorXd x_in,
                                         Eigen::VectorXd dx) {



        Eigen::VectorXd x_out = x_in + dx;

        Eigen::Vector3d epsilon(dx.block(6, 0, 3, 1));


//        SO3_rotation_ = Sophus::SO3::exp(epsilon) * SO3_rotation_;

        SO3_rotation_ =   SO3_rotation_ * Sophus::SO3::exp(epsilon);
        x_out(6)= SO3_rotation_.log()(0);
        x_out(7) = SO3_rotation_.log()(1);
        x_out(8) = SO3_rotation_.log()(2);


        return x_out;

    }

    Eigen::VectorXd GetPosition(Eigen::VectorXd u, double zupt1) {


        x_h_ = NavigationEquation(x_h_, u, para_.Ts_);

        StateMatrix( u, para_.Ts_);

        P_ = (F_ * (P_)) * (F_.transpose().eval()) +
             (G_ * Q_ * G_.transpose().eval());
        if (zupt1 > 0.5) {
            Eigen::Vector3d z(-x_h_.block(3, 0, 3, 1));

            Eigen::MatrixXd K;
            K = P_ * H_.transpose().eval() * (H_ * P_ * H_.transpose().eval() + R_).inverse();

            Eigen::VectorXd dx = K * z;
            dx_ = dx;

            Eigen::MatrixXd Id;
            Id.resize(9, 9);
            Id.setIdentity();

            P_ = (Id - K * H_) * P_;

            x_h_ = ComputeInternalState(x_h_, dx);
        }


        P_ = (P_*0.5 + P_.transpose().eval()*0.5);



        return x_h_;
    }


public:
    //Parameters in here.
    SettingPara para_;
private:

    double own_g_ = 9.81;



    //P for single foot
    Eigen::Matrix<double, 9, 9> P_;

    Eigen::Matrix<double, 6, 6> Q_;

    Eigen::Matrix3d R_;

    Eigen::Matrix<double, 3, 9> H_;

    Eigen::Matrix<double, 9, 1> x_h_;

    Eigen::MatrixXd F_;
    Eigen::MatrixXd G_;

    Eigen::MatrixXd K_;



//    Eigen::Vector4d quat_;

//    Eigen::Quaterniond quaterniond_;

    Sophus::SO3 SO3_rotation_;

    Eigen::MatrixXd dx_;


    std::deque<Eigen::Vector2d> heading_vec_deque_;
    std::deque<double> velocity_deque_;
    int count_move_times_ = 0;


    Eigen::VectorXd last_chage_state_;

    bool last_zupt_ = true;
};

#endif //INTEGRATINGFUSING_EKFEIGEN_H
