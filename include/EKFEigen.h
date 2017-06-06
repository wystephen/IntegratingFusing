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


        double roll(atan2(-f_v, -f_w)), pitch(atan2(f_u, sqrt(f_v * f_v + f_w * f_w)));

        Eigen::Vector3d attitude(roll, pitch, para_.init_heading1_);

//        Eigen::Matrix3d Rb2t = Rt2b(attitude);
//        Rb2t.transposeInPlace();

//        quat_ = dcm2q(Rb2t);
//        Eigen::Rotation3d
        quaterniond_ = Eigen::AngleAxis<double>(attitude.norm(),attitude/attitude.norm());

        x_h_.block(0, 0, 3, 1) = para_.init_pos1_;
        x_h_.block(6, 0, 3, 1) = attitude;


        return true;
    }

    /**
 * Initial Filter .(only run in construct function.)
 * @return
 */
    bool InitialFilter() {
//        MYCHECK("1");
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
                                       Eigen::Quaterniond q,
                                       double dt) {

//        MYCHECK(1);


        Eigen::VectorXd y;
        y.resize(9);

        Eigen::Vector3d w_tb(u.block(3, 0, 3, 1));

        double v(w_tb.norm() * dt);

        if (fabs(v) > 1e-8) {
            double P(w_tb(0) * dt * 0.5);
            double Q(w_tb(1) * dt * 0.5);
            double R(w_tb(2) * dt * 0.5);

            Eigen::Matrix4d OMEGA;

            OMEGA.setZero();

            OMEGA(0, 1) = R;
            OMEGA(0, 2) = -Q;
            OMEGA(0, 3) = P;

            OMEGA(1, 0) = -R;
            OMEGA(1, 2) = P;
            OMEGA(1, 3) = Q;

            OMEGA(2, 0) = Q;
            OMEGA(2, 1) = -P;
            OMEGA(2, 3) = R;

            OMEGA(3, 0) = -P;
            OMEGA(3, 1) = -Q;
            OMEGA(3, 2) = -R;


            //TODO: Try to use rotation matrix?
            // first-order Runge-Kutta use to update the q....
            Eigen::Vector4d tmp_q(q.x(),q.y(),q.z(),q.w());
            tmp_q = (cos(v / 2.0) * Eigen::Matrix4d::Identity() +
                     2.0 / v * sin(v / 2.0) * OMEGA) * (tmp_q);
//            quaterniond_ = Eigen::Quaterniond()
            quaterniond_.x() = tmp_q(0);
            quaterniond_.y() = tmp_q(1);
            quaterniond_.z() = tmp_q(2);
            quaterniond_.w() = tmp_q(3);

            quaterniond_.normalize();
//            quat_ /= quat_(3);


        } else {
            /*
             * Need not do any thing.
             */
            quaterniond_ = q;
        }

        //---------------
        Eigen::Vector3d g_t(0, 0, 9.81);
//        g_t = g_t.transpose();

        Eigen::Matrix3d Rb2t =quaterniond_.toRotationMatrix();
        Eigen::MatrixXd f_t(Rb2t * (u.block(0, 0, 3, 1)));

        Eigen::Vector3d acc_t(f_t + g_t);

        Eigen::MatrixXd A, B;

        A.resize(6, 6);
        A.setIdentity();
//        MYCHECK(1);
//        std::cout << A.rows() << " x " << A.cols() << std::endl;
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
        y.block(0, 0, 6, 1) = A * (x_h.block(0, 0, 6, 1)) +
                              B * acc_t;
//        MYCHECK(1);


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
    bool StateMatrix(Eigen::Quaterniond q, Eigen::VectorXd u, double dt) {

//        MYCHECK(1);

        Eigen::Matrix3d Rb2t = q.toRotationMatrix();
        Eigen::Translation3d R;


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
                                         Eigen::VectorXd dx,
                                         Eigen::Quaterniond q_in) {

//        MYCHECK(1);

        Eigen::Matrix3d R = q_in.toRotationMatrix();

        Eigen::VectorXd x_out = x_in + dx;

        Eigen::Vector3d epsilon(dx.block(6, 0, 3, 1));

        Eigen::Matrix3d OMEGA;
        OMEGA.setZero();

        OMEGA(0, 1) = -epsilon(2);
        OMEGA(0, 2) = epsilon(1);

        OMEGA(1, 0) = epsilon(2);
        OMEGA(1, 2) = -epsilon(0);

        OMEGA(2, 0) = -epsilon(1);
        OMEGA(2, 1) = epsilon(0);


        R = (Eigen::Matrix3d::Identity() - OMEGA) * (R);


        Eigen::AngleAxisd aa;
        aa = Eigen::Matrix3d(R);
        quaterniond_ = aa;

        return x_out;

    }

    Eigen::VectorXd GetPosition(Eigen::VectorXd u, double zupt1) {


        x_h_ = NavigationEquation(x_h_, u, quaterniond_, para_.Ts_);
        StateMatrix(quaterniond_, u, para_.Ts_);

        P_ = (F_ * (P_)) * (F_.transpose().eval()) +
             (G_ * Q_ * G_.transpose().eval());
        if (zupt1 > 0.5) {
            Eigen::Vector3d z(-x_h_.block(3, 0, 3, 1));


            Eigen::MatrixXd K;
            K = P_ * H_.transpose().eval() * (H_ * P_ * H_.transpose().eval() + R_).inverse();
//            K = P_ * H_.transpose() * R_.inverse();

            Eigen::VectorXd dx = K * z;
            dx_ = dx;

            Eigen::MatrixXd Id;
            Id.resize(9, 9);
            Id.setIdentity();

            P_ = (Id - K * H_) * P_;
//            P_ = (Id-K_*H_)*P_*(Id-K_*H_).transpose() + K_ * R_ * K_.transpose();

            x_h_ = ComputeInternalState(x_h_, dx, quaterniond_);
        }


        P_ = (P_ + P_.transpose()) * 0.5;



        return x_h_;
    }


public:
    //Parameters in here.
    SettingPara para_;
private:



    //P for single foot
    Eigen::Matrix<double, 9, 9> P_;

    Eigen::Matrix<double, 6, 6> Q_;

    Eigen::Matrix3d R_;

    Eigen::Matrix<double, 3, 9> H_;

    Eigen::Matrix<double, 9, 1> x_h_;

    Eigen::MatrixXd F_;
    Eigen::MatrixXd G_;

    Eigen::MatrixXd K_;


    Eigen::Vector4d quat_;

    Eigen::Quaterniond quaterniond_;

    Eigen::MatrixXd dx_;


    std::deque<Eigen::Vector2d> heading_vec_deque_;
    std::deque<double> velocity_deque_;
    int count_move_times_ = 0;


    Eigen::VectorXd last_chage_state_;

    bool last_zupt_ = true;
};

#endif //INTEGRATINGFUSING_EKFEIGEN_H
