/*
 * test_controller.cpp
 *  Created on: 2021. 11. 02.
 *      Author: Junsu Kang
 */

#include "../PD/PD.h"
#include "traj_gen.h"
#include <sys/stat.h>

using namespace RNB;

#define JOINT_DOF 7
#define TASK_DOF 6
#define DT 1e-3

int main(int argc, char* argv[]) {
    ControlAlgorithmPD contoller(ControllerInterface::ControlMode::JOINT_CONTROL, JOINT_DOF, TASK_DOF, DT);
    Eigen::VectorXd q0(JOINT_DOF);
    Eigen::VectorXd tauGrav(JOINT_DOF);
    Eigen::VectorXd tauExt(JOINT_DOF);
    Eigen::VectorXd FText(TASK_DOF);
    Eigen::VectorXd p(TASK_DOF);
    Eigen::VectorXd p_n(TASK_DOF);
    Eigen::MatrixXd M(JOINT_DOF, JOINT_DOF);
    Eigen::MatrixXd C(JOINT_DOF, JOINT_DOF);
    Eigen::MatrixXd J(TASK_DOF, JOINT_DOF);
    Eigen::MatrixXd Jdot(TASK_DOF, JOINT_DOF);
    Eigen::VectorXd custom_dat(JOINT_DOF);

    int test_len = 10000;
    M.setIdentity();
    C.setZero();
    J.setIdentity();
    Jdot.setZero();
    q0.fill(0);
    tauGrav.fill(0);
    tauExt.fill(0);
    p.fill(0);
    p_n.fill(0);
    FText.fill(0);
    custom_dat.setZero();
    Eigen::MatrixXd traj = get_sin_traj(JOINT_DOF, test_len, 1000, q0, PI/18);
    Eigen::MatrixXd traj_vel = differentitate(traj, DT);
    Eigen::MatrixXd traj_acc = differentitate(traj_vel, DT);
    Eigen::MatrixXd traj_nominal(test_len, JOINT_DOF);
    Eigen::MatrixXd traj_real(test_len, JOINT_DOF);
    Eigen::MatrixXd traj_torque(test_len, JOINT_DOF);
    Eigen::MatrixXd traj_qddot_n(test_len, JOINT_DOF);

    LowPassFilter<Eigen::VectorXd> real(DT, 50);
    LowPassFilter<Eigen::VectorXd> nominal(DT, 200);

    contoller.reset_control(q0);
    real.reset(q0);
    nominal.reset(q0);
    Eigen::VectorXd q_n_pre = q0;
    Eigen::VectorXd q_pre = q0;
    Eigen::VectorXd qdot_n_pre(JOINT_DOF);
    Eigen::VectorXd qdot_pre(JOINT_DOF);
    qdot_n_pre.fill(0);
    qdot_pre.fill(0);
    for(int i=0; i<test_len; i++){
        Eigen::VectorXd torque_out(JOINT_DOF);
        Eigen::VectorXd q_d = traj.block(i, 0, 1, JOINT_DOF).transpose();
        Eigen::VectorXd qdot_d = traj_vel.block(i, 0, 1, JOINT_DOF).transpose();
        Eigen::VectorXd qddot_d = traj_acc.block(i, 0, 1, JOINT_DOF).transpose();
        Eigen::VectorXd q_n = nominal.update(q_d);
        Eigen::VectorXd qdot_n = differentitate(q_n_pre, q_n, DT);
        Eigen::VectorXd qddot_n = differentitate(qdot_n_pre, qdot_n, DT);
        Eigen::VectorXd q = real.update(q_d);
        Eigen::VectorXd qdot = differentitate(q_pre, q, DT);
        Eigen::VectorXd qddot = differentitate(qdot_pre, qdot, DT);

        contoller.calculate_joint_control_torque(
                q, qdot, qddot, q_d, qdot_d, qddot_d, q_n, qdot_n, p, p_n,
                M, C, M, C, tauGrav, tauExt,
                J, Jdot, J, Jdot, FText, qddot_n, torque_out, custom_dat);
        traj_real.block(i, 0, 1, JOINT_DOF)<<q.transpose();
        traj_nominal.block(i, 0, 1, JOINT_DOF)<<q_n.transpose();
        traj_qddot_n.block(i, 0, 1, JOINT_DOF)<<qddot_n.transpose();
        traj_torque.block(i, 0, 1, JOINT_DOF)<<torque_out.transpose();
    }
    mkdir("test_dat", 0755);
    save_mat("test_dat/q_d.csv", traj);
    save_mat("test_dat/qdot_d.csv", traj_vel);
    save_mat("test_dat/qddot_d.csv", traj_acc);
    save_mat("test_dat/q.csv", traj_real);
    save_mat("test_dat/q_n.csv", traj_nominal);
    save_mat("test_dat/qddot_n.csv", traj_qddot_n);
    save_mat("test_dat/torque_out.csv", traj_torque);

    return 0;
}