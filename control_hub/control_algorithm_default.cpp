/**
 * control_algorithm.cpp
 *
 *  Created on: 2021. 1. 29.
 *      Author: RNB_CAD
 */

#include "control_algorithm_default.h"
#include <string.h>
#include <iostream>

using namespace RNB;

void ControlAlgorithmDefault::reset_control(Eigen::VectorXd &q0){
}

int ControlAlgorithmDefault::calculate_joint_control_torque(
		Eigen::VectorXd& q, Eigen::VectorXd& qdot, Eigen::VectorXd& qddot,
		Eigen::VectorXd& q_d, Eigen::VectorXd& qdot_d, Eigen::VectorXd& qddot_d,
		Eigen::VectorXd& q_n, Eigen::VectorXd& qdot_n,
		Eigen::VectorXd& p, Eigen::VectorXd& p_n,
		Eigen::MatrixXd& M, Eigen::MatrixXd& C,
		Eigen::MatrixXd& M_n, Eigen::MatrixXd& C_n,
		Eigen::VectorXd& tauGrav, Eigen::VectorXd& tauExt,
		Eigen::MatrixXd& J, Eigen::MatrixXd& Jdot,
		Eigen::MatrixXd& J_n, Eigen::MatrixXd& Jdot_n,
		Eigen::VectorXd& FText,
		Eigen::VectorXd& qddot_n, Eigen::VectorXd& torque_out,
		Eigen::VectorXd& custom_dat)
{
	qddot_n = qddot;
	torque_out = (tauGrav * Kg);
	return 0;
}

int ControlAlgorithmDefault::calculate_task_control_torque(
		Eigen::VectorXd& p, Eigen::VectorXd& p_n,
		Eigen::VectorXd& p_d, Eigen::VectorXd& pdot_d, Eigen::VectorXd& pddot_d,
		Eigen::VectorXd& q, Eigen::VectorXd& qdot, Eigen::VectorXd& qddot,
		Eigen::VectorXd& q_n, Eigen::VectorXd& qdot_n,
		Eigen::MatrixXd& M, Eigen::MatrixXd& C,
		Eigen::MatrixXd& M_n, Eigen::MatrixXd& C_n,
		Eigen::VectorXd& tauGrav, Eigen::VectorXd& tauExt,
		Eigen::MatrixXd& J, Eigen::MatrixXd& Jdot,
		Eigen::MatrixXd& J_n, Eigen::MatrixXd& Jdot_n,
		Eigen::VectorXd& FText,
		Eigen::VectorXd& qddot_n, Eigen::VectorXd& torque_out,
        Eigen::VectorXd& custom_dat)
{
	qddot_n.setZero();
	torque_out = (tauGrav * Kg);
	return 0;
}

// /* Not used: Default controller is not loaded as a dynamic library */
//EXPORT_CONTROLLER(ControlAlgorithmDefault)
