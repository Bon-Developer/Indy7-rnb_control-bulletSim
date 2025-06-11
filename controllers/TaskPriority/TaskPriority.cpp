#include <string.h>
#include <Eigen/Dense>
#include <iostream>
#include "TaskPriority.h"
using namespace RNB;
using namespace Eigen;

void ControlTaskPriority::reset_control(Eigen::VectorXd &q0){
	Eigen::VectorXd zerovec(mode==ControlMode::JOINT_CONTROL? JOINT_DOF : TASK_DOF);
	Eigen::VectorXd zerovec_task(TASK_DOF);
	zerovec.setZero();
	zerovec_task.setZero();
	lpf_pos.reset(q0);
	lpf_vel.reset(zerovec);
	lpf_acc.reset(zerovec);
	lpf_ft.reset(zerovec_task);
    lpf_tauExt.reset(zerovec);
	eI_1.setZero();
	eI_2.setZero();
	qdot_fin_prev.setZero();
}

int ControlTaskPriority::calculate_joint_control_torque(
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
	printf("=========== Here 1 =============");
	Eigen::VectorXd q_lpf = lpf_pos.update(q);
	Eigen::VectorXd qdot_lpf = lpf_vel.update(qdot);
	Eigen::VectorXd qddot_lpf = lpf_acc.update(qddot);
	Eigen::VectorXd FText_lpf = lpf_ft.update(FText);
	Eigen::VectorXd tauExt_pure = tauExt-tauGrav;
    Eigen::VectorXd tauExt_lpf = lpf_tauExt.update(tauExt_pure);

    // This code is based on "Multiple tasks manipulation for a robotic manipulator (2003)"

    // Declaration of Jacobian matrices of each task

    J_1 = J.block(0,0,5,JOINT_DOF); // 5xJOINT_DOF
	J_2 << 1,1,1,1,1,1; // maintaining joint sum 180[deg] 1xJOINT_DOF
    // Force/impedance control code is as follwos, (when using, check desired task velocity part)
	//    J_1 = J.block(0,0,5,JOINT_DOF); // xyuvw 5xJOINT_DOF
	//    J_1(0,0,2,JOINT_DOF) = J.block(0,0,2,JOINT_DOF);
	//    J_1(2,0,3,JOINT_DOF) = J.block(3,0,3,JOINT_DOF);
	//	  J_2 = J.block(2,0,1,JOINT_DOF); // z direction 1xJOINT_DOF

	// Weighted matrix
	W = J_1.transpose()*J_1 + J_2.transpose()*J_2; // JOINT_DOFxJOINT_DOF
	J_1W = W.inverse()*J_1.transpose()*( J_1*W.inverse()*J_1.transpose() ).inverse(); // JOINT_DOFx5
//	J_2W = W.inverse()*J_2.transpose()*( J_2*W.inverse()*J_2.transpose() ).inverse();

	// pseudo-inverse of J2
	Jinv_2 = J_2.completeOrthogonalDecomposition().pseudoInverse(); // JOINT_DOFx1

	// inclined projection matirces
	P_1W = MatrixXd::Identity(JOINT_DOF, JOINT_DOF) - J_1W*J_1; // JOINT_DOFxJOINT_DOF

	// joint-task space relationship
	pdot_1 = J_1 * qdot_lpf; // 5x1
	pdot_2 = J_2 * qdot_lpf; // 1x1
	pdot_d1 = J_1 * qdot_d; // 5x1
	pdot_d2 = J_2 * qdot_d; // 1x1

	// calculating error
	e_1 = J_1 * (q_d-q_lpf); // 5x1
	edot_1 = J_1 * (qdot_d-qdot_lpf);
	eI_1 += (e_1)*DT;
	e_2 = J_2 * (q_d-q_lpf); // 1x1
	edot_2 = J_2 * (qdot_d-qdot_lpf);
	eI_2 += (e_2)*DT;

	// position PID controller
	pdot_ref1 = pdot_d1 + kP_1[0] * e_1 + kP_1[1] * eI_1 + kP_1[2] * edot_1; // 5x1
	pdot_ref2 = pdot_d2 + kP_2[0] * e_2 + kP_2[1] * eI_2 + kP_2[2] * edot_2; // 1x1

	// finally calculated velocity
	qdot_fin = J_1W * pdot_ref1 + P_1W * Jinv_2 * pdot_ref2; // (JOINT_DOFx5)x5x1 + (JOINT_DOFxJOINT_DOF)xJOINT_DOFx1x1x1
	qddot_fin = (qdot_fin - qdot_fin_prev) / DT;

	// external torque is not included
	//	qddot_ref = M_n.inverse()*(-C_n*qdot_n + tauPD + FText_scale*(J.transpose() * FText_lpf - tauExt_lpf));

	// switching control mode
	if (control_force>0.5){
		torque_out = M*qddot_fin + C*qdot_fin + tauGrav;
	}
	else{
		torque_out = tauGrav;
	}

	// update final joint velocity
	qdot_fin_prev = qdot_fin;

	// torque limit
	for (int i=0; i<JOINT_DOF; i++){
		if(tauMax[i]>0.001){
			torque_out[i] = torque_out[i] > tauMax[i] ? tauMax[i] : torque_out[i];
			torque_out[i] = torque_out[i] < - tauMax[i] ? - tauMax[i] : torque_out[i];
		}
	}

	return 0;
}

int ControlTaskPriority::calculate_task_control_torque(
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
	torque_out = (tauGrav);
	return 0;
}

/**
 * @remark apply the gain values
 */
void ControlTaskPriority::apply_gain_values() {

	std::cout<<"============== apply lpf freq " << std::to_string(f_cut) << " ==============="<<std::endl;
	lpf_pos.cutoff_freq = f_cut;
	lpf_vel.cutoff_freq = f_cut;
	lpf_acc.cutoff_freq = f_cut;
	lpf_ft.cutoff_freq = f_cut_force;
	lpf_tauExt.cutoff_freq = f_cut;
	std::cout<<"============== apply lpf freq done ==============="<<std::endl;
}

EXPORT_CONTROLLER(ControlTaskPriority)
