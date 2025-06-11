#include <string.h>
#include <Eigen/Dense>
#include <iostream>

#include "Hybrid_Null.h"
using namespace RNB;
using namespace Eigen;

void ControlHybrid_Null::reset_control(Eigen::VectorXd &q0){
	Eigen::VectorXd zerovec(mode==ControlMode::JOINT_CONTROL? JOINT_DOF : TASK_DOF);
	Eigen::VectorXd zerovec_task(TASK_DOF);
	zerovec.setZero();
	zerovec_task.setZero();
	lpf_pos.reset(q0);
	lpf_vel.reset(zerovec);
	lpf_acc.reset(zerovec);
	lpf_ft.reset(zerovec_task);
    lpf_tauExt.reset(zerovec);
	eI_f = 0;
}

int ControlHybrid_Null::calculate_joint_control_torque(
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
	Eigen::VectorXd q_lpf = lpf_pos.update(q);
	Eigen::VectorXd qdot_lpf = lpf_vel.update(qdot);
	Eigen::VectorXd qddot_lpf = lpf_acc.update(qddot);
	Eigen::VectorXd FText_lpf = lpf_ft.update(FText);
	Eigen::VectorXd tauExt_pure = tauExt-tauGrav;
    Eigen::VectorXd tauExt_lpf = lpf_tauExt.update(tauExt_pure);

    // In the case of redundant manipulator, null motion exists

    // rotation matrix
	Rot = eulerZYX(p_n(5), p_n(4), p_n(3)); // absolute to end-tool
	MatrixXd Rot_block = Rot.transpose() * J_n.block(0,0,3,JOINT_DOF);

    // set-point control
    qdot_d = MatrixXd::Zero(JOINT_DOF, 1);

    // joint space error
    e = q_d - q;
	edot = qdot_d - qdot_lpf;

    // inertia modifying
    M_mod = M_n + M_diag * JointIdentity;

    // weight matrix of Jacobian
	W = M_mod;

	// reference force vector
	double F_mag = sqrt( pow(F_ref[0],2) + pow(F_ref[1],2) + pow(F_ref[2],2) );
	if (F_mag<0.01) {F_unit = Eigen::MatrixXd::Zero(3,1);}
	else{F_unit = F_ref.block(0,0,3,1) / F_mag;}

	// main task Jacobian (switch on/off <=> end-tool / frame)
	J_1.setZero(1,JOINT_DOF);
	if(switch_frame < 0.5){
		J_1 = F_unit[1] * Rot_block.block(0,0,1,JOINT_DOF) + F_unit[0] * Rot_block.block(1,0,1,JOINT_DOF)
				+ F_unit[2] * Rot_block.block(2,0,1,JOINT_DOF);
	}
	else{
		J_1 = F_unit[1] * J_n.block(0,0,1,JOINT_DOF) + F_unit[0] * J_n.block(1,0,1,JOINT_DOF)
				+ F_unit[2] * J_n.block(2,0,1,JOINT_DOF);
	}

	// weighted pseudo inverse of Jacobian 1
	Eigen::VectorXd J_1_part = J_1*W.inverse()*J_1.transpose();
	J_1_inv = W.inverse()*J_1.transpose()*J_1_part.inverse();

	// null space projection
	N_2 = JointIdentity - J_1.transpose()*J_1_inv.transpose();

    // force error
	if(switch_frame < 0.5){
		IP = F_unit[0] * FText_lpf[0] + F_unit[1] * FText_lpf[1] + F_unit[2] * FText_lpf[2];
		e_f = F_unit[0] * F_ref[0] + F_unit[1] * F_ref[1] + (F_unit[2] * F_ref[2]) - (IP);
	}
	else{
		F_rot =	Rot.transpose() * FText_lpf.block(0,0,3,1);
		IP = F_unit[0] * F_rot[0] + F_unit[1] * F_rot[1] + F_unit[2] * F_rot[2];
		e_f = (F_unit[0] * -F_ref[0]) + (F_unit[1] * -F_ref[1]) + (F_unit[2] * -F_ref[2]) - (IP);
	}
	eI_f += (e_f)*DT;

    // force controller
	F = kP * e_f + kI * eI_f;
    tau_1 = J_1.transpose() * F;

    // position controller
    tauImp = (kp_diag*e + kd_diag*edot);
	tau_2 = N_2 * tauImp;

	// damping injection
	tauDamp = J_1.transpose() * (D * J_1 * edot);
	tauSpring = J_1.transpose() * ( K * J_1 * e);

    // input torque
    if (switch_control < 0.5){torque_out = (tauGrav + tauImp);}
    else{torque_out = (tauGrav + tau_1 + tau_2 + tauDamp + tauSpring);}

    // plotting
    custom_dat[0] = IP;

    // update
    qddot_n.setZero(JOINT_DOF,1);

	// torque limit
	for (int i=0; i<JOINT_DOF; i++){
		if(tauMax[i]>0.001){
			torque_out[i] = torque_out[i] > tauMax[i] ? tauMax[i] : torque_out[i];
			torque_out[i] = torque_out[i] < - tauMax[i] ? - tauMax[i] : torque_out[i];
		}
	}
	return 0;

}

int ControlHybrid_Null::calculate_task_control_torque(
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
void ControlHybrid_Null::apply_gain_values() {

	std::cout<<"============== apply lpf freq " << std::to_string(f_cut) << " ==============="<<std::endl;
	lpf_pos.cutoff_freq = f_cut;
	lpf_vel.cutoff_freq = f_cut;
	lpf_acc.cutoff_freq = f_cut;
	lpf_ft.cutoff_freq = f_cut_force;
	lpf_tauExt.cutoff_freq = f_cut;
	std::cout<<"============== apply lpf freq done ==============="<<std::endl;
}

EXPORT_CONTROLLER(ControlHybrid_Null)
