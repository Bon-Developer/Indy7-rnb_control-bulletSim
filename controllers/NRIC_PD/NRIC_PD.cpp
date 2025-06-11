#include "NRIC_PD.h"
#include <string.h>
#include <iostream>
using namespace RNB;

void ControlNRIC_PD::reset_control(Eigen::VectorXd &x0){
	lpf_pos.reset();
	lpf_vel.reset();
	lpf_acc.reset();
	lpf_ft.reset();
    lpf_tauExt.reset();
	eI_nr.setZero();
}

int ControlNRIC_PD::calculate_joint_control_torque(
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
    q += lpf_Qoff.update(q_off);
	Eigen::VectorXd q_lpf = lpf_pos.update(q);
	Eigen::VectorXd qdot_lpf = lpf_vel.update(qdot);
	Eigen::VectorXd qddot_lpf = lpf_acc.update(qddot);
	Eigen::VectorXd FText_lpf = lpf_ft.update(FText);
    Eigen::VectorXd tauExt_lpf = lpf_tauExt.update(tauExt);

	// calculating error
	e_nr = q_n-q_lpf;
	edot_nr = qdot_n-qdot_lpf;
	eI_nr += (e_nr)*DT;

	// calculating torque output of PD controller (outer-loop position controller)
	tauPD = kp_diag*(q_d - q_n) + kv_diag*(-qdot_n);

	// set nominal model
	for (int i = 0; i < JOINT_DOF; i++) {
	    M_n(i,i) *= M_J[i];
	}

	// using only force components of FT sensor (x, y, z order)
	//	Eigen::Vector3d FText_3;
	for(int i=3; i<TASK_DOF; i++)	{
		FText_lpf[i] = 0;
		for(int j=0; j<JOINT_DOF;j++){
			J(i,j) = 0;
		}
	}

	FText_prev_x = FText_lpf(0);
	FText_prev_y = FText_lpf(1);
	FText_lpf(0) = cos(th_sensor*PI/180) * FText_prev_x + sin(th_sensor*PI/180) * FText_prev_y;
	FText_lpf(1) = -sin(th_sensor*PI/180) * FText_prev_x + cos(th_sensor*PI/180) * FText_prev_y;

    Eigen::Matrix3d Rot = eulerZYX(p_n(5), p_n(4), p_n(3));
    FText_lpf.block(0, 0, 3, 1) << Rot*FText_lpf.block(0, 0, 3, 1);
    FText_lpf.block(3, 0, 3, 1) << Rot*FText_lpf.block(3, 0, 3, 1);

//	+ J.transpose() * FText_3

	// update nominal model by calculating nominal acceleration
	qddot_n = M_n.inverse()*(-C_n*qdot_n + tauPD + FText_scale*(J.transpose() * FText_lpf - tauExt_lpf));

	// calculating auxiliary torque, apply cwiseSqrt to reduce deviation between links
	tauAux = (K+JointIdentity/(gamma*gamma))*(edot_nr + Kp*e_nr + KI*eI_nr);

	// input torque
	torque_out = (tauGrav + tauPD + tauAux);
	for (int i=0; i<JOINT_DOF; i++){
		if(tauMax[i]>0.001){
			torque_out[i] = torque_out[i] > tauMax[i] ? tauMax[i] : torque_out[i];
			torque_out[i] = torque_out[i] < - tauMax[i] ? - tauMax[i] : torque_out[i];
		}
	}
	return 0;
}

int ControlNRIC_PD::calculate_task_control_torque(
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
    q += q_off;
	qddot_n.setZero();
	torque_out = (tauGrav);
	return 0;
}

/**
 * @remark apply the gain values
 */
void ControlNRIC_PD::apply_gain_values() {
	std::cout<<"============== apply lpf freq " << std::to_string(f_cut) << " ==============="<<std::endl;
	lpf_pos.cutoff_freq = f_cut;
	lpf_vel.cutoff_freq = f_cut;
	lpf_acc.cutoff_freq = f_cut;
	lpf_ft.cutoff_freq = f_cut;
	lpf_tauExt.cutoff_freq = f_cut;
	std::cout<<"============== apply lpf freq done ==============="<<std::endl;
}

EXPORT_CONTROLLER(ControlNRIC_PD)
