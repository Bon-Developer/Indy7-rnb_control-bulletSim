#include "NRIC_Force.h"
#include <string.h>
#include <iostream>
using namespace RNB;
using namespace Eigen;

void ControlNRIC_Force::reset_control(Eigen::VectorXd &x0){
	lpf_pos.reset();
	lpf_vel.reset();
	lpf_acc.reset();
	lpf_ft.reset();
    lpf_tauExt.reset();
	eI_nr.setZero();
}

int ControlNRIC_Force::calculate_joint_control_torque(
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
    Eigen::VectorXd tauExt_lpf = lpf_tauExt.update(tauExt);

	double Imp_lpf = lpf_Imp.update((int)switch_control>0.5);
	double Fz_lpf = lpf_Fz.update((int)switch_control>1.5);

	// rotation matrix
	Rot = eulerZYX(p(5), p(4), p(3));
	VectorXd R_31 = Rot.block(0,2,3,1);
	MatrixXd Rot_block = Rot.transpose() * J.block(0,0,3,JOINT_DOF);

    FText_lpf.block(0, 0, 3, 1) << Rot*FText_lpf.block(0, 0, 3, 1); // FText in base_frame
    FText_lpf.block(3, 0, 3, 1) << Rot*FText_lpf.block(3, 0, 3, 1); // FText in base_frame

            // calculating torque output of PD controller (outer-loop position controller)
	tauPD = kp_diag*(q_d - q_n) + kv_diag*(-qdot_n);

	// set nominal model as real model for compensating friction
	for (int i = 0; i < JOINT_DOF; i++) {
	    M_n(i,i) *= M_J[i];
	}

	// weight matrix
		if (Weight == 0){
			W = Eigen::MatrixXd::Identity(JOINT_DOF, JOINT_DOF);
		}
		else {
			W = M_n;
		}

	// reference force
	Vector3d F_unit; // unit vector
	F_mag = sqrt( pow(F_refvec[0],2) + pow(F_refvec[1],2) + pow(F_refvec[2],2) );

	if (F_mag==0) {
		F_unit = Eigen::MatrixXd::Zero(3,1);
		F_unit[2] = 1;
	}
	else{
		F_unit = - F_refvec.block(0,0,3,1) / F_mag;
	}

//	F_rot = Rot * F_unit; // absolute coordinate -> current coordinate Rot: 3by3, F_unit : 3by1

	// main task Jacobian (x/y/z axis)
	if(switch_frame < 0.5){
		// Task1: end-tool frame F_unit direction force
		F_unit = (Rot*F_unit); // change F_unit to base frame
	}
	// Task1: F_unit direction force (base frame)
	F_b =	FText_lpf.block(0,0,3,1); // calculate F in base coordsIP in end-effector coords
	IP = F_unit.dot(F_b);
	J_1 = - (F_unit[0] * J.block(0,0,1,JOINT_DOF) + F_unit[1] * J.block(1,0,1,JOINT_DOF)
			+ F_unit[2] * J.block(2,0,1,JOINT_DOF));

	// weighted pseudo inverse of Jacobian 1
	Eigen::VectorXd J_1_part = J_1*W.inverse()*J_1.transpose();
	J_1_inv = W.inverse()*J_1.transpose()*J_1_part.inverse();
		// J_1_inv = W*J_1.transpose()*(J_1*W*J_1.transpose()).inverse();

	// N_2
	Eigen::MatrixXd I = Eigen::MatrixXd::Identity(JOINT_DOF, JOINT_DOF);
	Eigen::MatrixXd JJ_1 = J_1.transpose()*J_1_inv.transpose();
	N_2 = I - JJ_1;

	// calculating error
	e_nr = q_n-q_lpf;
	edot_nr = qdot_n-qdot_lpf;
	if (Imp_lpf>switch_thresh_up){
		eI_nr += N_2*(e_nr)*DT;
	}
	else{
		eI_nr += (e_nr)*DT;
	}

	// calculating auxiliary torque, apply cwiseSqrt to reduce deviation between links

	tauAux = (K+JointIdentity/(gamma*gamma))*(edot_nr + Kp*e_nr +  KI*eI_nr);

	// force control (arbitrary direction)
	e_f = F_mag-IP; // refvec
	edot_f = (e_f - e_f_prev)/DT;

	// switching control mode 0: joint control, 1: F-direction impedance, 2: Force + impedance
	if (Imp_lpf>switch_thresh_up){
		if(switch_control>1.5 && Fz_lpf>(1.0-switch_thresh_up)){
			eI_f += (e_f)*DT; // apply force error integral after Fz_lpf converge 1
		}
		if(Fz_lpf<switch_thresh_down){
			eI_f = 0; // set force error integral zero after Fz_lpf converge 0
		}

		// FF+FB
		Fz = Fz_lpf*(kP*e_f + kI*eI_f + kD*edot_f);	// Closed loop
//		Fz = kP*F_mag; 									// Open loop
		tauC = J_1.transpose() * (Fz) + (N_2 + (1.0-Imp_lpf) * JJ_1) * tauPD;

		Eigen::VectorXd tauImp = Imp_lpf * JJ_1 * (Kdamp*edot_nr + Kspring*e_nr);

		torque_out = (
				tauGrav + N_2*tauAux + (1.0-Imp_lpf)*JJ_1*tauAux
				+ tauImp + tauC);
	}
	else{
		eI_f = 0;
		// input torque
		torque_out = (tauGrav + tauAux + tauPD);
	}

	qddot_n = M_n.inverse()*(-C_n*qdot_n + tauPD + FText_scale*(J.transpose() * FText_lpf - tauExt_lpf));


	// update force error
	e_f_prev = e_f;

	// plotting
	custom_dat[0] = IP;
	custom_dat[1] = Imp_lpf;
	custom_dat[2] = Fz_lpf;
	custom_dat[3] = switch_thresh_up;
	custom_dat[4] = switch_thresh_down;

	// torque limit
	for (int i=0; i<JOINT_DOF; i++){
		if(tauMax[i]>0.001){
			torque_out[i] = torque_out[i] > tauMax[i] ? tauMax[i] : torque_out[i];
			torque_out[i] = torque_out[i] < - tauMax[i] ? - tauMax[i] : torque_out[i];
		}
	}
	return 0;
}

int ControlNRIC_Force::calculate_task_control_torque(
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
void ControlNRIC_Force::apply_gain_values() {

	std::cout<<"============== apply lpf freq " << std::to_string(f_cut) << " ==============="<<std::endl;
	lpf_pos.cutoff_freq = f_cut;
	lpf_vel.cutoff_freq = f_cut;
	lpf_acc.cutoff_freq = f_cut;
	lpf_ft.cutoff_freq = f_cut_force;
	lpf_tauExt.cutoff_freq = f_cut;
	lpf_Fz.cutoff_freq = switch_f_cut;
	lpf_Imp.cutoff_freq = switch_f_cut;
	std::cout<<"============== apply lpf freq done ==============="<<std::endl;
}

EXPORT_CONTROLLER(ControlNRIC_Force)
