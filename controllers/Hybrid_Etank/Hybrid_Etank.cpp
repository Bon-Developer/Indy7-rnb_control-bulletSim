#include <string.h>
#include <Eigen/Dense>
#include <iostream>

#include "Hybrid_Etank.h"
using namespace RNB;
using namespace Eigen;

void ControlHybrid_Etank::reset_control(Eigen::VectorXd &q0){
	Eigen::VectorXd zerovec(mode==ControlMode::JOINT_CONTROL? JOINT_DOF : TASK_DOF);
	Eigen::VectorXd zerovec_task(TASK_DOF);
	zerovec.setZero();
	zerovec_task.setZero();
	lpf_pos.reset(q0);
	lpf_vel.reset(zerovec);
	lpf_acc.reset(zerovec);
	lpf_ft.reset(zerovec_task);
    lpf_tauExt.reset(zerovec);
	eI_force.setZero();
}

int ControlHybrid_Etank::calculate_joint_control_torque(
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

    // task space
    pdot = J * qdot_lpf;

    // calculating error
    e_p = J * (q_d - q);
    e_force = F_refvec - FText_lpf;
    eI_force += (e_force)*DT;

    // rotation matrix
	Rot = eulerZYX(p_n(5), p_n(4), p_n(3)); // size 3x3
	J_task = J.block(0,0,3,JOINT_DOF); // size 3xDOF

    // impedance controller
    F_imp = kp_diag * e_p - kd_diag * pdot;
    tau_imp = J.transpose() * F_imp;

    // force tracking (z-direction)
    F_track = kP * e_force + kI * eI_force; // size 6x1

	Fz = (Rot.transpose()*F_track.block(0,0,3,1)); // 1x3 * 3x1
    tau_force = - J_task.transpose() * Fz; // DOFx3 * 3x1

    tau_fi = tau_imp + tau_force;
    // update nominal model
//    qddot_n = (qddot_d + ());

    // ****energy-tank part
    // tank energy boundary
    if (T<=limit_u){
    	beta = 1;
    }
    else{
    	beta = 0;
    }

    if (T>=limit_l){
        	alpha = 1;
        }
        else{
        	alpha = 0;
        }

    // tank energy
    T = 0.5 * x_t^2;
    w = alpha * (kP * e_force - kI * eI_force) / x_t; // check force error direction
    u_t = - w.transpose() * pdot;
    xdot_t = beta*(pdot.transpose()*kp_diag*pdot)/x_t + u_t;

    // calculated torque
    tau_fi = J.transpose() * (tau_imp - w*x_t);


	// input torque
    torque_out = (tauGrav + tau_fi);

    // update
    x_t += xdot_t*DT;

	// torque limit
	for (int i=0; i<JOINT_DOF; i++){
		if(tauMax[i]>0.001){
			torque_out[i] = torque_out[i] > tauMax[i] ? tauMax[i] : torque_out[i];
			torque_out[i] = torque_out[i] < - tauMax[i] ? - tauMax[i] : torque_out[i];
		}
	}
	return 0;

}

int ControlHybrid_Etank::calculate_task_control_torque(
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
void ControlHybrid_Etank::apply_gain_values() {

	std::cout<<"============== apply lpf freq " << std::to_string(f_cut) << " ==============="<<std::endl;
	lpf_pos.cutoff_freq = f_cut;
	lpf_vel.cutoff_freq = f_cut;
	lpf_acc.cutoff_freq = f_cut;
	lpf_ft.cutoff_freq = f_cut;
	lpf_tauExt.cutoff_freq = f_cut;
	std::cout<<"============== apply lpf freq done ==============="<<std::endl;
}

EXPORT_CONTROLLER(ControlHybrid_Etank)
