#include "PD.h"
#include <string.h>
#include <iostream>

using namespace RNB;

void ControlAlgorithmPD::reset_control(Eigen::VectorXd &x0){
	lpf_pos.reset();
	lpf_vel.reset();
	lpf_acc.reset();
}

int ControlAlgorithmPD::calculate_joint_control_torque(
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

	// calculating error
	e_dr = q_d-q_lpf;
	edot_dr = qdot_d-qdot_lpf;

	qddot_n = (qddot_d + (KvJoint*edot_dr) + (KpJoint*e_dr));
	M += M_J;
	tauPD = M*qddot_n + C*qdot_lpf;

	// input torque
	torque_out = (tauGrav + tauPD);
	qddot_n = qddot;
	return 0;
}

int ControlAlgorithmPD::calculate_task_control_torque(
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
    Eigen::VectorXd qdot_lpf = lpf_vel.update(qdot);

    Eigen::VectorXd pdot = J * qdot_lpf;

	// calculating error
    e_dr = alg_p->diff_in_alg(p_d, p);
	edot_dr = alg_p->diff_in_alg(p_d, p_d + pdot_d*DT)/DT - alg_p->diff_in_alg(p, p + pdot*DT)/DT;

	Eigen::VectorXd effort = (KvTask*edot_dr) + (KpTask*e_dr);
	tauPD = -J.transpose()*effort;
	qddot_n = qddot;

	// input torque
	torque_out = (tauGrav + tauPD);
	return 0;
}

/**
 * @remark apply the gain values
 */
void ControlAlgorithmPD::apply_gain_values() {
	std::cout<<"============== apply lpf freq " << std::to_string(f_cut) << " ==============="<<std::endl;
	lpf_pos.cutoff_freq = f_cut;
	lpf_vel.cutoff_freq = f_cut;
	lpf_acc.cutoff_freq = f_cut;
	std::cout<<"============== apply lpf freq done ==============="<<std::endl;
}

EXPORT_CONTROLLER(ControlAlgorithmPD)
