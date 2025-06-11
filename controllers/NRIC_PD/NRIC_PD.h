#ifndef NRIC_PD_H_
#define NRIC_PD_H_

#include "../../control_hub/controller_interface.h"
#include <iostream>

#define PI 3.141592
#define F_CUT_DEFAULT 100

namespace RNB {

class ControlNRIC_PD : public ControllerInterface{
public:
	Eigen::MatrixXd K, Kp, KI; /**< @brief PID gain of RIC */
	double gamma; /**< @brief L2 gain */
	Eigen::VectorXd M_J; 	/**< @brief Inertia reshaping gain */
    Eigen::MatrixXd JointIdentity;
	Eigen::MatrixXd kp_diag; /**< @brief P gain for PD controller */
	Eigen::MatrixXd kv_diag; /**< @brief D gain for PD controller */
	double f_cut; 	/**< @brief cutoff frequency for low-pass filters*/
	double th_sensor; 	/**< @brief Sensor rotation */
	double FText_scale;
    Eigen::VectorXd q_off; 	/**< @brief joint offset */

	double FText_prev_x; /**< @brief Save filtered sensor x value for configuration */
	double FText_prev_y; /**< @brief Save filtered sensor y value for configuration */

	Eigen::VectorXd e_nr; /**< @brief error between q and q_n */
	Eigen::VectorXd edot_nr; /**< @brief error between qdot and qdot_n */
	Eigen::VectorXd eI_nr; /**< @brief error integral between q and q_n */

	Eigen::VectorXd tauPD; /**< @brief preallocated matrix for external torque */
	Eigen::VectorXd tauAux; /**< @brief preallocated matrix for auxiliary torque */

	LowPassFilter<Eigen::VectorXd> lpf_pos; /**< @brief low-pass filtered position */
	LowPassFilter<Eigen::VectorXd> lpf_vel; /**< @brief low-pass filtered velocity */
	LowPassFilter<Eigen::VectorXd> lpf_acc; /**< @brief low-pass filtered acceleration */
    LowPassFilter<Eigen::VectorXd> lpf_ft; /**< @brief low-pass filtered FT senser values */
    LowPassFilter<Eigen::VectorXd> lpf_tauExt; /**< @brief low-pass filtered FT senser values */
    LowPassFilter<Eigen::VectorXd> lpf_Qoff; /**< @brief low-pass filtered FT senser values */

	Eigen::VectorXd tauMax; /**< @brief preallocated matrix for auxiliary torque */

	/**
	 * @param 	JOINT_DOF	degree of freedom for joint space
	 * @param 	TASK_DOF	degree of freedom for task space
	 */
	ControlNRIC_PD(ControlMode mode, int JOINT_DOF, int TASK_DOF, double DT):
		ControllerInterface(mode, JOINT_DOF, TASK_DOF, DT, true, true, false, true),
		tauPD(JOINT_DOF), tauAux(JOINT_DOF),
		lpf_pos(DT, F_CUT_DEFAULT), lpf_vel(DT, F_CUT_DEFAULT), lpf_acc(DT, F_CUT_DEFAULT),
		lpf_ft(DT, F_CUT_DEFAULT), lpf_tauExt(DT, F_CUT_DEFAULT), lpf_Qoff(DT, 2),
		JointIdentity(JOINT_DOF, JOINT_DOF)
	{
		printf("============================= ControlNRIC_PD =============================\n");
		printf("================= %dD Joints / %dD Task space / DT = %f ==============\n", JOINT_DOF, TASK_DOF, DT);

		JointIdentity.setIdentity();
		register_gain("f_cut", &f_cut, F_CUT_DEFAULT);
        register_gain("q_off", &q_off, JOINT_DOF, 0);
        register_gain("tauMax", &tauMax, JOINT_DOF, 0);
		switch(mode){
		case ControlMode::JOINT_CONTROL:
			printf("====================== ControlMode: JOINT_CONTROL ========================\n");
			register_gain("K", &K, JOINT_DOF, 0);
			register_gain("Kp", &Kp, JOINT_DOF, 0);
			register_gain("KI", &KI, JOINT_DOF, 0);
			register_gain("Gamma", &gamma, 1);

			register_gain("kp_diag", &kp_diag, JOINT_DOF, 0);
			register_gain("kv_diag", &kv_diag, JOINT_DOF, 0);
			register_gain("M", &M_J, JOINT_DOF, 1);

			register_gain("th_sensor", &th_sensor, 0);
			register_gain("FText_scale", &FText_scale, 0);
			e_nr = Eigen::VectorXd(JOINT_DOF);
			edot_nr = Eigen::VectorXd(JOINT_DOF);
			eI_nr = Eigen::VectorXd(JOINT_DOF);
			break;
		case ControlMode::TASK_CONTROL:
			printf("======================= ControlMode: TASK_CONTROL ========================\n");
			e_nr = Eigen::VectorXd(TASK_DOF);
			edot_nr = Eigen::VectorXd(TASK_DOF);
			eI_nr = Eigen::VectorXd(TASK_DOF);
			break;
		}
        Eigen::VectorXd zerovec(JOINT_DOF);
        zerovec.setZero();
        lpf_Qoff.reset();
        lpf_Qoff.update(zerovec);
		printf("==========================================================================\n");
	}

    /**
     * @brief apply the gain values
     */
    void apply_gain_values() override;

	/**
	 * @brief	(you must override) calculate joint control torque and nominal qddot
	 * @param	q			current joint value (VectorXd(Joint_DOF))
	 * @param	qdot		current joint velocity (VectorXd(Joint_DOF))
	 * @param	qddot		current joint acceleration (VectorXd(Joint_DOF))
	 * @param	q_d			desired joint value (VectorXd(Joint_DOF))
	 * @param	qdot_d		desired joint velocity (VectorXd(Joint_DOF))
	 * @param	qddot_d		desired joint acceleration (VectorXd(Joint_DOF))
	 * @param	q_n			nominal joint value (VectorXd(Joint_DOF))
	 * @param	qdot_n		nominal joint velocity (VectorXd(Joint_DOF))
	 * @param	p			nominal joint value (VectorXd(Joint_DOF))
	 * @param	p_n		nominal joint velocity (VectorXd(Joint_DOF))
	 * @param	M			Inertia matrix (MatrixXd(JOINT_DOF,JOINT_DOF))
	 * @param	C			Coriolis matrix (MatrixXd(JOINT_DOF,JOINT_DOF))
	 * @param	M_n			Nominal Inertia matrix (MatrixXd(JOINT_DOF,JOINT_DOF))
	 * @param	C_n			Nominal Coriolis matrix (MatrixXd(JOINT_DOF,JOINT_DOF))
	 * @param	tauGrav		gravity torque (VectorXd(Joint_DOF))
	 * @param	tauExt		external torque = real torque - (dynamic model torque & gravity) (VectorXd(Joint_DOF))
	 * @param	J			Jacobian matrix (MatrixXd(TASK_DOF,JOINT_DOF))
	 * @param	Jdot		Jacobian dot matrix (MatrixXd(TASK_DOF,JOINT_DOF))
	 * @param	J_n			nominal Jacobian matrix (MatrixXd(TASK_DOF,JOINT_DOF))
	 * @param	Jdot_n		nominal Jacobian dot matrix (MatrixXd(TASK_DOF,JOINT_DOF))
	 * @param	FText		FT sensor input - aligned with and represented in last joint coordinate (VectorXd(TASK_DOF))
	 * @param	qddot_n		nominal joint acceleration output (VectorXd(JOINT_DOF))
	 * @param	torque_out	control torque output (VectorXd(JOINT_DOF))
	 */
    int calculate_joint_control_torque(
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
            Eigen::VectorXd& custom_dat) override;

	/**
	 * @brief	(you must override) calculate task control torque and nominal qddot
	 * @param	p			current task value (VectorXd(Task_DOF))
	 * @param	p_n			nominal task value (VectorXd(Task_DOF))
	 * @param	p_d			desired task value (VectorXd(Task_DOF))
	 * @param	pdot_d		desired task velocity (VectorXd(Task_DOF))
	 * @param	pddot_d		desired task acceleration (VectorXd(Task_DOF))
	 * @param	q			current joint value (VectorXd(Joint_DOF))
	 * @param	qdot		current joint velocity (VectorXd(Joint_DOF))
	 * @param	qddot		current joint acceleration (VectorXd(Joint_DOF))
	 * @param	q_n			nominal joint value (VectorXd(Joint_DOF))
	 * @param	qdot_n		nominal joint velocity (VectorXd(Joint_DOF))
	 * @param	M			Inertia matrix (MatrixXd(JOINT_DOF,JOINT_DOF))
	 * @param	C			Coriolis matrix (MatrixXd(JOINT_DOF,JOINT_DOF))
	 * @param	M_n			Nominal Inertia matrix (MatrixXd(JOINT_DOF,JOINT_DOF))
	 * @param	C_n			Nominal Coriolis matrix (MatrixXd(JOINT_DOF,JOINT_DOF))
	 * @param	tauGrav		gravity torque (VectorXd(Joint_DOF))
	 * @param	tauExt		external torque = real torque - (dynamic model torque & gravity) (VectorXd(Joint_DOF))
	 * @param	J			Jacobian matrix (MatrixXd(TASK_DOF,JOINT_DOF))
	 * @param	Jdot		Jacobian dot matrix (MatrixXd(TASK_DOF,JOINT_DOF))
	 * @param	J_n			nominal Jacobian matrix (MatrixXd(TASK_DOF,JOINT_DOF))
	 * @param	Jdot_n		nominal Jacobian dot matrix (MatrixXd(TASK_DOF,JOINT_DOF))
	 * @param	FText		FT sensor input - aligned with and represented in last joint coordinate (VectorXd(TASK_DOF))
	 * @param	qddot_n		nominal joint acceleration output (VectorXd(JOINT_DOF))
	 * @param	torque_out	control torque output (VectorXd(JOINT_DOF))
	 */
    int calculate_task_control_torque(
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
            Eigen::VectorXd& custom_dat) override;


	/**
	 * @brief	reset controller
	 * @param	x0	initial state to reset. (Joint_DOF,1) if JOINT_CONTROL and (TASK_DOF, 1) if TASK_CONTROL
	 */
	void reset_control(Eigen::VectorXd &x0) override;
};

}
#endif /* NRIC_PD_H_ */
