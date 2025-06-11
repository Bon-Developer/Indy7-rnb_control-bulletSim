#ifndef Hybrid_Etank_H_
#define Hybrid_Etank_H_

#include "../../control_hub/controller_interface.h"
#include <iostream>

#define PI 3.141592
#define F_CUT_DEFAULT 1000

namespace RNB {

class ControlHybrid_Etank : public ControllerInterface{
public:
	Eigen::MatrixXd K, Kp, KI;		 /**< @brief Unused gain */
	double kP, kI; 				 	 /**< @brief PD gain of force tracking controller */
	double k_v, k_a; 				 /**< @brief velocity and acceleration gain for second task */
    Eigen::MatrixXd JointIdentity;	 /**< @brief Identity matrix (JOINT_DOF) */
	double kp_diag; 				 /**< @brief p-gain for impedance controller */
	double kd_diag;					 /**< @brief d-gain for impedance controller */
	double f_cut; 					 /**< @brief cutoff frequency for low-pass filters*/
	double th_sensor; 				 /**< @brief Sensor rotation */
	double FText_scale;
	Eigen::VectorXd F_refvec;		 /**< @brief target force in task space */

	double T;						 /**< @brief Tank energy */
	double alpha;
	double beta;
	double limit_u;					 /**< @brief upper limit of energy tank */
	double limit_l;					 /**< @brief lower limit of energy tank */

	Eigen::VectorXd pdot;			 /**< @brief velocity in Task space */

	Eigen::VectorXd edot, e;		 /**< @brief Joint error between q and q_d */
	Eigen::VectorXd edot_p, e_p;	 /**< @brief Task error obtained by q and q_d */

	Eigen::VectorXd e_force;		 /**< @brief error between reference and measured force */
	Eigen::VectorXd eI_force;		 /**< @brief error integral between reference and measured force */

	Eigen::MatrixXd J_task;			 /**< @brief Jacobian matrix corresponding to xyz dimensions */

	Eigen::VectorXd F_imp;			 /**< @brief force calculated by impedance controller */
	Eigen::VectorXd Fz;				 /**< @brief z-directional force of F_imp */
	Eigen::VectorXd tau_imp;		 /**< @brief target torque corresponding to Fz */
	Eigen::VectorXd F_track;	 	 /**< @brief force calculated by force controller */
	Eigen::VectorXd tau_force;		 /**< @brief target torque corresponding to tau_force */
	Eigen::VectorXd tau_fi;			 /**< @brief sum of tau_imp and tau_force */

	LowPassFilter<Eigen::VectorXd> lpf_pos;		/**< @brief low-pass filtered position */
	LowPassFilter<Eigen::VectorXd> lpf_vel; 	/**< @brief low-pass filtered velocity */
	LowPassFilter<Eigen::VectorXd> lpf_acc; 	/**< @brief low-pass filtered acceleration */
    LowPassFilter<Eigen::VectorXd> lpf_ft; 		/**< @brief low-pass filtered FT senser values */
    LowPassFilter<Eigen::VectorXd> lpf_tauExt; 	/**< @brief low-pass filtered FT senser values */

	Eigen::MatrixXd Rot; /**< @brief rotation matrix from base to end-tool */

    Eigen::VectorXd tauMax; /**< @brief preallocated matrix for auxiliary torque */

	/**
	 * @param 	JOINT_DOF	degree of freedom for joint space
	 * @param 	TASK_DOF	degree of freedom for task space
	 */
	ControlHybrid_Etank(ControlMode mode, int JOINT_DOF, int TASK_DOF, double DT):
		ControllerInterface(mode, JOINT_DOF, TASK_DOF, DT, true, true, false, true),
		lpf_pos(DT, F_CUT_DEFAULT), lpf_vel(DT, F_CUT_DEFAULT), lpf_acc(DT, F_CUT_DEFAULT),
		lpf_ft(DT, F_CUT_DEFAULT), lpf_tauExt(DT, F_CUT_DEFAULT),
		JointIdentity(JOINT_DOF, JOINT_DOF)
	{
		printf("============================= Hybrid_Etank =============================\n");
		printf("================= %dD Joints / %dD Task space / DT = %f ==============\n", JOINT_DOF, TASK_DOF, DT);


		JointIdentity.setIdentity();
        register_gain("tauMax", &tauMax, JOINT_DOF, 0);
		register_gain("f_cut", &f_cut, F_CUT_DEFAULT);
		switch(mode){
		case ControlMode::JOINT_CONTROL:
			printf("====================== ControlMode: JOINT_CONTROL ========================\n");
			register_gain("K", &K, JOINT_DOF, 0);
			register_gain("Kp", &Kp, JOINT_DOF, 0);
			register_gain("KI", &KI, JOINT_DOF, 0);

			register_gain("kP", &kP, 0);
			register_gain("kI", &kI, 0);

			register_gain("kp_diag", &kp_diag, 0);
			register_gain("kd_diag", &kd_diag, 0);

			register_gain("FText_scale", &FText_scale, 1);
			register_gain("F_ref", &F_refvec, TASK_DOF, 0);

			e_force = Eigen::VectorXd(TASK_DOF);
			eI_force = Eigen::VectorXd(TASK_DOF);
			break;
		case ControlMode::TASK_CONTROL:
			printf("======================= ControlMode: TASK_CONTROL ========================\n");
			e_force = Eigen::VectorXd(TASK_DOF);
			eI_force = Eigen::VectorXd(TASK_DOF);
			break;
		}
		printf("==========================================================================\n");
        apply_gain_values();
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
	 * @param	p			current task pose (VectorXd(TASK_DOF))
	 * @param	p_n			nominal task pose (VectorXd(TASK_DOF))
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
	 */
	void reset_control(Eigen::VectorXd &q0) override;

};

}
#endif /* NRIC_Force_H_ */
