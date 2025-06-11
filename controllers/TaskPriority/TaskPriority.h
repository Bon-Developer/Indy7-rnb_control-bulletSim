#ifndef TaskPriority_
#define TaskPriority_

#include "../../control_hub/controller_interface.h"
#include <iostream>

#define PI 3.141592
#define F_CUT_DEFAULT 100

namespace RNB {

class ControlTaskPriority : public ControllerInterface{
public:
	Eigen::VectorXd kP_1; 						/**< @brief PID gain of task 1 controller */
	Eigen::VectorXd kP_2; 						/**< @brief PID gain of task 2 controller */
	Eigen::MatrixXd JointIdentity;
	double kp_diag; 							/**< @brief p-gain for desired-nominal control */
	double kv_diag; 							/**< @brief d-gain for desired-nominal control */
	double f_cut; 								/**< @brief cutoff frequency for low-pass filters*/
	double f_cut_force; 						/**< @brief cutoff frequency for low-pass filter of external force*/
	double th_sensor; 							/**< @brief Sensor rotation */
	double FText_scale;
	double control_force;

	Eigen::VectorXd e_1; 						/**< @brief priority 1 error */
	Eigen::VectorXd edot_1; 					/**< @brief priority 1 error derivative */
	Eigen::VectorXd eI_1; 						/**< @brief priority 1 error integration */
	Eigen::VectorXd e_2; 						/**< @brief priority 2 error */
	Eigen::VectorXd edot_2; 					/**< @brief priority 2 error derivative */
	Eigen::VectorXd eI_2; 						/**< @brief priority 2 error integration */

	LowPassFilter<Eigen::VectorXd> lpf_pos; 	/**< @brief low-pass filtered position */
	LowPassFilter<Eigen::VectorXd> lpf_vel; 	/**< @brief low-pass filtered velocity */
	LowPassFilter<Eigen::VectorXd> lpf_acc; 	/**< @brief low-pass filtered acceleration */
    LowPassFilter<Eigen::VectorXd> lpf_ft; 		/**< @brief low-pass filtered FT sensor values */
    LowPassFilter<Eigen::VectorXd> lpf_tauExt;  /**< @brief low-pass filtered FT sensor values */

	Eigen::VectorXd pdot_1; 					/**< @brief velocity of priority 1 in task space */
	Eigen::VectorXd pdot_2; 					/**< @brief velocity of priority 2 in task space */
	Eigen::VectorXd pdot_d1; 					/**< @brief desired velocity in task 1 space */
	Eigen::VectorXd pdot_d2; 					/**< @brief desired velocity in task 2 space */

	Eigen::VectorXd pdot_ref1;  				/**< @brief desired velocity is modified in priority 1 */
	Eigen::VectorXd pdot_ref2;  				/**< @brief desired velocity is modified in priority 2 */
	Eigen::MatrixXd J_1;  						/**< @brief Jacobian matrix of priority 1 */
	Eigen::MatrixXd J_2;  						/**< @brief Jacobian matrix of priority 2 */
	Eigen::MatrixXd W;  						/**< @brief weight matrix */
	Eigen::MatrixXd J_1W;  						/**< @brief weighted pseudo-inverse Jacobian matrix of priority 2 */
	Eigen::MatrixXd J_2W;  						/**< @brief weighted pseudo-inverse Jacobian matrix of priority 2 */
	Eigen::MatrixXd P_1W;  						/**< @brief projection matrix w.r.t task 1 */
	Eigen::MatrixXd Jinv_2;  					/**< @brief pseudo-inverse matrix of J_2 */

	Eigen::VectorXd qdot_fin;  					/**< @brief calculated joint velocity */
	Eigen::VectorXd qdot_fin_prev;  			/**< @brief previous qdot_fin */
	Eigen::VectorXd qddot_fin;  				/**< @brief CTM method */

    Eigen::VectorXd tauMax; 					/**< @brief preallocated matrix for auxiliary torque */

	/**
	 * @param 	JOINT_DOF	degree of freedom for joint space
	 * @param 	TASK_DOF	degree of freedom for task space
	 */
	ControlTaskPriority(ControlMode mode, int JOINT_DOF, int TASK_DOF, double DT):
		ControllerInterface(mode, JOINT_DOF, TASK_DOF, DT, true, true, false, true),
		lpf_pos(DT, F_CUT_DEFAULT), lpf_vel(DT, F_CUT_DEFAULT), lpf_acc(DT, F_CUT_DEFAULT),
		lpf_ft(DT, F_CUT_DEFAULT), lpf_tauExt(DT, F_CUT_DEFAULT),
		JointIdentity(JOINT_DOF, JOINT_DOF),
		eI_1(5), eI_2(1), e_1(5), e_2(1), edot_1(5), edot_2(1),
		pdot_1(5), pdot_2(1), pdot_d1(5), pdot_d2(1),
		pdot_ref1(5), pdot_ref2(1), J_1(5,JOINT_DOF), J_2(1,JOINT_DOF),
		qdot_fin(JOINT_DOF), qddot_fin(JOINT_DOF), qdot_fin_prev(JOINT_DOF)
//		W(JOINT_DOF, JOINT_DOF), J_1W(JOINT_DOF), P_1W(JOINT_DOF, JOINT_DOF)
//		, Jinv_2(JOINT_DOF, TASK_DOF),
	{
		printf("============================= ControlTaskPriority =============================\n");
		printf("================= %dD Joints / %dD Task space / DT = %f ==============\n", JOINT_DOF, TASK_DOF, DT);


		JointIdentity.setIdentity();
        register_gain("tauMax", &tauMax, JOINT_DOF, 0);
		register_gain("f_cut", &f_cut, F_CUT_DEFAULT);
		register_gain("f_cut_force", &f_cut_force, F_CUT_DEFAULT);
		switch(mode){
		case ControlMode::JOINT_CONTROL:
			printf("====================== ControlMode: JOINT_CONTROL ========================\n");

			register_gain("control_force", &control_force, 0);

			register_gain("kP_1", &kP_1, 3, 0);
			register_gain("kP_2", &kP_2, 3, 0);

			register_gain("kp_diag", &kp_diag, 0);
			register_gain("kv_diag", &kv_diag, 0);

			register_gain("FText_scale", &FText_scale, 1);

			break;
		case ControlMode::TASK_CONTROL:
			printf("======================= ControlMode: TASK_CONTROL ========================\n");
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

#endif /* TaskPriority_ */
