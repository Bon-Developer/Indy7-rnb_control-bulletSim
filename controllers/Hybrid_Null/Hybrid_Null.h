#ifndef Hybrid_Null_H_
#define Hybrid_Null_H_

#include "../../control_hub/controller_interface.h"
#include <iostream>

#define PI 3.141592
#define F_CUT_DEFAULT 1000
#define F_CUT_FORCE_DEFAULT 100

namespace RNB {

class ControlHybrid_Null : public ControllerInterface{
public:
	// gains & inputs
	double kP, kI;						/**< @brief PD gain of force tracking controller */
	double k_v, k_a;					/**< @brief velocity and acceleration gain for second task */
	Eigen::MatrixXd kp_diag, kd_diag;	/**< @brief impedance control gains */
	double f_cut; 					 	/**< @brief cutoff frequency for low-pass filters*/
	double f_cut_force; 	 	     	/**< @brief cutoff frequency of external force*/
	Eigen::VectorXd F_ref;				/**< @brief reference force vector */
	double M_diag;						/**< @brief inertia reshaping */
	double K;							/**< @brief virtual spring in force direction */
	double D;							/**< @brief damping injection in force direction */

	// switches
	double switch_control;				/**< @brief control on/off */
	double switch_frame;				/**< @brief space decoupling end-tool/global */
	double switch_feedforward;			/**< @brief force feed-forward on/off */
	double switch_damping;				/**< @brief damping injection on/off */
	double switch_passivity;			/**< @brief passivity-based approach on/off */

	// variables
	double F;							/**< @brief exerted force */
	double e_f, e_f_prev;				/**< @brief force error */
	double edot_f;						/**< @brief derivative of e_f */
	double eI_f;						/**< @brief integral of e_f */
	Eigen::VectorXd edot, e;			/**< @brief Joint error between q and q_d */
	double F_mag;						/**< @brief magnitude of F_ref */
	double IP;
	Eigen::VectorXd F_unit;				/**< @brief unit vector of F_ref (3) */
	Eigen::VectorXd F_rot;				/**< @brief  */
	Eigen::VectorXd Fext;				/**< @brief  */

	Eigen::MatrixXd JointIdentity;		/**< @brief Identity matrix (JOINT_DOF) */

	Eigen::MatrixXd Rot;				/**< @brief rotation matrix from base to end-tool */

	Eigen::MatrixXd M_mod;			 	/**< @brief modified inertia matrix */
	Eigen::MatrixXd W;					/**< @brief weight matrix of Jacobian inverse */
	Eigen::MatrixXd J_1;				/**< @brief main task Jacobian (m by n) */
	Eigen::MatrixXd J_1_inv;			/**< @brief inverse of J_1 */
	Eigen::MatrixXd N_2;				/**< @brief null space projector of J_1  */

	Eigen::VectorXd tauImp;		 		/**< @brief impedance control input */
	Eigen::VectorXd tau_1;		 		/**< @brief task 1 torque : force control */
	Eigen::VectorXd tau_2;				/**< @brief task 2 torque : impedance control */

	Eigen::VectorXd tauMax; 			/**< @brief preallocated matrix for auxiliary torque */

	Eigen::VectorXd pdot_1;				/**< @brief velocity of task 1 */
	Eigen::VectorXd tauDamp;			/**< @brief damping force in force direction */
	Eigen::VectorXd tauSpring;			/**< @brief spring force in force direction */


	LowPassFilter<Eigen::VectorXd> lpf_pos;		/**< @brief low-pass filtered position */
	LowPassFilter<Eigen::VectorXd> lpf_vel; 	/**< @brief low-pass filtered velocity */
	LowPassFilter<Eigen::VectorXd> lpf_acc; 	/**< @brief low-pass filtered acceleration */
    LowPassFilter<Eigen::VectorXd> lpf_ft; 		/**< @brief low-pass filtered FT senser values */
    LowPassFilter<Eigen::VectorXd> lpf_tauExt; 	/**< @brief low-pass filtered FT senser values */

	/**
	 * @param 	JOINT_DOF	degree of freedom for joint space
	 * @param 	TASK_DOF	degree of freedom for task space
	 */
	ControlHybrid_Null(ControlMode mode, int JOINT_DOF, int TASK_DOF, double DT):
		ControllerInterface(mode, JOINT_DOF, TASK_DOF, DT, true, true, false, true),
		lpf_pos(DT, F_CUT_DEFAULT), lpf_vel(DT, F_CUT_DEFAULT), lpf_acc(DT, F_CUT_DEFAULT),
		lpf_ft(DT, F_CUT_DEFAULT), lpf_tauExt(DT, F_CUT_DEFAULT),
		JointIdentity(JOINT_DOF, JOINT_DOF),
		M_mod(JOINT_DOF, JOINT_DOF), J_1(1, JOINT_DOF), F_unit(3), Fext(3)

	{
		printf("============================= Hybrid_Null =============================\n");
		printf("================= %dD Joints / %dD Task space / DT = %f ==============\n", JOINT_DOF, TASK_DOF, DT);


		JointIdentity.setIdentity();
        register_gain("tauMax", &tauMax, JOINT_DOF, 0);
		register_gain("f_cut", &f_cut, F_CUT_DEFAULT);
		register_gain("f_cut_force", &f_cut_force, F_CUT_FORCE_DEFAULT);
		switch(mode){
		case ControlMode::JOINT_CONTROL:
			printf("====================== ControlMode: JOINT_CONTROL ========================\n");

			register_gain("kP", &kP, 0);
			register_gain("kI", &kI, 0);

			register_gain("M_diag", &M_diag, 0);

			register_gain("kp_diag", &kp_diag, JOINT_DOF, 0);
			register_gain("kd_diag", &kd_diag, JOINT_DOF, 0);

			register_gain("Spring", &K, 0);
			register_gain("Damping", &D, 0);

			register_gain("switch_control", &switch_control, 0);
			register_gain("switch_frame", &switch_frame, 0);
			register_gain("switch_feedforward", &switch_feedforward, 0);

			register_gain("F_ref", &F_ref, TASK_DOF, 0);

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

#endif /* NRIC_Force_H_ */
