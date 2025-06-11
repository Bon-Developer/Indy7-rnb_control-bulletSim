#ifndef NRIC_Force_H_
#define NRIC_Force_H_

#include "../../control_hub/controller_interface.h"
#include <iostream>

#define PI 3.141592
#define F_CUT_DEFAULT 100
#define F_SWITCH_DEFAULT 1

namespace RNB {

class ControlNRIC_Force : public ControllerInterface{
public:
	Eigen::MatrixXd K, Kp, KI; 		/**< @brief PID gain of RIC */
	double gamma; 					/**< @brief L2 gain */
	double kP, kI, kD; 				/**< @brief PID gain of force tracking controller */
	double k_v, k_a; 				/**< @brief velocity and acceleration gain for second task */
	Eigen::VectorXd M_J; 			/**< @brief Inertia reshaping gain */
    Eigen::MatrixXd JointIdentity;

    double IP;						/**< @brief Inner product */
    double F_mag;
	double Fz;
	double switch_frame;

    Eigen::MatrixXd kp_diag;		/**< @brief p-gain for desired-nominal control */
    Eigen::MatrixXd kv_diag;		/**< @brief d-gain for desired-nominal control */
    Eigen::MatrixXd kp_diag_f;		/**< @brief p-gain for desired-nominal control */
    Eigen::MatrixXd kv_diag_f;		/**< @brief d-gain for desired-nominal control */
//	double kp_diag; 				/**< @brief p-gain for desired-nominal control */
//	double kv_diag; 				/**< @brief d-gain for desired-nominal control */
	double f_cut; 					/**< @brief cutoff frequency for low-pass filters*/
	double f_cut_force; 			/**< @brief cutoff frequency for low-pass filter of external force*/
	double th_sensor; 				/**< @brief Sensor rotation */
	double FText_scale;
	double Weight;					/**< @brief switch for changing weight matrix */
	Eigen::MatrixXd W;				/**< @brief weight matrix */

	double switch_f_cut; 			/**< @brief cutoff frequency for switches */
	double switch_thresh_up; 			/**< @brief cutoff frequency for switches */
	double switch_thresh_down; 			/**< @brief cutoff frequency for switches */

	Eigen::VectorXd F_refvec;		/**< @brief target force exerted to environment by robot */

	/**
	 * @brief 	reference frame for F_refvec, 0: end-effector, 1: base frame
	 * @remark 	0(E-E): when tool z-axis looking down, to press down, F_refvec[2] > 0 \n
	 * 			1(BASE): in any configuration, to press down, F_refvec[2] < 0 \n
	 * */
	double switch_control;

	double Kspring;					/**< @brief spring constant for force direction */
	double Kdamp;					/**< @brief damping constant for force direction */

	Eigen::VectorXd e_nr; 			/**< @brief error between q and q_n */
	Eigen::VectorXd edot_nr; 		/**< @brief error between qdot and qdot_n */
	Eigen::VectorXd eI_nr; 			/**< @brief error integral between q and q_n */

	Eigen::VectorXd qddot_ref, edot, e;
	Eigen::VectorXd qddot_n_1, qddot_n_2;
	Eigen::VectorXd pdot_2;
	Eigen::VectorXd qdot_n_prev;

	double e_f;		/**< @brief error between reference and measured force */
	double e_f_prev; 	/**< @brief previous value of e_force */
	double edot_f; 	/**< @brief error between reference and measured force */
	double eI_f;		/**< @brief error integral between reference and measured force */

	Eigen::VectorXd tauC;			/**< @brief preallocated matrix for control torque */
	Eigen::VectorXd tauPD;			/**< @brief preallocated matrix for outer-loop controller */
	Eigen::VectorXd forcePD;		/**< @brief preallocated matrix for outer-loop controller */
	Eigen::VectorXd tauAux;			/**< @brief preallocated matrix for auxiliary torque */

	Eigen::VectorXd F_b;			/**< @brief end-effector force represented in base coordinate*/

	LowPassFilter<Eigen::VectorXd> lpf_pos;		/**< @brief low-pass filtered position */
	LowPassFilter<Eigen::VectorXd> lpf_vel;		/**< @brief low-pass filtered velocity */
	LowPassFilter<Eigen::VectorXd> lpf_acc;		/**< @brief low-pass filtered acceleration */
    LowPassFilter<Eigen::VectorXd> lpf_ft;		/**< @brief low-pass filtered FT senser values */
    LowPassFilter<Eigen::VectorXd> lpf_tauExt;	/**< @brief low-pass filtered FT senser values */

    LowPassFilter<double> lpf_Imp;	/**< @brief low-pass filter for slow impedance switch */
    LowPassFilter<double> lpf_Fz;	/**< @brief low-pass filter for slow force switch */

    Eigen::MatrixXd J_1;			/**< @brief main task Jacobian (m by n) */
    Eigen::MatrixXd J_1_inv;		/**< @brief weighted pseudo-inverse (n by m) */
    Eigen::MatrixXd J_1_inv_prev;	/**< @brief previous weighted pseudo-inverse for derivative */
    Eigen::MatrixXd Jdot_1_inv;		/**< @brief derivative of weighted pseudo-inverse*/
    Eigen::MatrixXd J_bar;			/**< @brief null space projection */
	Eigen::VectorXd pdot_1;			/**< @brief velocity of priority 1 in task space */
	Eigen::VectorXd pddot_1;		/**< @brief acceleration of priority 1 in task space */

	Eigen::MatrixXd N_2;			/**< @brief null space of J_1 */

	Eigen::MatrixXd Z_2;			/**< @brief null space for second task */
	Eigen::MatrixXd Z_2_prev;		/**< @brief previous null space for second task for derivative */
	Eigen::MatrixXd Zdot_2;		/**< @brief derivative of null space for second task */
	Eigen::MatrixXd Rot;			/**< @brief rotation matrix from base to end-tool */

    Eigen::VectorXd tauMax;			/**< @brief preallocated matrix for auxiliary torque */

	/**
	 * @param 	JOINT_DOF	degree of freedom for joint space
	 * @param 	TASK_DOF	degree of freedom for task space
	 */
	ControlNRIC_Force(ControlMode mode, int JOINT_DOF, int TASK_DOF, double DT):
		ControllerInterface(mode, JOINT_DOF, TASK_DOF, DT, true, true, false, true),
		tauPD(JOINT_DOF), tauAux(JOINT_DOF),
		lpf_pos(DT, F_CUT_DEFAULT), lpf_vel(DT, F_CUT_DEFAULT), lpf_acc(DT, F_CUT_DEFAULT),
		lpf_ft(DT, F_CUT_DEFAULT), lpf_tauExt(DT, F_CUT_DEFAULT),
		JointIdentity(JOINT_DOF, JOINT_DOF),
		lpf_Fz(DT, F_SWITCH_DEFAULT), lpf_Imp(DT, F_SWITCH_DEFAULT)
	{
		printf("============================= ControlNRIC_Force =============================\n");
		printf("================= %dD Joints / %dD Task space / DT = %f ==============\n", JOINT_DOF, TASK_DOF, DT);


		JointIdentity.setIdentity();
        register_gain("tauMax", &tauMax, JOINT_DOF, 0);
		register_gain("f_cut", &f_cut, F_CUT_DEFAULT);
		register_gain("f_cut_force", &f_cut_force, F_CUT_DEFAULT);
		switch(mode){
		case ControlMode::JOINT_CONTROL:
			printf("====================== ControlMode: JOINT_CONTROL ========================\n");
			register_gain("K", &K, JOINT_DOF, 0);
			register_gain("Kp", &Kp, JOINT_DOF, 0);
			register_gain("KI", &KI, JOINT_DOF, 0);
			register_gain("Gamma", &gamma, 1);
			register_gain("switch_control", &switch_control, 0);
			register_gain("switch_frame", &switch_frame, 0);

			register_gain("kP", &kP, 0);
			register_gain("kI", &kI, 0);
			register_gain("kD", &kD, 0);

			register_gain("kp_diag", &kp_diag, JOINT_DOF, 0);
			register_gain("kv_diag", &kv_diag, JOINT_DOF, 0);

//			register_gain("kp_diag", &kp_diag, 0);
//			register_gain("kv_diag", &kv_diag, 0);
			register_gain("M", &M_J, JOINT_DOF, 1);
			register_gain("Kspring", &Kspring, 0);
			register_gain("Kdamp", &Kdamp, 0);

			register_gain("switch_thresh_up", &switch_thresh_up, 0.1);
			register_gain("switch_thresh_down", &switch_thresh_down, 0.01);
			register_gain("switch_f_cut", &switch_f_cut, F_SWITCH_DEFAULT);

			register_gain("Weight", &Weight, 0);

			register_gain("FText_scale", &FText_scale, 0);
			register_gain("F_ref", &F_refvec, TASK_DOF, 0);

			e_nr = Eigen::VectorXd(JOINT_DOF);
			edot_nr = Eigen::VectorXd(JOINT_DOF);
			eI_nr = Eigen::VectorXd(JOINT_DOF);

			lpf_Imp.reset();
			lpf_Fz.reset();
			lpf_Imp.update(0);
			lpf_Fz.update(0);
			eI_f = 0;
			e_f_prev = 0;
			break;
		case ControlMode::TASK_CONTROL:
			printf("======================= ControlMode: TASK_CONTROL ========================\n");
			e_nr = Eigen::VectorXd(JOINT_DOF);
			edot_nr = Eigen::VectorXd(JOINT_DOF);
			eI_nr = Eigen::VectorXd(JOINT_DOF);
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
