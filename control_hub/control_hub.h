/*
 * control_hub.h
 *
 *  Created on: 2021. 3. 3.
 *      Author: RNB_CAD
 */

#ifndef CONTROL_HUB_H_
#define CONTROL_HUB_H_

#include "trajectory_interface.h"
#ifdef _WIN32
#include <process.h>
#else
#include <pthread.h>
#endif

#include "control_algorithm_default.h"
#include "controller_interface.h"
#include "controller_ui.h"
#include "data_logger.h"
#include "utils_rnb.h"

#define PORT_BASE 9990
#define CONTROLLER_PATH "./controllers/"


/**
 * @namespace 	RNB
 * @brief		namespace for RNB functionalities.
 * @remark 		implement all in-house classes and functions in here.
 */
namespace RNB {

/**
 * @class	ControlHub
 * @brief	Controller switching ui manager
 */
class ControlHub {
private:
    bool need_nominal_reset = false; 		/**< @brief flag for nominal reset requirement state */

public:

	ControllerInterface::ControlMode mode; /**< @brief joint/task control mode */

	int JOINT_DOF; 	/**< @brief degree of freedom for joint space */
	int TASK_DOF; 	/**< @brief degree of freedom for task space */
	double DT; 		/**< @brief reciprocal of control frequency */
	double _t; 		/**< @brief time */

	Eigen::VectorXd p; 			/**< @brief preallocated vector for task vector p, in xyzuvw order. Rotation order is Rot_z(w)*Rot_y(v)*Rot_x(u). */
	Eigen::VectorXd p_n; 		/**< @brief preallocated vector for nominal task vector p, in xyzuvw order. Rotation order is Rot_z(w)*Rot_y(v)*Rot_x(u). */

	Eigen::VectorXd p_d; 		/**< @brief preallocated vector for desire value of p */
	Eigen::VectorXd pdot_d; 	/**< @brief preallocated vector for desire value of pdot */
	Eigen::VectorXd pddot_d; 	/**< @brief preallocated vector for desire value of pddot */

	Eigen::VectorXd q; 			/**< @brief preallocated vector for joint vector q */
	Eigen::VectorXd qdot; 		/**< @brief preallocated vector for qdot */
	Eigen::VectorXd qddot; 		/**< @brief preallocated vector for qddot */

	Eigen::VectorXd q_d; 		/**< @brief preallocated vector for desire value of q */
	Eigen::VectorXd qdot_d; 	/**< @brief preallocated vector for desire value of qdot */
	Eigen::VectorXd qddot_d; 	/**< @brief preallocated vector for desire value of qddot */

	Eigen::VectorXd q_n; 		/**< @brief preallocated vector for nominal value of q */
	Eigen::VectorXd qdot_n; 	/**< @brief preallocated vector for nominal value of qdot */
	Eigen::VectorXd qddot_n; 	/**< @brief preallocated vector for nominal value of qddot */

	Eigen::MatrixXd M; 			/**< @brief preallocated matrix for inertia matrix (JOINT_DOF, JOINT_DOF) */
	Eigen::MatrixXd C; 			/**< @brief preallocated matrix for coriolis matrix (JOINT_DOF, JOINT_DOF) */

	Eigen::MatrixXd M_n;		/**< @brief preallocated matrix for nominal inertia matrix (JOINT_DOF, JOINT_DOF) */
	Eigen::MatrixXd C_n;		/**< @brief preallocated matrix for nominal coriolis matrix (JOINT_DOF, JOINT_DOF) */

	Eigen::VectorXd tauGrav; 	/**< @brief preallocated vector for gravity torques */
	Eigen::VectorXd tauExt; 	/**< @brief preallocated vector for external torques = real torque - model & gravity */

	Eigen::MatrixXd J;			/**< @brief preallocated matrix for jacobian, in xyzuvw row order. Rotation order is Rot_z(w)*Rot_y(v)*Rot_x(u). */
	Eigen::MatrixXd Jdot;		/**< @brief preallocated matrix for derivative of jacobian */
	Eigen::MatrixXd J_n;		/**< @brief preallocated matrix for nominal jacobian */
	Eigen::MatrixXd Jdot_n;		/**< @brief preallocated matrix for nominal derivative of jacobian */

	Eigen::VectorXd FText; 		/**< @brief preallocated vector for FT sensor inputn, in xyzuvw order */

	Eigen::VectorXd torque; 	/**< @brief preallocated vector for torque output */

    Eigen::VectorXd custom_dat;    /**< @brief preallocated vector for custom plot value (JOINT_DOF,1) */

	int updating_controller = 0; 		/**< @brief counter for updating actions to block access to controller when controller is changing */
	bool reset_controller_now = false; 		/**< @brief to notify the reset_controller function that position (q or p) is updated */
	ControllerInterfacePtr controller_p = nullptr; 	/**< @brief shared pointer to controller */
	DataLogger data_logger;
    RNB::TrajectoryInterface trajectory_interface;

#ifdef _WIN32
	HANDLE hThread; 		/**< @brief ui thread pointer */
	unsigned dwThreadID; 	/**< @brief ui thread ID */
#else
	pthread_t p_thread_ui = NULL; 	/**< @brief ui thread pointer */
#endif

	int thr_id__ui = NULL; 			/**< @brief thread id */
	bool stop_thread = false;		/**< @brief flag to request stop thread */
	bool thread_stopped = false;	/**< @brief flag to check thread status */

	bool use_real_kinematics=false; 		/**< @brief flag to indicate if the algorithm uses real kinematics - you should initialize in constructor */
	bool use_nominal_kinematics=false;		/**< @brief flag to indicate if the algorithm uses nominal kinematics - you should initialize in constructor */
	bool use_real_dynamics=false;			/**< @brief flag to indicate if the algorithm uses real dynamics - you should initialize in constructor */
	bool use_nominal_dynamics=false;		/**< @brief flag to indicate if the algorithm uses nominal dynamics - you should initialize in constructor */
    AlgebraPtr alg_p= nullptr;              /**< @brief shared_ptr to algebra */


	bool before_first_reset = true;			/**< @brief flag to indicate the algorithm has run at least one time */

	int ui_port = NULL;				/**< @brief web ui port */
	int traj_port = NULL;			/**< @brief trajectory interface port */


	/**
	 * @param 	JOINT_DOF	degree of freedom for joint space
	 * @param 	TASK_DOF	degree of freedom for task space
	 * @param 	DT			control period in seconds
	 * @param 	ui_port		port for web ui
	 * @param 	traj_port	port for trajectory interface
	 */
	ControlHub(ControllerInterface::ControlMode mode, int JOINT_DOF, int TASK_DOF, double DT, 
		int ui_port = NULL, int traj_port = NULL);

	~ControlHub();

	/**
	 * @brief   check if nominal reset is required. need_nominal_reset flag is automatically turned off.
	 * @return  true if nominal reset is required.
	 */
	bool check_nominal_reset(){
	    if(need_nominal_reset){
            need_nominal_reset = false;
            return true;
	    }
	    if(updating_controller){
            return true;
	    }
	    return false;
	}

    /**
     * @brief	define task space for trajectory interpolation in algebra
     */
    void set_task_space(AlgebraPtr alg_p){
        assert(mode==ControllerInterface::TASK_CONTROL);
        this->alg_p.reset();
        this->alg_p = alg_p;
        if (controller_p!=nullptr){
            controller_p->set_task_space(alg_p);
        }
    }

	/**
	 * @brief	reset controller - wait until q is updated on control loop and reset controller and trajectory interface.
	 * @param	reset_traj	flag to reset trajectory - from UI, trajectory should not be reset.
	 * @param	reset_nom	flag to reset nominal model - from UI, nominal model should be reset only when controller is changed.
	 */
	void reset_controller(bool reset_traj=true, bool reset_nom=true);

	/**
	 * @brief	reset controller and trajectory interface.
	 * @param	x0	double array to reset trajectoory
	 * @param	reset_traj	flag to reset trajectory - from UI, trajectory should not be reset.
	 * @param	reset_nom	flag to reset nominal model - from UI, nominal model should be reset only when controller is changed.
	 */
	void reset_controller(double* x0, bool reset_traj=true, bool reset_nom=true);

	/**
	 * @brief	calculate joint control torque
	 * @param	_q			current joint value (double[JOINT_DOF])
	 * @param	_qdot		current joint velocity (double[JOINT_DOF])
	 * @param	_qddot		current joint acceleration (double[JOINT_DOF])
	 * @param	_q_d		desired joint value (double[JOINT_DOF])
	 * @param	_qdot_d		desired joint velocity (double[JOINT_DOF])
	 * @param	_qddot_d	desired joint acceleration (double[JOINT_DOF])
	 * @param	_q_n		nominal joint value (double[JOINT_DOF])
	 * @param	_qdot_n		nominal joint velocity (double[JOINT_DOF])
	 * @param	_p			current task pose (double(TASK_DOF))
	 * @param	_p_n		nominal task pose (double(TASK_DOF))
	 * @param	_M			Inertia matrix (double[JOINT_DOFxJOINT_DOF])
	 * @param	_C			Coriolis matrix (double[JOINT_DOFxJOINT_DOF])
	 * @param	_M_n		nominal Inertia matrix (double[JOINT_DOFxJOINT_DOF])
	 * @param	_C_n		nominal Coriolis matrix (double[JOINT_DOFxJOINT_DOF])
	 * @param	__tauGrav	gravity torque (double[JOINT_DOF])
	 * @param	__tauExt	external torque = real torque - (dynamic model torque & gravity) (double[JOINT_DOF])
	 * @param	_J			Jacobian matrix, in xyzuvw row order (double[TASK_DOFxJOINT_DOF])
	 * @param	_Jdot		Jacobian dot matrix (double[TASK_DOFxJOINT_DOF])
	 * @param	_J_n		nominal Jacobian matrix (double[TASK_DOFxJOINT_DOF])
	 * @param	_Jdot_n	nominal Jacobian dot matrix (double[TASK_DOFxJOINT_DOF])
	 * @param	__FText		FT sensor input - aligned with and represented in last joint coordinate (double[TASK_DOF])
	 * @param	_qddot_n_out	nominal joint acceleration output (double[JOINT_DOF])
	 * @param	torque_out		control torque output (double[JOINT_DOF])
	 */
	int calculate_joint_control_torque(
			const double *_q, const double *_qdot, const double *_qddot,
			const double *_q_d, const double *_qdot_d, const double *_qddot_d,
			const double *_q_n, const double *_qdot_n,
			const double *_p, const double *_p_n,
			const double *_M, const double *_C,
			const double *_M_n, const double *_C_n,
			const double *_tauGrav, const double *_tauExt,
			const double *_J, const double *_Jdot,
			const double *_J_n, const double *_Jdot_n,
			const double *_FText,
			double *_qddot_n_out, double *torque_out);

	/**
	 * @brief	calculate task control torque
	 * @param	_p			current task value, in xyzuvw order. Rotation order is Rot_z(w)*Rot_y(v)*Rot_x(u). (double[TASK_DOF])
	 * @param	_p_n		nominal task value, in xyzuvw order. Rotation order is Rot_z(w)*Rot_y(v)*Rot_x(u). (double[TASK_DOF])
	 * @param	_p_d		desired task value (double[TASK_DOF])
	 * @param	_pdot_d		desired task velocity (double[TASK_DOF])
	 * @param	_pddot_d	desired task acceleration (double[TASK_DOF])
	 * @param	_q			current joint value (double[JOINT_DOF])
	 * @param	_qdot		current joint velocity (double[JOINT_DOF])
	 * @param	_qddot		current joint acceleration (double[JOINT_DOF])
	 * @param	_q_n		nominal joint value (double[JOINT_DOF])
	 * @param	_qdot_n		nominal joint velocity (double[JOINT_DOF])
	 * @param	_M			Inertia matrix (double[JOINT_DOFxJOINT_DOF])
	 * @param	_C			Coriolis matrix (double[JOINT_DOFxJOINT_DOF])
	 * @param	_M_n		nominal Inertia matrix (double[JOINT_DOFxJOINT_DOF])
	 * @param	_C_n		nominal Coriolis matrix (double[JOINT_DOFxJOINT_DOF])
	 * @param	__tauGrav	gravity torque (double[JOINT_DOF])
	 * @param	__tauExt	external torque = real torque - (dynamic model torque & gravity) (double[JOINT_DOF])
	 * @param	_J			Jacobian matrix, in xyzuvw row order. Rotation order is Rot_z(w)*Rot_y(v)*Rot_x(u). (double[TASK_DOFxJOINT_DOF])
	 * @param	_Jdot		Jacobian dot matrix (double[TASK_DOFxJOINT_DOF])
	 * @param	_J_n		nominal Jacobian matrix (double[TASK_DOFxJOINT_DOF])
	 * @param	_Jdot_n	nominal Jacobian dot matrix (double[TASK_DOFxJOINT_DOF])
	 * @param	__FText		FT sensor input - aligned with and represented in last joint coordinate. (double[TASK_DOF])
	 * @param	_qddot_n_out	nominal joint acceleration output (double[JOINT_DOF])
	 * @param	torque_out		control torque output (double[JOINT_DOF])
	 */
	int calculate_task_control_torque(
			const double *_p, const double *_p_n,
			const double *_p_d, const double *_pdot_d, const double *_pddot_d,
			const double *_q, const double *_qdot, const double *_qddot,
			const double *_q_n, const double *_qdot_n,
			const double *_M, const double *_C,
			const double *_M_n, const double *_C_n,
			const double *_tauGrav, const double *_tauExt,
			const double *_J, const double *_Jdot,
			const double *_J_n, const double *_Jdot_n,
			const double *_FText,
			double *_qddot_n_out, double *torque_out);
};

/**
 * @brief load controller with dropdown_controller.selected and load parameters to param_setting
 */
void * change_controller(ControlHub* control_hup_p,
		Dropdown &dropdown_controller, MultipleInputForm &param_setting, void *controller_lib);

/**
 * @brief apply gain values
 */
void apply_gains(ControlHub* control_hup_p, MultipleInputForm &param_setting, bool reset_nom);

typedef std::shared_ptr<ControlHub> ControlHubPtr;

/**
 * @brief control server thread function
 */
#ifdef _WIN32
unsigned __stdcall serve_controller_server(void* control_hub_vp);
#else
void* serve_controller_server(void* control_hub_vp);
#endif

}

#endif /* CONTROL_HUB_H_ */
