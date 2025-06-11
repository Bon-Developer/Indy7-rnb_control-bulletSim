/**
 * @file 	control_algorithm.h
 * @brief	control algorithm interface
 *  Created on: 2021. 1. 29.
 * @author	RNB_CAD
 */

#ifndef CONTROL_ALGORITHM_DEFAULT_H_
#define CONTROL_ALGORITHM_DEFAULT_H_

#include "controller_interface.h"
#include <iostream>

#define DEFAULT_CONTROLLER_NAME "GravComp"


namespace RNB {

/**
 * @class ControlAlgorithmDefault
 * @brief default controller to be used when no dynamic library is loaded
 */
class ControlAlgorithmDefault : public ControllerInterface{
public:
	double Kg; /**< @brief Gravity gain */

	/**
	 * @param 	JOINT_DOF	degree of freedom for joint space
	 * @param 	TASK_DOF	degree of freedom for task space
	 */
	ControlAlgorithmDefault(ControlMode mode, int JOINT_DOF, int TASK_DOF, double DT):
		ControllerInterface(mode, JOINT_DOF, TASK_DOF, DT, false, false, false, false)
	{
		printf("=========================== Default Gravity Compensator ===========================\n");
		printf("================= %dD Joints / %dD Task space / DT = %f ==============\n", JOINT_DOF, TASK_DOF, DT);
		printf("==========================================================================\n");
		register_gain("Kg", &Kg, 1);
	}

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
	 * @param	p_n			nominal task value (VectorXd(Task_DOF)), in xyzuvw row order. Rotation order is Rot_z(w)*Rot_y(v)*Rot_x(u).
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

    /**
     * @remark apply the gain values
     */
    void apply_gain_values() override {}
};

}

#endif /* CONTROL_ALGORITHM_DEFAULT_H_ */
