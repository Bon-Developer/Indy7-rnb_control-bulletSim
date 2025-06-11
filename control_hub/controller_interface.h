//
// Created by rnb on 21. 3. 2..
//

#ifndef WEBTEST_CONTROLLER_INTERFACE_H
#define WEBTEST_CONTROLLER_INTERFACE_H

#include <map>
#include <string>
#include <memory>
#include <iostream>

#include "algebra.h"

#define BOLD_RED_TMP          "\x1b[31m"
#define ANSI_RESET_TMP        "\x1b[0m"

#ifdef _WIN32
#define EXPORT extern "C" __declspec(dllexport) 
#else
#define EXPORT extern "C"
#endif

namespace RNB {

/**
 * @class ControllerInterface
 * @brief interface class for controller
 * @remark You must override the constructor and 4 other methods to implement your own controller
 */
extern "C" class ControllerInterface{
public:
	/** ContorlMode	 */
	enum ControlMode{
		JOINT_CONTROL=0,	/**< @brief joint control mode */
		TASK_CONTROL=1		/**< @brief task control mode */
	};

	std::map<std::string, std::vector<double>> gain_map; 	/**< @brief map for name and value of each gain */

	ControlMode mode;
	int JOINT_DOF; /**< @brief degree of freedom for joint space */
	int TASK_DOF; /**< @brief degree of freedom for task space */
	double DT; /**< @brief reciprocal of control frequency */
	double _t; /**< @brief time */
	/// boolean flags for kinematics and dynamics usage. These flags are to optimize compuation efficiency
	bool use_real_kinematics=false; 		/**< @brief flag to indicate if the algorithm uses real kinematics - you should initialize in constructor */
	bool use_nominal_kinematics=false;		/**< @brief flag to indicate if the algorithm uses nominal kinematics - you should initialize in constructor */
	bool use_real_dynamics=false;			/**< @brief flag to indicate if the algorithm uses real dynamics - you should initialize in constructor */
	bool use_nominal_dynamics=false;		/**< @brief flag to indicate if the algorithm uses nominal dynamics - you should initialize in constructor */

    AlgebraPtr alg_p=nullptr;                         /**< @brief shraed pointer for Task space algebra */

	/**
	 * @brief	you must override constructor and  initialize gain map in it
	 * @param 	JOINT_DOF	degree of freedom for joint space
	 * @param 	TASK_DOF	degree of freedom for task space
	 * @param 	DT			contol time step
	 * @param 	use_real_kinematics		flag to indicate if the algorithm uses real kinematics - you should initialize in constructor
	 * @param 	use_nominal_kinematics	flag to indicate if the algorithm uses nominal kinematics - you should initialize in constructor
	 * @param 	use_real_dynamics		flag to indicate if the algorithm uses real dynamics - you should initialize in constructor
	 * @param 	use_nominal_dynamics	flag to indicate if the algorithm uses nominal dynamics - you should initialize in constructor
	 */
    ControllerInterface(ControlMode mode, int JOINT_DOF, int TASK_DOF, double DT,
    		bool use_real_kinematics=false, bool use_nominal_kinematics=false,
			bool use_real_dynamics=false, bool use_nominal_dynamics=false):
    	mode(mode), JOINT_DOF(JOINT_DOF), TASK_DOF(TASK_DOF), DT(DT), _t(0),
		use_real_kinematics(use_real_kinematics), use_nominal_kinematics(use_nominal_kinematics),
		use_real_dynamics(use_real_dynamics), use_nominal_dynamics(use_nominal_dynamics){
    	if (!(use_real_kinematics || use_nominal_kinematics || use_real_dynamics || use_nominal_dynamics)){
    		std::cout<< BOLD_RED_TMP "=======================================================================================" ANSI_RESET_TMP <<std::endl;
    		std::cout<< BOLD_RED_TMP "===== [WARNING] You should set following boolean flags in controller constructor ======" ANSI_RESET_TMP <<std::endl;
    		std::cout<< BOLD_RED_TMP "use_real_kinematics, use_nominal_kinematics, use_real_dynamics, use_nominal_dynamics = ?" ANSI_RESET_TMP <<std::endl;
    		std::cout<< BOLD_RED_TMP "=======================================================================================" ANSI_RESET_TMP <<std::endl;
    	}
        alg_p.reset();
	}

	~ControllerInterface() {
		std::cout << "====================== Controller Destroyed ======================"<< std::endl;
	}

	/**
	 * @brief set algebra for task control
	 */
	 void set_task_space(AlgebraPtr alg_p){
        this->alg_p.reset();
        this->alg_p = alg_p;
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
	 * @param	J			Jacobian matrix (MatrixXd(TASK_DOF,JOINT_DOF)), in xyzuvw row order. Rotation order is Rot_z(w)*Rot_y(v)*Rot_x(u).
	 * @param	Jdot		Jacobian dot matrix (MatrixXd(TASK_DOF,JOINT_DOF))
	 * @param	J_n			nominal Jacobian matrix (MatrixXd(TASK_DOF,JOINT_DOF))
	 * @param	Jdot_n		nominal Jacobian dot matrix (MatrixXd(TASK_DOF,JOINT_DOF))
	 * @param	FText		FT sensor input - aligned with and represented in last joint coordinate (VectorXd(TASK_DOF))
	 * @param	qddot_n		nominal joint acceleration output (VectorXd(JOINT_DOF))
	 * @param	torque_out	control torque output (VectorXd(JOINT_DOF))
	 */
    virtual int calculate_joint_control_torque(
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
			Eigen::VectorXd& custom_dat) = 0;

	/**
	 * @brief	(you must override) calculate task control torque and nominal qddot
	 * @param	p			current task value (VectorXd(Task_DOF)), in xyzuvw row order. Rotation order is Rot_z(w)*Rot_y(v)*Rot_x(u).
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
	 * @param	J			Jacobian matrix (MatrixXd(TASK_DOF,JOINT_DOF)), in xyzuvw row order. Rotation order is Rot_z(w)*Rot_y(v)*Rot_x(u).
	 * @param	Jdot		Jacobian dot matrix (MatrixXd(TASK_DOF,JOINT_DOF))
	 * @param	J_n			nominal Jacobian matrix (MatrixXd(TASK_DOF,JOINT_DOF))
	 * @param	Jdot_n		nominal Jacobian dot matrix (MatrixXd(TASK_DOF,JOINT_DOF))
	 * @param	FText		FT sensor input - aligned with and represented in last joint coordinate (VectorXd(TASK_DOF))
	 * @param	qddot_n		nominal joint acceleration output (VectorXd(JOINT_DOF))
	 * @param	torque_out	control torque output (VectorXd(JOINT_DOF))
	 */
    virtual int calculate_task_control_torque(
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
			Eigen::VectorXd& custom_dat) = 0;

	/**
	 * @brief	(you must override) reset controller
	 */
    virtual void reset_control(Eigen::VectorXd &q0) = 0;

    /**
     * @remark (you must override) apply the gain values
     */
    virtual void apply_gain_values() = 0;

private:
	std::map<std::string, double*> double_map;				/**< @brief map for double variable assignment */
	std::map<std::string, Eigen::VectorXd*> vector_map;		/**< @brief map for Vector variable assignment */
	std::map<std::string, Eigen::MatrixXd*> matrix_map;		/**< @brief map for Matrix variable assignment */

	/**
	 * @brief	register new value on the gain map
	 */
    void register_gain_map(std::string key, std::vector<double> &gain_var){
		gain_map.insert(std::make_pair(key, gain_var));
    }

protected:
	/**
	 * @brief	register new double value on the gain map and initialize it
	 * @param 	key			name of the gain
	 * @param 	variable_p	pointer to the member variable that will receive the gain value
	 * @param 	initial_value	initial value
	 */
    void register_gain(std::string key, double* variable_p, double initial_value){
    	std::vector<double> gain_var(1, initial_value);
    	register_gain_map(key, gain_var);
    	double_map.insert(std::make_pair(key, variable_p));
    	(*variable_p) = initial_value;
    }

	/**
	 * @brief	register new Vector value on the gain map and initialize it
	 * @param 	key			name of the gain
	 * @param 	variable_p	pointer to the member variable that will receive the gain value
	 * @param 	size		number of gain components
	 * @param 	initial_value	initial value
	 */
    void register_gain(std::string key, Eigen::VectorXd* variable_p, int size, double initial_value){
    	std::vector<double> gain_var(size, initial_value);
    	register_gain_map(key, gain_var);
    	vector_map.insert(std::make_pair(key, variable_p));
    	variable_p->setOnes(size);
    	(*variable_p) *= initial_value;
    }


	/**
	 * @brief	register new diagonal Matrix value on the gain map and initialize it
	 * @param 	key			name of the gain
	 * @param 	variable_p	pointer to the member variable that will receive the gain value
	 * @param 	size		number of gain components
	 * @param 	initial_value	initial value
	 */
    void register_gain(std::string key, Eigen::MatrixXd* variable_p, int size, double initial_value){
    	std::vector<double> gain_var(size, initial_value);
    	register_gain_map(key, gain_var);
    	matrix_map.insert(std::make_pair(key, variable_p));
    	variable_p->setIdentity(size, size);
    	(*variable_p) *= initial_value;
    }

public:

    /**
     * @remark apply the gain values
     */
    void update_gain_values() {
    	std::cout<<"============== apply_gain_values (double) ==============="<<std::endl;
    	for(auto itor=double_map.begin(); itor!=double_map.end(); itor++){
            if((*itor->second) != gain_map[itor->first][0]){
                std::cout<<itor->first<<":"<<std::to_string(*itor->second)<<std::endl;
            }
    		(*itor->second) = gain_map[itor->first][0];
    	}

    	std::cout<<"============== apply_gain_values (Vector) ==============="<<std::endl;
    	for(auto itor=vector_map.begin(); itor!=vector_map.end(); itor++){
    		bool changed = false;
    		for(int i=0; i<gain_map[itor->first].size(); i++){
    		    changed = changed || ((*itor->second)[i] != gain_map[itor->first][i]);
        		(*itor->second)[i]= gain_map[itor->first][i];
    		}
    		if(changed){
                std::cout<<itor->first<<":"<<std::endl<<(*itor->second).transpose()<<std::endl;
    		}
    	}

    	std::cout<<"============== apply_gain_values (Matrix) ==============="<<std::endl;
    	for(auto itor=matrix_map.begin(); itor!=matrix_map.end(); itor++){
            bool changed = false;
    		for(int i=0; i<gain_map[itor->first].size(); i++){
                changed = changed || ((*itor->second)(i,i) != gain_map[itor->first][i]);
        		(*itor->second)(i,i) = gain_map[itor->first][i];
    		}
			Eigen::VectorXd diag = (*itor->second).diagonal();
			(*itor->second) = diag.asDiagonal();
            if(changed){
        	    std::cout<<itor->first<<":"<<std::endl<<(*itor->second)<<std::endl;
            }
    	}

    	std::cout<<"============== apply_gain_values done ==============="<<std::endl;
    }

};

extern "C" typedef std::shared_ptr<ControllerInterface> ControllerInterfacePtr;
extern "C" typedef void(*ControlChangeFun)(
	ControllerInterface::ControlMode, int, int, double, ControllerInterfacePtr&);

#define EXPORT_CONTROLLER(CONTROLLER)									\
EXPORT void create_controller(											\
		RNB::ControllerInterface::ControlMode mode,						\
		int JOINT_DOF, int TASK_DOF, double DT,							\
		RNB::ControllerInterfacePtr& cip){								\
	cip.reset();														\
	cip = std::make_shared<CONTROLLER>(mode, JOINT_DOF, TASK_DOF, DT);	\
}

/**
 * @class	LowPassFilter
 * @brief	low pass filter
 */
template <typename T>
class LowPassFilter{
public:
	T pre_input;
	T pre_output;
	double dT;
	double cutoff_freq;
	bool reset_flag;

	LowPassFilter(double dT, double cutoff_freq) :
		dT(dT), cutoff_freq(cutoff_freq), reset_flag(true) {}

	void reset(){
		reset_flag=true;
	}

	T& update(const T& input){
		if (reset_flag) {
			reset_flag = false;
			pre_input = input;
			pre_output = input;
		}
		pre_output = (1-cutoff_freq*dT/2) / (1+cutoff_freq*dT/2) * pre_output + (cutoff_freq*dT/2) / (1+cutoff_freq*dT/2)*(input+pre_input);
		pre_input = input;
		return pre_output;
	}

};

inline Eigen::Matrix3d eulerZYX(double w, double v, double u){
	Eigen::AngleAxisd Rz(w, Eigen::Vector3d::UnitZ());
	Eigen::AngleAxisd Ry(v, Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd Rx(u, Eigen::Vector3d::UnitX());

	Eigen::Quaternion<double> q = Rz * Ry * Rx;

	return q.matrix();
}

}

#endif //WEBTEST_CONTROLLER_INTERFACE_H
