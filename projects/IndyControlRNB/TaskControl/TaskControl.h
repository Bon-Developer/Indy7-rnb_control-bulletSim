#ifndef TASKCONTROL_H_
#define TASKCONTROL_H_

#include <NRMKFramework/Components/AbstractTaskController.h>
#include <NRMKFramework/Components/AbstractRobot6D.h>

#include <Indy/DynamicAnalysis6D.h>
#include "NRMKFramework/Indy/Sharedmemory/SharedData.h"
NRMKFramework::ShmemManager * indyShm = NULL;
NRMKIndy::SharedData::RobotControlSharedData ctrData;

// for the use of lowpass filter and filtered derviative
#include <Util/Filter.h>
#include <math.h>

#include "control_hub.h"
#include "trajectory_interface.h"


class TaskControl : public AbstractTaskController<AbstractRobot6D>
{
	// degree of freedom (6dof)
	enum
	{
	  JOINT_DOF = AbstractRobot6D::JOINT_DOF
	};
	typedef AbstractTaskController<AbstractRobot6D> AbstractController;
	typedef Eigen::Matrix<double, 6, 1> TaskVec;
	typedef Eigen::Matrix<double, 6, 6> TaskJacobian;
	typedef NRMKFoundation::IndySDK::DynamicAnalysis6D DynamicAnalysis;

	// new type definitaions
	typedef Eigen::Matrix<double, 6, 1> Vector6d;
	typedef typename ROBOT::JointVec JointVec;
	typedef typename ROBOT::JointMat JointMat;
	typedef NRMKFoundation::FilterLowPass<double> LowPassFilter;
	typedef NRMKFoundation::FilterDerivative<double> FilteredDerivative;

	RNB::ControlHubPtr control_hub_p;
	double qddot_n_out[JOINT_DOF];
	double torque_out[JOINT_DOF];
	JointVec torque_limit;

public:
	TaskControl();
	virtual ~TaskControl();

#ifdef __X86_64__
	void initialize(double delt); // STEP3 SDK interface does not get robot as argument!
#else
	void initialize(ROBOT & robot, double delt);
#endif

	void setPassivityMode(bool enable);

	void setGains(TaskVec const & kp, TaskVec const & kv, TaskVec const & ki);
	void setImpedanceParams(TaskVec const & iMass, TaskVec const & iDamping, TaskVec const & iStiffness, TaskVec const & iKi = TaskVec::Zero());
	void setImpedanceGains(TaskVec const & iKp, TaskVec const & iKv, TaskVec const & iKi, TaskVec const & iKf = TaskVec::Zero());

	void reset();

	int computeControlTorq(ROBOT & robot, LieGroup::Vector3D const & gravDir, TaskPosition const & posDesired, TaskVelocity const & velDesired, TaskAcceleration const & accDesired, JointVec & torque);

	// member functions
	void readFTSensor(ROBOT & robot);
	void initNominalSystem(ROBOT & robot);
	void setBias() {_ftBias = _ftSensor;}


private:
	ROBOT * _robotNom;

	JointVec tauGrav, tauGrav_n, tauExt;

	Vector6d _p, _pdot;		/**< @brief task pose, velocity in xyzuvw order */
	Vector6d _p_d, _pdot_d, _pddot_d;		/**< @brief desired task pose, velocity, acceleration in xyzuvw order */
	JointVec _q, _qdot, _qddot;
	JointVec _qddot_nom;

private:
    // getting nominal robot dynamics
	JointMat M, C;
	JointMat M_n, C_n;
	JointVec g, g_n;

	TaskPosition _tpos; 	/**< @brief task pose */
	TaskVelocity _tvel; 	/**< @brief task velocity */
	TaskPosition _tpos_nom;
	TaskVelocity _tvel_nom;
	TaskJacobian _J, _Jdot, _J_nom, _Jdot_nom; /**< @brief Jacobian in NRMK framework, rows: uvwxyz */

private:
    // with respect to ft sensor
    LieGroup::Wrench    _ftBias;
    LieGroup::Wrench    _ftSensor; 	/**< @brief FT sensor value in order of force xyz moment xyz */

private:
    // reset control
    bool                _reset_ctrl;

private:
	double _delT;
	double _time;
};

#endif /* TASKCONTROL_H_ */
