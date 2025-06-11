#ifndef JointImpedance_H_
#define JointImpedance_H_

#include <NRMKFramework/Components/AbstractJointController.h>
#include <NRMKFramework/Components/AbstractRobot6D.h>

#include <Indy/DynamicAnalysis6D.h>
#include <NRMKFramework/Indy/Sharedmemory/SharedData.h>
#include "control_hub.h"
#include "trajectory_interface.h"


class JointControl : public AbstractJointController<AbstractRobot6D>
{
	enum
	{
		JOINT_DOF = AbstractRobot6D::JOINT_DOF
	};
	typedef AbstractJointController<AbstractRobot6D> AbstractController;
	typedef AbstractRobot6D ROBOT;
	typedef typename ROBOT::JointVec JointVec;
	typedef NRMKFoundation::IndySDK::DynamicAnalysis6D DynamicAnalysis;

	typedef typename ROBOT::JointMat JointMat;
	typedef typename ROBOT::TaskPosition TaskPosition;
	typedef typename ROBOT::TaskVelocity TaskVelocity;
	typedef typename ROBOT::TaskJacobian TaskJacobian;

	NRMKFramework::ShmemManager * indyShm = NULL;
	NRMKIndy::SharedData::ExtraIOData extioData; // extra input-output data
	NRMKIndy::SharedData::RobotControlSharedData ctrData; // control data
	NRMKIndy::SharedData::RobotControlStatusSharedData ctrStatusData; // control status data
	RNB::ControlHubPtr control_hub_p;
	double qddot_n_out[JOINT_DOF];
	double torque_out[JOINT_DOF];
	JointVec torque_limit;

public:
	JointControl();
	~JointControl();

#ifdef __X86_64__
	void initialize(double delt); // STEP3 SDK interface does not get robot as argument!
#else
	void initialize(ROBOT & robot, double delt);
#endif

	void setPassivityMode(bool enable);
	void setGains(JointVec const & kp, JointVec const & kv, JointVec const & ki);

	void reset();
	void reset(int jIndex);

	int computeControlTorq(ROBOT & robot, LieGroup::Vector3D const & gravDir, JointVec const & qDesired, JointVec const & qdotDesired, JointVec const & qddotDesired, JointVec & torque);
	int computeGravityTorq(ROBOT & robot, LieGroup::Vector3D const & gravDir, JointVec & torque);

	int computePDcontroller(JointVec const & qdes, JointVec const & q,	JointVec const & qdot, JointVec const & tauGrav, JointVec & tauPD);

	void setBias() {_ftBias = _ftSensor;}
	void readFTSensor(ROBOT & robot);
	void initNominalSystem(ROBOT & robot);

	double _delT;
	int key;
	int flag_bias;

	typedef Eigen::Matrix<double, 6, 1> TaskVec;

	typedef NRMKFoundation::RotationController::TaskVec TaskVecRot;
	typedef NRMKFoundation::PositionController<3>::TaskVec TaskVecDisp;
	NRMKFoundation::RotationController _rot;
	NRMKFoundation::PositionController<3> _disp;


	void TaskError(TaskPosition const & cur_pos, TaskVelocity const & cur_vel,
			TaskPosition const & des_pos, TaskVelocity const & des_vel,
		TaskVec & e_pos, TaskVec & e_vel)
	{
		TaskVecRot e_pos_rot;
		TaskVecRot e_vel_rot;
		//_rot.error(cur_pos.rot, cur_vel.rot, des_pos.rot, des_vel.rot, e_pos_rot, e_vel_rot);

		LieGroup::Rotation Rtilde = cur_pos.R().icascade(des_pos.R());
		LieGroup::Vector3D xi_ = Rtilde.expCoord();
	    e_pos_rot = xi_;

		double alpha, beta, theta_sqr;
		xi_.compute_terms(&alpha, &beta, &theta_sqr);

	    Eigen::Matrix<double, 3, 3> dexpinv = xi_.dexp_inv(alpha, beta, theta_sqr);

		LieGroup::Vector3D wtilde = des_vel.w() - Rtilde.irotate(cur_vel.w());
		LieGroup::Vector3D xidot = dexpinv.transpose()*wtilde;
	    e_vel_rot = xidot;

		TaskVecDisp e_pos_disp;
		TaskVecDisp e_vel_disp;
		_disp.error(cur_pos.disp, cur_vel.disp, des_pos.disp, des_vel.disp, e_pos_disp, e_vel_disp);

		e_pos << e_pos_rot, e_pos_disp;
		e_vel << e_vel_rot, e_vel_disp;
	}


private:
	ROBOT * _robotNom;
	bool init_robotNom;

	JointVec tauGrav, tauGrav_n, tauExt;

	JointVec _q, _qdot, _qddot;
	JointVec _q_d, _qdot_d, _qddot_d;
	JointVec _qddot_nom;

	JointMat M, C;
	JointMat M_n, C_n;
	JointVec g, g_n;

	TaskPosition _tpos; 	/**< @brief task pose */
	TaskVelocity _tvel; 	/**< @brief task velocity */
	TaskPosition _tpos_nom;
	TaskVelocity _tvel_nom;
	TaskJacobian _J, _Jdot, _J_nom, _Jdot_nom; /**< @brief Jacobian in NRMK framework, rows: uvwxyz */
private:
    // with respect to FT sensor
    LieGroup::Wrench    _ftBias;
    LieGroup::Wrench    _ftSensor; 	/**< @brief FT sensor value in force xyz moment xyz order */

	double _time;
};

#endif /* JointImpedance_H_ */
