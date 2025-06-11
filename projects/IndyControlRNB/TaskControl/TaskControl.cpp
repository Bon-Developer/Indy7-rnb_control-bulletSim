/*
 * TaskControl.cpp
 *
 *  Created on: 2021. 3. 6.
 *      Author: RNB_CAD
 */

#include "TaskControl.h"
#include "../indyutils.h"
#include <algorithm>

#define TLIM0 550
#define TLIM1 550
#define TLIM2 550
#define TLIM3 550
#define TLIM4 550
#define TLIM5 550

TaskControl::TaskControl() :
		AbstractController(read_license(0).c_str(), read_license(1).c_str(),
				read_license(2).c_str()), _robotNom(NULL), _time(0) {
	indyShm = new NRMKFramework::ShmemManager(INDY_SHM_NAME, INDY_SHM_LEN);
	for (int i = 0; i < 6; i++)
		_ftBias[i] = 0;

	torque_limit << TLIM0, TLIM1, TLIM2, TLIM3, TLIM4, TLIM5;
}

TaskControl::~TaskControl() {
	if (_robotNom != NULL)
		delete _robotNom;

	if (indyShm != NULL)
		delete indyShm;
}

#ifdef __X86_64__
void TaskControl::initialize(double delt) { // STEP3 SDK interface does not get robot as argument!
#else
void TaskControl::initialize(ROBOT & robot, double delt) {
#endif
	_robotNom = AbstractController::createRobotObj();

	// update sampling time
	_delT = delt;

	//initialize ControlHub
	control_hub_p = std::make_shared<RNB::ControlHub>(
			RNB::ControllerInterface::TASK_CONTROL, JOINT_DOF, 6, _delT);// initialize controller hub
}

void TaskControl::setPassivityMode(bool enable) {
}

void TaskControl::setGains(TaskVec const & kp, TaskVec const & kv,
		TaskVec const & ki) {
}

void TaskControl::setImpedanceParams(TaskVec const & iMass,
		TaskVec const & iDamping, TaskVec const & iStiffness,
		TaskVec const & iKi) {
}

void TaskControl::setImpedanceGains(TaskVec const & iKp, TaskVec const & iKv,
		TaskVec const & iKi, TaskVec const & iKf) {
}

void TaskControl::reset() {

	NRMKIndy::SharedData::getControlData(*indyShm, ctrData);

	for (int i = 0; i < 6; i++)
		_ftSensor[i] = ctrData.Fext[i];

	 // compensate ft sensor bias
	 setBias();

	// reset flag defined for controller reset
	_reset_ctrl = true;
}

int TaskControl::computeControlTorq(ROBOT & robot,
		LieGroup::Vector3D const & gravDir, TaskPosition const & posDesired,
		TaskVelocity const & velDesired, TaskAcceleration const & accDesired,
		JointVec & torque) {
	if (indyShm == NULL) {
		printf("Shared memory is missed\n");
	}

	// reset admittance trajectory and position controller
	if (_reset_ctrl) {
		_reset_ctrl = false;

		// align ft sensor coordinates with robot's tool frame's coordinates
		robot.Tft() = LieGroup::HTransform(
				LieGroup::Rotation(1, 0, 0, 0, 1, 0, 0, 0, 1),
				LieGroup::Displacement::Zero());
		control_hub_p->reset_controller(robot.q().data()); // this will trigger check_nominal_reset() on
	}
    if (control_hub_p->check_nominal_reset())
    {
        //reset nominal system "robotNom"; the function is defined below
        initNominalSystem(robot);
    }

	// read ft sensor value; the function is defined below
	readFTSensor(robot);

	//calculate real and nominal robot dynamics
	//DynamicAnalysis::JointDynamics(*_robotNom, M, C, g,
	//                          LieGroup::Vector3D(0, 0, -GRAV_ACC));
	if(control_hub_p->use_real_dynamics){
#ifdef __X86_64__
		DynamicAnalysis::JointDynamics(robot, M, C, g, gravDir);
#else
		robot.computeDynamicsParams(gravDir, M, C, g);
#endif
	}
	if(control_hub_p->use_nominal_dynamics){
#ifdef __X86_64__
		DynamicAnalysis::JointDynamics(*_robotNom, M_n, C_n, g_n, gravDir);
#else
		_robotNom->computeDynamicsParams(gravDir, M_n, C_n, g_n);
#endif
	}

	Vector6d _p, _p_n;
	if(control_hub_p->use_real_kinematics){
#ifdef __X86_64__
		_J << robot.J(robot.NUM_BODIES-1);
		_tpos = TaskPosition(robot.T(robot.NUM_BODIES-1));
		_tvel = TaskVelocity(robot.V(robot.NUM_BODIES-1));
		_tpos = TaskPosition(
				LieGroup::HTransform(_tpos.R(),_tpos.r())*robot.Ttarget());
#else
		robot.computeFK(_tpos, _tvel);
		robot.computeJacobian(_tpos, _tvel, _J, _Jdot);
		// Convert jacobian to xyzuvw row order
		_J = flip_pr<TaskJacobian>(_J);
		_Jdot = flip_pr<TaskJacobian>(_Jdot);
#endif
		_p = flip_pr<Vector6d>(_tpos.asVector());
	    Eigen::Matrix3d R = RNB::Rot_zyx(_p[5], _p[4], _p[3]);
	    _J.block(3, 0, 3, JOINT_DOF) << R.transpose() * _J.block(3, 0, 3, JOINT_DOF); // rotation jacobian is on end-effector. transform to base coordinate
	    _Jdot.block(3, 0, 3, JOINT_DOF) << R.transpose() * _Jdot.block(3, 0, 3, JOINT_DOF); // transform to base coordinate
	}

	if(control_hub_p->use_nominal_kinematics){
#ifdef __X86_64__
		_J_nom << _robotNom->J(_robotNom->NUM_BODIES-1);
		_tpos_nom = TaskPosition(_robotNom->T(_robotNom->NUM_BODIES-1));
		_tvel_nom  = TaskVelocity(_robotNom->V(_robotNom->NUM_BODIES-1));
		_tpos_nom = TaskPosition(
				LieGroup::HTransform(_tpos_nom.R(),_tpos_nom.r())*_robotNom->Ttarget());
#else
		_robotNom->computeFK(_tpos_nom, _tvel_nom);
		_robotNom->computeJacobian(_tpos_nom, _tvel_nom, _J_nom, _Jdot_nom);
		_J_nom = flip_pr<TaskJacobian>(_J_nom);
		_Jdot_nom = flip_pr<TaskJacobian>(_Jdot_nom);
#endif
		_p_n = flip_pr<Vector6d>(_tpos_nom.asVector());
	    Eigen::Matrix3d R_n = RNB::Rot_zyx(_p_n[5], _p_n[4], _p_n[3]);
	    _J_nom.block(3, 0, 3, JOINT_DOF) << R_n.transpose() * _J_nom.block(3, 0, 3, JOINT_DOF); // transform to base coordinate
	    _Jdot_nom.block(3, 0, 3, JOINT_DOF) << R_n.transpose() * _Jdot_nom.block(3, 0, 3, JOINT_DOF); // transform to base coordinate
	}

	robot.idyn_gravity(LieGroup::Vector3D(0, 0, -GRAV_ACC));
	tauGrav = robot.tau();

	//	_robotNom->idyn_gravity(LieGroup::Vector3D(0, 0, -GRAV_ACC));
	//	tauGrav_n = _robotNom->tau();

	_q = robot.q();
	_qdot = robot.qdot();
	_qddot = robot.qddot();

	// Convert ftSensor to xyzuvw row order
	_ftSensor = flip_pr<TaskVec>(_ftSensor);

	tauExt.setZero(JOINT_DOF, 1);// set external torque zero
	control_hub_p->calculate_task_control_torque(
			_p.data(), _p_n.data(),
			_p_d.data(), _pdot_d.data(), _pddot_d.data(),
			_q.data(), _qdot.data(), _qddot.data(),
			_robotNom->q().data(), _robotNom->qdot().data(),
			M.data(), C.data(), M_n.data(), C_n.data(),
			tauGrav.data(), tauExt.data(),
			_J.data(), 	_Jdot.data(), _J_nom.data(), _Jdot_nom.data(), _ftSensor.data(),
			qddot_n_out, torque_out);

	for (int i = 0; i < JOINT_DOF; i++)
	{
		_qddot_nom[i] = qddot_n_out[i];
//		torque[i] = std::max(-torque_limit[i], std::min(torque_limit[i], torque_out[i]));
		torque[i] = torque_out[i];
	}

	// update nominal states
	_time += _delT;
	_robotNom->q() += ((_delT*_robotNom->qdot()) + (0.5*_delT*_delT*_qddot_nom));
	_robotNom->qdot() += (_delT*_qddot_nom);
	_robotNom->update();
	_robotNom->updateJacobian();

	return 0;   //return 1: Close to Singularity
}

void TaskControl::initNominalSystem(ROBOT & robot) {
	_robotNom->setTtarget(robot.Ttarget().R(), robot.Ttarget().r());
	_robotNom->setTReference(robot.Tref().R(), robot.Tref().r());
	_robotNom->q() = robot.q();
	_robotNom->qdot() = robot.qdot();
	_robotNom->qdot().setZero();
	_robotNom->update();
	_robotNom->updateJacobian();
}

void TaskControl::readFTSensor(ROBOT & robot) {

	static unsigned int print_cnt = 0;

	if (++print_cnt == 4000) {
		print_cnt = 0;

		std::cout << "read FT sensor value" << std::endl;
		std::cout << _ftBias << std::endl;
	}

	NRMKIndy::SharedData::getControlData(*indyShm, ctrData);
	TaskPosition _tposT;
	TaskVelocity _tvelT;
	LieGroup::Wrench _ftSensorTemp;
	_ftSensorTemp.setZero();

	for (int i = 0; i < 6; i++)
		_ftSensorTemp[i] = ctrData.Fext[i];
	_ftSensorTemp = _ftSensorTemp - _ftBias;

    LieGroup::HTransform T_ft_target = robot.Ttarget().icascade(robot.Tft());
    LieGroup::Wrench _ftSensorTransformed = T_ft_target.itransform(_ftSensorTemp);

    for (int i=0; i<3; i++) _ftSensor[i] = _ftSensorTransformed[i+3];
    for (int i=0; i<3; i++) _ftSensor[i+3] = _ftSensorTransformed[i];
}

typedef TaskControl TaskControlRNB;

POCO_BEGIN_MANIFEST(AbstractTaskController<AbstractRobot6D>)
		POCO_EXPORT_CLASS(TaskControlRNB)
POCO_END_MANIFEST
