#include "JointControl.h"
#include "../indyutils.h"

#include <iostream>
#include <sstream>
#include <algorithm>
#include <string>

#include <chrono>

#define TLIM0 550
#define TLIM1 550
#define TLIM2 550
#define TLIM3 550
#define TLIM4 550
#define TLIM5 550

JointControl::JointControl()
: AbstractController(read_license(0).c_str(), read_license(1).c_str(), read_license(2).c_str())
, _robotNom(NULL)
, _time(0)
{
	// initialize shared memory
	// INDY_SHM_NAME: "indySHM"
	// INDY_SHM_LEN: 0x1000000 16MB == 0x000000~0xFFFFFF (~16777216)
	indyShm = new NRMKFramework::ShmemManager(INDY_SHM_NAME, INDY_SHM_LEN);
	for (int i=0; i<6; i++) _ftBias[i] = 0;

	// initialize variables
	_J.setZero();
	_Jdot.setZero();

	torque_limit << TLIM0, TLIM1, TLIM2, TLIM3, TLIM4, TLIM5;
}

JointControl::~JointControl()
{
	// destroy nominal robot
	if (_robotNom != NULL) delete _robotNom;

	// destroy shared memory
	if (indyShm != NULL)
		delete indyShm;
}

#ifdef __X86_64__
void JointControl::initialize(double delt) { // STEP3 SDK interface does not get robot as argument!
#else
void JointControl::initialize(ROBOT & robot, double delt) {
#endif
	_robotNom = AbstractController::createRobotObj();

	_delT = delt;

	//initialize ControlHub
	control_hub_p = std::make_shared<RNB::ControlHub>(
			RNB::ControllerInterface::JOINT_CONTROL, JOINT_DOF, 6, _delT);	// initialize controller hub
}

void JointControl::setPassivityMode(bool enable)
{
}

void JointControl::reset()
{
    // get ft sensor values from shared memory
     NRMKIndy::SharedData::getControlData(*indyShm, ctrData);

     for (int i=0; i<6; i++) _ftSensor[i] = ctrData.Fext[i];

     // compensate ft sensor bias
     setBias();

    init_robotNom = true;
}

void JointControl::reset(int jIndex)
{
	// get ft sensor values from shared memory
	 NRMKIndy::SharedData::getControlData(*indyShm, ctrData);

	 for (int i=0; i<6; i++) _ftSensor[i] = ctrData.Fext[i];

	 // compensate ft sensor bias
	 setBias();

	 init_robotNom = true;
}

void JointControl::setGains(JointVec const & kp, JointVec const & kv, JointVec const & ki)
{
}


int JointControl::computeControlTorq(ROBOT & robot, LieGroup::Vector3D const & gravDir,
		JointVec const & qDesired, JointVec const & qdotDesired, JointVec const & qddotDesired, JointVec & torque)
{
	// wait until servo is On
	NRMKIndy::SharedData::getControlStatusData(*indyShm, ctrStatusData);
	if(!ctrStatusData.isReady)
	{
		robot.idyn_gravity(gravDir);
		torque = robot.tau();

		return 0;
	}

	// update initial states of nominal robot with those of real robot
	if (init_robotNom == true)
	{
		// align ft sensor coordinates with robot's tool frame's coordinates
		robot.Tft() = LieGroup::HTransform(
				LieGroup::Rotation(1, 0, 0, 0, 1, 0, 0, 0, 1),
				LieGroup::Displacement::Zero());
		init_robotNom = false;
		control_hub_p->reset_controller(robot.q().data()); // this will trigger check_nominal_reset() on
	}
	if (control_hub_p->check_nominal_reset())
    {
        //reset nominal system "robotNom"; the function is defined below
        initNominalSystem(robot);
    }

	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

	NRMKIndy::SharedData::getControlData(*indyShm, ctrData);
	JointVec tauAct;
	std::memcpy(tauAct.data(), ctrData.tauact, JOINT_DOF*sizeof(double));

	// read ft sensor value; the function is defined below
	readFTSensor(robot);

	// set external torque zero, as there is no torque sensor
	tauExt.setZero();
//
//	//calculate real and nominal robot dynamics
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
	control_hub_p->calculate_joint_control_torque(
			_q.data(), _qdot.data(), _qddot.data(),
			qDesired.data(), qdotDesired.data(), qddotDesired.data(),
			_robotNom->q().data(), _robotNom->qdot().data(),
			_p.data(), _p_n.data(),
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

	// Don't erase below line! This seems stablize time sequence of control loop - without this, position error due to timeout frequently occurs
	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
//	std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[us]" << std::endl;

	return 0;
}

void JointControl::initNominalSystem(ROBOT & robot) {
	_robotNom->setTtarget(robot.Ttarget().R(), robot.Ttarget().r());
	_robotNom->setTReference(robot.Tref().R(), robot.Tref().r());
	_robotNom->q() = robot.q();
	_robotNom->qdot() = robot.qdot();
	_robotNom->qdot().setZero();
	_robotNom->update();
	_robotNom->updateJacobian();
}


void JointControl::readFTSensor(ROBOT & robot)
{
    NRMKIndy::SharedData::getControlData(*indyShm, ctrData);
    LieGroup::Wrench _ftSensorTemp;
    _ftSensorTemp.setZero();

    for (int i=0; i<6; i++) _ftSensorTemp[i] = ctrData.Fext[i];
    _ftSensorTemp = _ftSensorTemp - _ftBias;

    LieGroup::HTransform T_ft_target = robot.Ttarget().icascade(robot.Tft());
    LieGroup::Wrench _ftSensorTransformed = T_ft_target.itransform(_ftSensorTemp);

    for (int i=0; i<3; i++) _ftSensor[i] = _ftSensorTransformed[i+3];
    for (int i=0; i<3; i++) _ftSensor[i+3] = _ftSensorTransformed[i];
}

int JointControl::computeGravityTorq(ROBOT & robot, LieGroup::Vector3D const & gravDir, JointVec & tauGrav)
{
    init_robotNom = true;
	//update robot position to trajectory interface
	control_hub_p->trajectory_interface.update_position(robot.q().data());

    // compute gravitational torque
    robot.idyn_gravity(LieGroup::Vector3D(0, 0, -GRAV_ACC));
    tauGrav = robot.tau();
	return 0;
}



typedef JointControl JointControlRNB;

POCO_BEGIN_MANIFEST(AbstractJointController<AbstractRobot6D>)
	POCO_EXPORT_CLASS(JointControlRNB)
POCO_END_MANIFEST
