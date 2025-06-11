
#include "../../../../control_hub/control_hub.h"
#include "ControlHubPlugin.h"
#include "SharedMemory/SharedMemoryPublic.h"
#include "SharedMemory/plugins/b3PluginContext.h"
#include <stdio.h>

#include "LinearMath/btScalar.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "SharedMemory/b3RobotSimulatorClientAPI_NoDirect.h"
#include "SharedMemory/b3RobotSimulatorClientAPI_InternalData.h"

#include <map>

struct ControlHubContainer
{
	RNB::ControlHubPtr control_hub_p;
	int robot_id=-1;
	int nominal_id=-1;
	bool reset_cmd;

	Eigen::VectorXd q;
	Eigen::VectorXd qdot;
	Eigen::VectorXd qddot;

	Eigen::VectorXd q_ref;

	Eigen::VectorXd q_n;
	Eigen::VectorXd qdot_n;


	Eigen::VectorXd p;
	Eigen::VectorXd p_n;

    Eigen::VectorXd p_ref;

	Eigen::MatrixXd M;
	Eigen::MatrixXd M_n;

	Eigen::MatrixXd C;
	Eigen::MatrixXd C_n;

	Eigen::VectorXd tauGrav;
	Eigen::VectorXd tauExt;

	Eigen::MatrixXd J;
	Eigen::MatrixXd Jdot;

	Eigen::MatrixXd J_n;
	Eigen::MatrixXd Jdot_n;

	Eigen::VectorXd FText;

	Eigen::VectorXd qddot_n;
	Eigen::VectorXd torque_out;
	ControlHubContainer(): reset_cmd(false)
	{
	}
};

class ControlHubHandle {
public:
	b3RobotSimulatorClientAPI_NoDirect m_api;
	b3RobotSimulatorSetPhysicsEngineParameters e_args;
	std::map<int, ControlHubContainer*> chc_map;
};


B3_SHARED_API int initPlugin_ControlHub(struct b3PluginContext* context)
{
	ControlHubHandle* chh_p= new ControlHubHandle();
	b3RobotSimulatorClientAPI_InternalData data;
	data.m_physicsClientHandle = context->m_physClient;
	data.m_guiHelper = 0;
	chh_p->m_api.setInternalData(&data);
	context->m_userPointer = chh_p;

	return SHARED_MEMORY_MAGIC_NUMBER;
}

B3_SHARED_API int preTickPluginCallback_ControlHub(struct b3PluginContext* context)
{
	//apply pd control here, apply forces using the PD gains
	ControlHubHandle* chh_p = (ControlHubHandle*)context->m_userPointer;
	b3RobotSimulatorClientAPI_NoDirect* m_api = &chh_p->m_api;
	for (auto it = chh_p->chc_map.begin(); it != chh_p->chc_map.end(); it++) {
		ControlHubContainer* chc_p = it->second;
		if (chc_p->robot_id < 0) {
			return 0;
		}
		RNB::ControlHubPtr control_hub_p = chc_p->control_hub_p;
		double DT = control_hub_p->DT;
		int JOINT_DOF = control_hub_p->JOINT_DOF;
		int TASK_DOF = control_hub_p->TASK_DOF;
		int TIP_LINK = JOINT_DOF - 1; // manual says lin index == joint index
		double tmp_val;
		bool reset_seq = false;

		// Read Joints
		b3JointStates2 state_r;
		b3JointSensorState j_state;
		for (int i = 0; i < JOINT_DOF; i++) {
			m_api->getJointState(chc_p->robot_id, i, &j_state);
			tmp_val = j_state.m_jointVelocity;
			chc_p->qddot[i] = (tmp_val - chc_p->qdot[i]) / DT;
			chc_p->qdot[i] = tmp_val;
			chc_p->q[i] = j_state.m_jointPosition;
		}

		// Get Tip Position
		b3LinkState lstate;
		m_api->getLinkState(chc_p->robot_id, TIP_LINK, 0, 1, &lstate);
		chc_p->p[0] = lstate.m_worldLinkFramePosition[0];
		chc_p->p[1] = lstate.m_worldLinkFramePosition[1];
		chc_p->p[2] = lstate.m_worldLinkFramePosition[2];

		Eigen::Quaterniond quat(
			lstate.m_worldLinkFrameOrientation[3],
			lstate.m_worldLinkFrameOrientation[0],
			lstate.m_worldLinkFrameOrientation[1],
			lstate.m_worldLinkFrameOrientation[2]); //  w, x, y, z
		Eigen::Vector3d wvu;
		wvu = RNB::Rot2zyx(quat.matrix());
		chc_p->p[3] = wvu[2];
		chc_p->p[4] = wvu[1];
		chc_p->p[5] = wvu[0];

		// Read last joint ForceTorque
		for (int i = 0; i < 6; i++) {
			chc_p->FText[i] = j_state.m_jointForceTorque[i];
		}

		// Handle reset command
		if (chc_p->reset_cmd) {
			chc_p->reset_cmd = false;
			if (control_hub_p->mode == RNB::ControllerInterface::JOINT_CONTROL) {
				control_hub_p->reset_controller(chc_p->q.data());
			}
			else if (control_hub_p->mode == RNB::ControllerInterface::TASK_CONTROL) {
				control_hub_p->reset_controller(chc_p->p.data());
			}
			else {
				throw("undefined control mode");
			}
		}

		if (control_hub_p->check_nominal_reset()) {// reset nominal to joint
			reset_seq = true;
			chc_p->qddot.setZero(); // set qddot to zero to avoid initial numerical error caused by differentiation above
			chc_p->q_n = chc_p->q;
			chc_p->qdot_n = chc_p->qdot;
			chc_p->qddot_n = chc_p->qddot;
            chc_p->q_ref= chc_p->q;
			chc_p->p_ref = chc_p->p;
		}


		b3DynamicsInfo dynamicsInfo;
		Eigen::Vector3d p_end;
		Eigen::MatrixXd Jpos(JOINT_DOF, 3);
		Eigen::MatrixXd Jrot(JOINT_DOF, 3);
		Eigen::VectorXd zerojointvec(JOINT_DOF);
        Eigen::VectorXd zerotaskvec(TASK_DOF);
		Eigen::VectorXd onejointvec(JOINT_DOF);
		Eigen::VectorXd torque_model(JOINT_DOF);
		Eigen::VectorXd torque_g_n(JOINT_DOF);
		Eigen::VectorXd cvec;
		p_end.setZero();
		zerojointvec.setZero();
		onejointvec.setOnes();

		// Get gravity torque by calculating inverse dynamics torque for zero-acc
		m_api->calculateInverseDynamics(chc_p->robot_id, chc_p->q.data(), zerojointvec.data(), zerojointvec.data(),
			chc_p->tauGrav.data());

		// Get Robot Jacobian
		if (control_hub_p->use_real_kinematics) {
			m_api->getBodyJacobian(chc_p->robot_id, TIP_LINK, p_end.data(),
				chc_p->q.data(), chc_p->qdot.data(), chc_p->qddot.data(), Jpos.data(), Jrot.data());
			if (reset_seq) {
				chc_p->Jdot.setZero();
			}
			else {
				chc_p->Jdot.block(0, 0, 3, JOINT_DOF) = Jpos.transpose() - chc_p->J.block(0, 0, 3, JOINT_DOF);
				chc_p->Jdot.block(3, 0, 3, JOINT_DOF) = Jrot.transpose() - chc_p->J.block(3, 0, 3, JOINT_DOF);
				chc_p->Jdot = chc_p->Jdot / chc_p->control_hub_p->DT;
			}
			chc_p->J.block(0, 0, 3, JOINT_DOF) = Jpos.transpose();
			chc_p->J.block(3, 0, 3, JOINT_DOF) = Jrot.transpose();
		}

		// Get NominalJacobian
		if (chc_p->nominal_id >= 0) {
			for (int i = 0; i < JOINT_DOF; i++) {
				m_api->resetJointState(chc_p->nominal_id, i, chc_p->q_n[i]);
			}
		}
		if (control_hub_p->use_nominal_kinematics) {
			if (chc_p->nominal_id < 0) {
				throw("Controller use nominal kinematics but nominal robot is not set");
			}
			m_api->getLinkState(chc_p->nominal_id, TIP_LINK, 0, 1, &lstate);
			chc_p->p_n[0] = lstate.m_worldLinkFramePosition[0];
			chc_p->p_n[1] = lstate.m_worldLinkFramePosition[1];
			chc_p->p_n[2] = lstate.m_worldLinkFramePosition[2];

			Eigen::Quaterniond quat_n(
				lstate.m_worldLinkFrameOrientation[3],
				lstate.m_worldLinkFrameOrientation[0],
				lstate.m_worldLinkFrameOrientation[1],
				lstate.m_worldLinkFrameOrientation[2]); //  w, x, y, z
			wvu = RNB::Rot2zyx(quat_n.matrix());
			chc_p->p_n[3] = wvu[2];
			chc_p->p_n[4] = wvu[1];
			chc_p->p_n[5] = wvu[0];

			m_api->getBodyJacobian(chc_p->robot_id, TIP_LINK, p_end.data(),
				chc_p->q_n.data(), chc_p->qdot_n.data(), chc_p->qddot_n.data(), Jpos.data(), Jrot.data());
			if (reset_seq) {
				chc_p->Jdot_n.setZero();
			}
			else {
				chc_p->Jdot_n.block(0, 0, 3, JOINT_DOF) = Jpos.transpose() - chc_p->J_n.block(0, 0, 3, JOINT_DOF);
				chc_p->Jdot_n.block(3, 0, 3, JOINT_DOF) = Jrot.transpose() - chc_p->J_n.block(3, 0, 3, JOINT_DOF);
			}
			chc_p->J_n.block(0, 0, 3, JOINT_DOF) = Jpos.transpose();
			chc_p->J_n.block(3, 0, 3, JOINT_DOF) = Jrot.transpose();
		}

		if (control_hub_p->use_real_dynamics) {
			// Get Inertia matrix
			m_api->calculateMassMatrix(chc_p->robot_id, chc_p->q.data(), JOINT_DOF, chc_p->M.data(), 0);

			// Get real coriolis by tau(q, qdot) - tau(q), approximate to diagonal
			m_api->calculateInverseDynamics(chc_p->robot_id, chc_p->q.data(), chc_p->qdot.data(), zerojointvec.data(),
				torque_model.data());
			torque_model = torque_model - chc_p->tauGrav;
			cvec = torque_model.cwiseProduct(
				(chc_p->qdot + onejointvec.cwiseMax(chc_p->qdot.cwiseSign())*1e-16
					).cwiseInverse());
			chc_p->C = cvec.asDiagonal();
		}

		if (control_hub_p->use_nominal_dynamics) {
			if (chc_p->nominal_id < 0) {
				throw("Controller use nominal kinematics but nominal robot is not set");
			}
			// Get Inertia matrix
			m_api->calculateMassMatrix(chc_p->robot_id, chc_p->q_n.data(), JOINT_DOF, chc_p->M_n.data(), 0);

			// Get nominal coriolis by tau(q_n, qdot_n) - tau(q_n), approximate to diagonal
			m_api->calculateInverseDynamics(chc_p->robot_id, chc_p->q_n.data(), zerojointvec.data(), zerojointvec.data(),
				torque_g_n.data());
			m_api->calculateInverseDynamics(chc_p->robot_id, chc_p->q_n.data(), chc_p->qdot_n.data(), zerojointvec.data(),
				torque_model.data());
			torque_model = torque_model - torque_g_n;
			cvec = torque_model.cwiseProduct(
				(chc_p->qdot_n + onejointvec.cwiseMax(chc_p->qdot_n.cwiseSign())*1e-16
					).cwiseInverse());
			chc_p->C_n = cvec.asDiagonal();
		}

		// Get external torque as deviation from inverse dynamics torque - the motor is considered to be ideal in bullet.
		m_api->calculateInverseDynamics(chc_p->robot_id, chc_p->q.data(), chc_p->qdot.data(), chc_p->qddot.data(),
			torque_model.data());
		chc_p->tauExt = chc_p->torque_out - torque_model; // exerted torque - motion-calculated torque

														  // Get end-effector force torque by deviation between joint FT value and model acc
		b3JointInfo jnt_info;
		Eigen::Vector3d joint_torque;
		m_api->getJointInfo(chc_p->robot_id, TIP_LINK, &jnt_info);
		joint_torque << jnt_info.m_jointAxis[0], jnt_info.m_jointAxis[1], jnt_info.m_jointAxis[2];
		joint_torque = joint_torque * chc_p->tauExt[TIP_LINK]; // applied joint torque value
		chc_p->FText.block(3, 0, 3, 1) = chc_p->FText.block(3, 0, 3, 1) + joint_torque; // applied joint torque
		chc_p->FText = -chc_p->FText;

		chc_p->qddot_n = chc_p->qddot; // set qddot_n = qddot by default
		chc_p->torque_out = chc_p->tauGrav; // set torque_out = tauGrav by default

		if(control_hub_p->mode==RNB::ControllerInterface::JOINT_CONTROL){
            control_hub_p->calculate_joint_control_torque(chc_p->q.data(), chc_p->qdot.data(), chc_p->qddot.data(),
                                                          chc_p->q_ref.data(), zerojointvec.data(), zerojointvec.data(),
                                                          chc_p->q_n.data(), chc_p->qdot_n.data(), chc_p->p.data(), chc_p->p_n.data(),
                                                          chc_p->M.data(), chc_p->C.data(), chc_p->M_n.data(), chc_p->C_n.data(),
                                                          chc_p->tauGrav.data(), chc_p->tauExt.data(),
                                                          chc_p->J.data(), chc_p->Jdot.data(), chc_p->J_n.data(), chc_p->Jdot_n.data(),
                                                          chc_p->FText.data(),
                                                          chc_p->qddot_n.data(), chc_p->torque_out.data());
		}
        else if(control_hub_p->mode==RNB::ControllerInterface::TASK_CONTROL) {
            control_hub_p->calculate_task_control_torque(chc_p->p.data(), chc_p->p_n.data(),
                                                         chc_p->p_ref.data(), zerotaskvec.data(), zerotaskvec.data(),
                                                         chc_p->q.data(), chc_p->qdot.data(), chc_p->qddot.data(),
                                                         chc_p->q_n.data(), chc_p->qdot_n.data(),
                                                         chc_p->M.data(), chc_p->C.data(), chc_p->M_n.data(), chc_p->C_n.data(),
                                                         chc_p->tauGrav.data(), chc_p->tauExt.data(),
                                                         chc_p->J.data(), chc_p->Jdot.data(), chc_p->J_n.data(), chc_p->Jdot_n.data(),
                                                         chc_p->FText.data(),
                                                         chc_p->qddot_n.data(), chc_p->torque_out.data());
        }
        else{
            throw("Undefined control mode");
        }

		if (control_hub_p->trajectory_interface.follow_traj) {
			chc_p->q_ref = chc_p->q;
		}

		for (int i = 0; i < JOINT_DOF; i++) {
			chc_p->q_n[i] += (DT*chc_p->qdot_n[i]) + (0.5*DT*DT*chc_p->qddot_n[i]);
			chc_p->qdot_n[i] += (DT *chc_p->qddot_n[i]);
			b3RobotSimulatorJointMotorArgs args(CONTROL_MODE_TORQUE);
			args.m_maxTorqueValue = chc_p->torque_out[i];
			m_api->setJointMotorControl(chc_p->robot_id, i, args);
		}
	}

	return 0;
}

B3_SHARED_API int executePluginCommand_ControlHub(struct b3PluginContext* context, const struct b3PluginArguments* arguments)
{
	ControlHubHandle* chh_p = (ControlHubHandle*)context->m_userPointer;

	//protocol:
	//first int is the control mode: 0=Joint / 1=Task / 2=reset / 3=unload
	//second int is the robot id
	//third int is the nominal id
	//fourth int is the ui port
	//fifth int is the trajectory port

	RNB::ControllerInterface::ControlMode mode; 
	int joint_dof;
	int task_dof;
	double dt;
	int ui_port=NULL;
	int traj_port = NULL;

	if (arguments->m_numInts < 2) {
		printf(ANSI_COLOR_RED "================= ControlHubPlugin needs at least 2 arguments (command, robot_id) ==============\n" ANSI_COLOR_RESET);
		return -1;
	}

	mode = static_cast<RNB::ControllerInterface::ControlMode>(arguments->m_ints[0]);
	int robot_id = arguments->m_ints[1];
	ControlHubContainer* chc_p;
	if (chh_p->chc_map.find(robot_id) == chh_p->chc_map.end()) {
		chc_p = new ControlHubContainer();
		chc_p->robot_id = robot_id;
		chh_p->chc_map[robot_id] = chc_p;
		chh_p->m_api.syncBodies();
		chh_p->m_api.getPhysicsEngineParameters(chh_p->e_args);
	}
	else {
		chc_p = chh_p->chc_map[robot_id];
	}
	
	switch (mode)
	{
	case 2: // reset controller
	{
		chc_p->reset_cmd = true;
		break;
	}
	case 3: // unload controller
	{
		auto chc = chh_p->chc_map[robot_id];
		chh_p->chc_map.erase(robot_id);
		delete chc;
		break;
	}
	default:
	{
		if (arguments->m_numInts < 3) {
			printf(ANSI_COLOR_RED "================= Creating ControlHub need at least 3 arguments (mode, robot_id, nominal_id) ==============\n" ANSI_COLOR_RESET);
			return -1;
		}
		chc_p->nominal_id = arguments->m_ints[2];
		if (arguments->m_numInts>3)	ui_port = arguments->m_ints[3];
		if (arguments->m_numInts>4) traj_port = arguments->m_ints[4];
		joint_dof = chh_p->m_api.getNumJoints(chc_p->robot_id);
		task_dof = 6;
		dt = chh_p->e_args.m_deltaTime;
		chc_p->control_hub_p = std::make_shared<RNB::ControlHub>(
			mode, joint_dof, task_dof, dt, ui_port, traj_port);
		if(mode==RNB::ControllerInterface::TASK_CONTROL){
		    std::list<RNB::AlgebraPtr> alg_list;
            alg_list.push_back(std::make_shared<RNB::Euclidean>(3));
            alg_list.push_back(std::make_shared<RNB::RotationUVW>());
		    auto alg_p = std::make_shared<RNB::Combined>(alg_list);
		    chc_p->control_hub_p->set_task_space(alg_p);
		}
		chc_p->q.setZero(joint_dof, 1);
		chc_p->qdot.setZero(joint_dof, 1);
		chc_p->qddot.setZero(joint_dof, 1);
		chc_p->q_n.setZero(joint_dof, 1);
		chc_p->qdot_n.setZero(joint_dof, 1);

        chc_p->q_ref.setZero(joint_dof, 1);

        chc_p->p_ref.setZero(task_dof, 1);

		chc_p->p.setZero(task_dof, 1);
		chc_p->p_n.setZero(task_dof, 1);

		chc_p->M.setZero(joint_dof, joint_dof);
		chc_p->M_n.setZero(joint_dof, joint_dof);

		chc_p->C.setZero(joint_dof, joint_dof);
		chc_p->C_n.setZero(joint_dof, joint_dof);

		chc_p->tauGrav.setZero(joint_dof, 1);
		chc_p->tauExt.setZero(joint_dof, 1);

		chc_p->J.setZero(task_dof, joint_dof);
		chc_p->Jdot.setZero(task_dof, joint_dof);

		chc_p->J_n.setZero(task_dof, joint_dof);
		chc_p->Jdot_n.setZero(task_dof, joint_dof);

		chc_p->FText.setZero(task_dof, 1);

		chc_p->qddot_n.setZero(joint_dof, 1);
		chc_p->torque_out.setZero(joint_dof, 1);
		chc_p->reset_cmd = true;
	}
	}

	return 0;
}

B3_SHARED_API void exitPlugin_ControlHub(struct b3PluginContext* context)
{
	ControlHubHandle * chh_p = (ControlHubHandle*)context->m_userPointer;
	for (auto it = chh_p->chc_map.begin(); it != chh_p->chc_map.end(); it++) {
		delete it->second;
	}
}
