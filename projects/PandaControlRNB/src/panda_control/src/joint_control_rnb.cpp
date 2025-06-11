// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <panda_control/joint_control_rnb.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

#define PI__ 3.1415926535897932384626433832795028841971693993751058209

namespace panda_control {
    JointControlRNB::JointControlRNB(bool allow_optional_interfaces) :
    MultiInterfaceController(allow_optional_interfaces),
    control_hub(RNB::ControllerInterface::JOINT_CONTROL, JOINT_DOF, TASK_DOF, 1.0/CONTROL_FREQ){
    }

    bool JointControlRNB::init(hardware_interface::RobotHW* robot_hw,
                               ros::NodeHandle& node_handle) {
        std::string arm_id;
        if (!node_handle.getParam("arm_id", arm_id)) {
            ROS_ERROR("JointControlRNB: Could not read parameter arm_id");
            return false;
        }

        std::vector<std::string> joint_names;
        if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != JOINT_DOF) {
            ROS_ERROR(
                    "JointControlRNB: Invalid or no joint_names parameters provided, aborting "
                    "controller init!");
            return false;
        }

        auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
        if (model_interface == nullptr) {
            ROS_ERROR_STREAM(
                    "JointControlRNB: Error getting model interface from hardware");
            return false;
        }
        try {
            auto handle = model_interface->getHandle(arm_id + "_model");
            model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(handle);
        } catch (hardware_interface::HardwareInterfaceException& ex) {
            ROS_ERROR_STREAM(
                    "JointControlRNB: Exception getting model handle from interface: "
                            << ex.what());
            return false;
        }

        auto* state_interface_ = robot_hw->get<franka_hw::FrankaStateInterface>();
        if (state_interface_ == nullptr) {
            ROS_ERROR_STREAM(
                    "JointControlRNB: Error getting state handle from hardware");
            return false;
        }
        try {
            state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
                    state_interface_->getHandle(arm_id + "_robot"));
        } catch (hardware_interface::HardwareInterfaceException& ex) {
            ROS_ERROR_STREAM(
                    "JointControlRNB: Exception getting cartesian pose handle from interface: "
                            << ex.what());
            return false;
        }

        auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
        if (effort_joint_interface == nullptr) {
            ROS_ERROR_STREAM(
                    "JointControlRNB: Error getting effort joint interface from hardware");
            return false;
        }
        for (size_t i = 0; i < JOINT_DOF; ++i) {
            try {
                joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
            } catch (const hardware_interface::HardwareInterfaceException& ex) {
                ROS_ERROR_STREAM(
                        "JointControlRNB: Exception getting joint handles: " << ex.what());
                return false;
            }
        }

        return true;
    }

    void JointControlRNB::starting(const ros::Time& /*time*/) {
        auto robot_state = state_handle_->getRobotState();
        q_init = robot_state.q;
        q_n = q_init;
        std::fill(dq_prev.begin(), dq_prev.end(), 0);
        std::fill(dq_n.begin(), dq_n.end(), 0);
        std::fill(ddq_n_out.begin(), ddq_n_out.end(), 0);
        std::cout <<"q_init: ";
        for (auto qitor = q_init.begin(); qitor!=q_init.end(); qitor++){
            std::cout << *qitor << ", ";
        }
        std::cout<<std::endl;
        control_hub.reset_controller(q_init.data());
    }

    void JointControlRNB::update(const ros::Time& time,
                                 const ros::Duration& period) {
        // get robot states
        franka::RobotState robot_state = state_handle_->getRobotState();

        if(control_hub.check_nominal_reset()){
            q_n = robot_state.q;
            dq_n = robot_state.dq;
        }

        Eigen::MatrixXd M(JOINT_DOF, JOINT_DOF);
        Eigen::MatrixXd C(JOINT_DOF, JOINT_DOF);
        Eigen::VectorXd Cvec(JOINT_DOF);
        Eigen::MatrixXd M_n(JOINT_DOF, JOINT_DOF);
        Eigen::MatrixXd C_n(JOINT_DOF, JOINT_DOF);
        Eigen::VectorXd Cvec_n(JOINT_DOF);

        if (control_hub.use_real_dynamics){
            std::array<double, JOINT_DOF*JOINT_DOF> mass = model_handle_->getMass();
            std::array<double, JOINT_DOF> coriolis = model_handle_->getCoriolis(); // C*dq
            RNB::double_to_matrix(mass.data(), M);
            RNB::double_to_vector(coriolis.data(), Cvec);
        }

        if (control_hub.use_nominal_dynamics) {
            std::array<double, JOINT_DOF * JOINT_DOF> mass_n =
                    model_handle_->getMass(q_n,
                                           model_handle_->robot_state_->I_total, model_handle_->robot_state_->m_total,
                                           model_handle_->robot_state_->F_x_Ctotal
                    );
            std::array<double, JOINT_DOF> coriolis_n =
                    model_handle_->getCoriolis(q_n, dq_n,
                                               model_handle_->robot_state_->I_total,
                                               model_handle_->robot_state_->m_total,
                                               model_handle_->robot_state_->F_x_Ctotal); // C*dq
            RNB::double_to_matrix(mass_n.data(), M_n);
            RNB::double_to_vector(coriolis_n.data(), Cvec_n);
        }

        std::array<double, JOINT_DOF> gravity = model_handle_->getGravity();

        std::array<double, JOINT_DOF> ddq;
        for (size_t i = 0; i < JOINT_DOF; i++) {
            ddq[i] = (robot_state.dq[i] - dq_prev[i])*CONTROL_FREQ;
            Cvec[i] = Cvec[i]/robot_state.dq[i]; // Cvec is C*dq. to estimate C, we divide Cvec by dq
            if(dq_n[i] == 0){
                Cvec_n[i] = 0;
            }
            else{
                Cvec_n[i] = Cvec_n[i]/dq_n[i]; // Cvec is C*dq. to estimate C, we divide Cvec by dq
            }
        }
        C = Cvec.asDiagonal();
        C_n = Cvec_n.asDiagonal();

        std::array<double, JOINT_DOF*TASK_DOF> jac;
        std::array<double, JOINT_DOF*TASK_DOF> jac_n;

        std::array<double, 16> pos;
        std::array<double, 16> pos_n;

        if(control_hub.use_real_kinematics){
            jac = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
            pos = model_handle_->getPose(franka::Frame::kEndEffector);
        }

        if(control_hub.use_nominal_kinematics){
            jac_n =
                    model_handle_->getZeroJacobian(franka::Frame::kEndEffector, q_n,
                                                   model_handle_->robot_state_->F_T_EE,
                                                   model_handle_->robot_state_->EE_T_K);
            pos_n = model_handle_->getPose(franka::Frame::kEndEffector, q_n,
                                           model_handle_->robot_state_->F_T_EE,
                                           model_handle_->robot_state_->EE_T_K);
        }

        Eigen::MatrixXd J(TASK_DOF, JOINT_DOF);
        Eigen::MatrixXd J_n(TASK_DOF, JOINT_DOF);
        Eigen::MatrixXd Jdot(TASK_DOF, JOINT_DOF);
        Eigen::MatrixXd Jdot_n(TASK_DOF, JOINT_DOF);
        RNB::double_to_matrix(jac.data(), J);
        RNB::double_to_matrix(jac_n.data(), J_n);
        Jdot.setZero();
        Jdot_n.setZero();

        std::array<double, TASK_DOF> Fext = {0};

        control_hub.calculate_joint_control_torque(robot_state.q.data(), robot_state.dq.data(), ddq.data(),
                                                   control_hub.q_d.data(), control_hub.qdot_d.data(), control_hub.qddot_d.data(),
                                                   q_n.data(), dq_n.data(),
                                                   pos.data(), pos_n.data(),
                                                   M.data(), C.data(), M_n.data(), C_n.data(),
                                                   gravity.data(), robot_state.tau_J.data(),
                                                   J.data(), Jdot.data(),
                                                   J_n.data(), Jdot_n.data(), // need nominal jacobian here
                                                   Fext.data(),
                                                   ddq_n_out.data(), torque_out.data()
                                                   );

        dq_prev = robot_state.dq;

        std::array<double, JOINT_DOF> tau_d_calculated;
        for (size_t i = 0; i < JOINT_DOF; ++i) {
            q_n[i] += ((dq_n[i]/CONTROL_FREQ) + (0.5*ddq_n_out[i]/CONTROL_FREQ/CONTROL_FREQ));
            dq_n[i] += (ddq_n_out[i]/CONTROL_FREQ);
            tau_d_calculated[i] = torque_out[i] - gravity[i]; // subtract gravity as it seems like added automatically
        }

//        // Maximum torque difference with a sampling rate of 1 kHz. The maximum torque rate is
//        // 1000 * (1 / sampling_time).
//        std::array<double, JOINT_DOF> tau_d_saturated = saturateTorqueRate(tau_d_calculated, robot_state.tau_J_d);

        for (size_t i = 0; i < JOINT_DOF; ++i) {
            joint_handles_[i].setCommand(tau_d_calculated[i]);
        }
    }

    std::array<double, JOINT_DOF> JointControlRNB::saturateTorqueRate(
            const std::array<double, JOINT_DOF>& tau_d_calculated,
            const std::array<double, JOINT_DOF>& tau_J_d) {  // NOLINT (readability-identifier-naming)
        std::array<double, JOINT_DOF> tau_d_saturated{};
        for (size_t i = 0; i < JOINT_DOF; i++) {
            double difference = tau_d_calculated[i] - tau_J_d[i];
            tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
        }
        return tau_d_saturated;
    }

}  // namespace panda_control

PLUGINLIB_EXPORT_CLASS(panda_control::JointControlRNB,
                       controller_interface::ControllerBase)
