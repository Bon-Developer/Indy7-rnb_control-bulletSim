// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <panda_control/JointTorqueComparison.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/trigger_rate.h>

#include "control_hub.h"
#include "franka_model_interface_custom.h"

#define JOINT_DOF 7
#define TASK_DOF 6
#define CONTROL_FREQ 1000.0

namespace panda_control {

    class JointControlRNB : public controller_interface::MultiInterfaceController<
            franka_hw::FrankaModelInterface,
            hardware_interface::EffortJointInterface,
            franka_hw::FrankaStateInterface> {
    public:
        /**
         * \param allow_optional_interfaces If set to true, \ref initRequest will
         * not fail if one or more of the requested interfaces is not present.
         * If set to false (the default), all requested interfaces are required.
         */
        JointControlRNB(bool allow_optional_interfaces = false);
        bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
        void starting(const ros::Time&) override;
        void update(const ros::Time&, const ros::Duration& period) override;

    private:
        RNB::ControlHub control_hub;
        // Saturation
        std::array<double, JOINT_DOF> saturateTorqueRate(
                const std::array<double, JOINT_DOF>& tau_d_calculated,
                const std::array<double, JOINT_DOF>& tau_J_d);  // NOLINT (readability-identifier-naming)

        std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
        std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
        std::vector<hardware_interface::JointHandle> joint_handles_;

        static constexpr double kDeltaTauMax{1.0};

        // RNB
        std::array<double, JOINT_DOF> dq_prev;
        std::array<double, JOINT_DOF> q_init;
        std::array<double, JOINT_DOF> q_n;
        std::array<double, JOINT_DOF> dq_n;
        std::array<double, JOINT_DOF> ddq_n_out;
        std::array<double, JOINT_DOF> torque_out;
    };

}  // namespace panda_control
