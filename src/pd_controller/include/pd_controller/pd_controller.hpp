#pragma once

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "sensor_msgs/msg/joint_state.hpp"

#include <Eigen/Core>

#include <string>
#include <vector>



namespace pd_controller {

/* ========================================================================== */
/*                                PDCONTROLLER                               */
/* ========================================================================== */

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class PDController : public controller_interface::ControllerInterface {
public:
    PDController();

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::return_type update(
        const rclcpp::Time& time, const rclcpp::Duration& /*period*/
    ) override;

    CallbackReturn on_init() override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State& /*previous_state*/) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& /*previous_state*/) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override;
    // CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
    // CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;
    // CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscription_ = nullptr;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr vj_ref_subscription_ = nullptr;

    std::vector<std::string> joint_names_;

    Eigen::VectorXd qj_;
    Eigen::VectorXd vj_;

    std::vector<double> qj_ref_;
    std::vector<double> vj_ref_;
    std::vector<double> tau_ref_;

    /// @brief Initialization time to give the state estimator some time to get better estimates. During this time, a PD controller is used to keep the robot in q0 and the planner is paused.
    double init_time_ = 1;
    std::vector<double> init_phases_ = {1};

    Eigen::VectorXd q0_;
    Eigen::VectorXd q1_;
    Eigen::VectorXd q2_;

    std::vector<double> PD_proportional_;
    std::vector<double> PD_derivative_;
};

} // namespace pd_controller