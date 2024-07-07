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

    Eigen::VectorXd qj_;    ///< @brief Measured joint positions.
    Eigen::VectorXd vj_;    ///< @brief Measured joint velocities.

    std::vector<double> qj_ref_;    ///< @brief Reference joint positions.
    std::vector<double> vj_ref_;    ///< @brief Reference joint velocities.
    std::vector<double> tau_ref_;   ///< @brief Feed-forward joint torques.

    double init_time_1_ = 1;    ///< @brief Initialization time from q0 to q1.
    double init_time_2_ = 1;    ///< @brief Initialization time from q1 to q2.
    
    Eigen::VectorXd q0_;
    Eigen::VectorXd q1_;
    Eigen::VectorXd q2_;

    std::vector<double> PD_proportional_;
    std::vector<double> PD_derivative_;


    bool use_torques_ = true;       ///< @brief When false, the feed-forward torques term is not used.
    bool use_velocities_ = true;    ///< @brief When false, the derivative contribution is: PD_derivative * (- q_dot_meas).
};

} // namespace pd_controller