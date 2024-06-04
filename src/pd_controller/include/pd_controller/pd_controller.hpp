#ifndef PD_CONTROLLER__PD_CONTROLLER_HPP_
#define PD_CONTROLLER__PD_CONTROLLER_HPP_

#include <chrono>
#include <fstream>
#include <list>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "custom_msg/msg/pdparams.hpp"
#include "custom_msg/msg/torque.hpp"
#include "pd_controller/visibility_control.h"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

using CmdType = custom_msg::msg::Pdparams;

namespace pd_controller
{
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /**
   * \brief Pd controller for a set of joints.
   *
   * This class computes the torque given by the pd relationship and applies it to the defined joints.
   *
   * \param joints Names of the joints to control.
   * \param interface_name Name of the interface to command.
   *
   * Subscribes to:
   * - \b proxy (trajectory_msgs::msg::JointTrajectory) : The trajectory of the proxy to follow.
   */
  class PDController : public controller_interface::ControllerInterface
  {
  public:
    PD_CONTROLLER_PUBLIC
    PDController();

    PD_CONTROLLER_PUBLIC
    CallbackReturn on_init() override;

    /**
     * @brief command_interface_configuration This controller requires the effort command
     * interface for the controlled joints
     */
    PD_CONTROLLER_PUBLIC
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    /**
     * @brief command_interface_configuration This controller requires the position
     * state interface for the controlled joints
     */
    PD_CONTROLLER_PUBLIC
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    PD_CONTROLLER_PUBLIC
    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

    PD_CONTROLLER_PUBLIC
    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    PD_CONTROLLER_PUBLIC
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    PD_CONTROLLER_PUBLIC
    controller_interface::return_type update(
					     const rclcpp::Time & time,
					     const rclcpp::Duration & period) override;

    void load_params(const custom_msg::msg::Pdparams::SharedPtr params);

  protected:
    std::vector<std::string> joint_names_;
    std::vector<float> kp_;
    std::vector<float> kd_;
    std::vector<double> max_torques_;
    std::vector<double> joint_limits_;

    // Input arrays
    std::vector<double> q_extra_, v_extra_;
    std::vector<double> q_trajectory_, v_trajectory_;

    // Data arrays
    std::string output_path_;
    std::vector<double> q_desired_, v_desired_;
    std::vector<double> trajectory_history_, velocity_history_,
      torque_history_, commanded_history_;

    // ROS communication
    rclcpp::Subscription<CmdType>::SharedPtr params_subscriber_;

    // MSG variables
    int controller_id_;
    
  private:
    void update_params(const CmdType::SharedPtr msg);
    bool trajectory_started_, active_;
    int num_joints_;
    int step_index_;

  };

}  // namespace pd_controller

#endif  // PD_CONTROLLER__PD_CONTROLLER_HPP_
