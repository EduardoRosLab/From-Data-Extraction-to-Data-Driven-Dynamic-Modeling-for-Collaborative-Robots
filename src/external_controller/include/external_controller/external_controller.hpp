#ifndef PD_CONTROLLER__PD_CONTROLLER_HPP_
#define PD_CONTROLLER__PD_CONTROLLER_HPP_

#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "custom_msg/msg/state.hpp"
#include "custom_msg/msg/torque.hpp"
#include "external_controller/visibility_control.h"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

using CmdType = custom_msg::msg::Torque;
using stType = custom_msg::msg::State;

namespace external_controller
{
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  class EXTERNALController : public controller_interface::ControllerInterface
  {
  public:
    EXTERNAL_CONTROLLER_PUBLIC
    EXTERNALController();

    EXTERNAL_CONTROLLER_PUBLIC
    CallbackReturn on_init() override;

    /**
     * @brief command_interface_configuration This controller requires the effort command
     * interface for the controlled joints
     */
    EXTERNAL_CONTROLLER_PUBLIC
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    /**
     * @brief command_interface_configuration This controller requires the position
     * state interface for the controlled joints
     */
    EXTERNAL_CONTROLLER_PUBLIC
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    EXTERNAL_CONTROLLER_PUBLIC
    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

    EXTERNAL_CONTROLLER_PUBLIC
    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    EXTERNAL_CONTROLLER_PUBLIC
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    EXTERNAL_CONTROLLER_PUBLIC
    controller_interface::return_type update(
					     const rclcpp::Time & time,
					     const rclcpp::Duration & period) override;

    void receive_torque(const CmdType::SharedPtr torque);

  protected:
    // External torque command
    std::vector<float> torque_;
    std::vector<std::string> joint_names_;
    int it;
    
    
    // ROS communication
    rclcpp::Subscription<CmdType>::SharedPtr params_subscriber_;
    rclcpp::Publisher<stType>::SharedPtr publisher_;
    
  };

}  // namespace external_controller

#endif  // EXTERNAL_CONTROLLER__EXTERNAL_CONTROLLER_HPP_
