#include "external_controller/external_controller.hpp"

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace external_controller
{
  
  using hardware_interface::LoanedCommandInterface;

  EXTERNALController::EXTERNALController()
    : controller_interface::ControllerInterface(),
      params_subscriber_(nullptr)
  {
  }

  void EXTERNALController::receive_torque(const custom_msg::msg::Torque::SharedPtr msg){
    torque_ = msg->torque;
  }

  CallbackReturn EXTERNALController::on_init()
  {
    try {
      // definition of the parameters that need to be queried from the
      // controller configuration file with default values
      auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    } catch (const std::exception & e) {
      fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
      return CallbackReturn::ERROR;
    }

    joint_names_ = {"joint_a1",
      "joint_a2",
      "joint_a3",
      "joint_a4",
      "joint_a5",
      "joint_a6",
      "joint_a7"};

    torque_ = {0,0,0,0,0,0,0};
    it = 0;

    return CallbackReturn::SUCCESS;
  }
  
  CallbackReturn EXTERNALController::on_configure(
						  const rclcpp_lifecycle::State & /*previous_state*/)
  {

    joint_names_ = get_node()->get_parameter("joints").as_string_array();
    
    params_subscriber_ = get_node()->create_subscription<CmdType>("/torque", rclcpp::SystemDefaultsQoS(),[this](const CmdType::SharedPtr msg) { receive_torque(msg); });
    publisher_ = get_node()->create_publisher<stType>("custom_joint_states", 10);
    
    RCLCPP_INFO(get_node()->get_logger(), "configure successful");
    return CallbackReturn::SUCCESS;
  }
  
  controller_interface::InterfaceConfiguration
  EXTERNALController::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration conf;
    conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    conf.names.reserve(joint_names_.size());
    for (const auto & joint_name : joint_names_)
      {
	conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_EFFORT);
      }
    return conf;
  }
  
  controller_interface::InterfaceConfiguration
  EXTERNALController::state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration conf;
    conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    conf.names.reserve(joint_names_.size() * 3);
    for (const auto & joint_name : joint_names_)
      {
      conf.names.push_back(joint_name + "/" +
                           hardware_interface::HW_IF_POSITION);
      conf.names.push_back(joint_name + "/" +
                           hardware_interface::HW_IF_VELOCITY);
      conf.names.push_back(joint_name + "/" +
			   hardware_interface::HW_IF_EFFORT);
      }
    return conf;
  }
  
  // Fill ordered_interfaces with references to the matching interfaces
  // in the same order as in joint_names
  template <typename T>
  bool get_ordered_interfaces(
      std::vector<T> &unordered_interfaces,
      const std::vector<std::string> &joint_names,
      const std::string &interface_type,
      std::vector<std::reference_wrapper<T>> &ordered_interfaces) {
    for (const auto &joint_name : joint_names) {
      for (auto &command_interface : unordered_interfaces) {
        if (command_interface.get_name() == joint_name + "/" + interface_type) {
          ordered_interfaces.push_back(std::ref(command_interface));
        }
      }
    }

    return joint_names.size() == ordered_interfaces.size();
  }

  CallbackReturn EXTERNALController::on_activate(
						 const rclcpp_lifecycle::State & /*previous_state*/)
  {
    std::vector<std::reference_wrapper<LoanedCommandInterface>>
        ordered_interfaces;
    if (!get_ordered_interfaces(command_interfaces_, joint_names_, "effort",
                                ordered_interfaces) ||
        command_interfaces_.size() != ordered_interfaces.size()) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Expected %zu position command interfaces, got %zu",
                   joint_names_.size(), ordered_interfaces.size());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
  }
  
  // When deactivating the controller, the effort command on all joints is set to 0
  CallbackReturn EXTERNALController::on_deactivate(
						   const rclcpp_lifecycle::State & /*previous_state*/)
  {
    for (auto index = 0ul; index < joint_names_.size(); ++index) {
      command_interfaces_[index].set_value(0.0);
    }
    return CallbackReturn::SUCCESS;
  }
  
  // main control loop function getting the state interface and writing to the command interface
  controller_interface::return_type EXTERNALController::update(const rclcpp::Time & /*time*/,
							       const rclcpp::Duration & /*period*/)
  {
    auto message = stType();
    std::vector<double> aux;
    aux.resize(7);
    for(int i = 0; i<7; ++i)
      aux[i] = state_interfaces_[i * 3].get_value();
    message.position = aux;
    for(int i = 0; i<7; ++i)
      aux[i] = state_interfaces_[i*3+1].get_value();
    message.velocity = aux;
    message.id = it++;
    publisher_->publish(message);
    
    for(int joint = 0; joint<7; ++joint)
      command_interfaces_[joint].set_value(torque_[joint]);

    return controller_interface::return_type::OK;
  }

}  // namespace external_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(external_controller::EXTERNALController, controller_interface::ControllerInterface)
