#include "pd_controller/pd_controller.hpp"

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

void WriteVectorToFile(std::vector<double> v, std::string filename){

  std::ofstream file;

  file.open(filename, std::ios_base::binary);

  const char *pointer = reinterpret_cast<const char *>(&v[0]);

  file.write(pointer, v.size() * sizeof(v[0]));

  file.close();
}

bool LoadVectorFromFile(std::vector<double> & v, std::string filename){

  std::ifstream input_stream;

  input_stream.open(filename, std::ios::binary);
  input_stream.seekg(0, std::ios::end);
  size_t filesize=input_stream.tellg();
  input_stream.seekg(0, std::ios::beg);

  v.resize(filesize/sizeof(double));

  input_stream.read((char *)v.data(), filesize);

  return true;
}

namespace pd_controller
{
  
  void PDController::update_params(const CmdType::SharedPtr msg){
    // We first check if its a new message
    if(msg->id != controller_id_){
      RCLCPP_INFO(get_node()->get_logger(), "PD params received");
      // We copy the data of the message
      controller_id_ = msg->id;
      for(int i = 0; i<num_joints_; ++i){
	kp_[i] = msg->kp[i];
	kd_[i] = msg->kd[i];
	max_torques_[i] = msg->max_torques[i];
        joint_limits_[i] = msg->joint_limits[i];
      }

      std::string joints_filename = msg->joints_filename;
      std::string velocity_filename = msg->velocity_filename;
      output_path_ = msg->output_filename;

      LoadVectorFromFile(q_trajectory_, joints_filename);
      LoadVectorFromFile(v_trajectory_, velocity_filename);

      // Resize history arrays to trajectory size
      int history_size = q_trajectory_.size();
      trajectory_history_.resize(history_size);
      velocity_history_.resize(history_size);
      torque_history_.resize(history_size);
      commanded_history_.resize(history_size);

      // Get robot into position
      step_index_ = 0;
      active_ = true;
    }
  }
  
  using hardware_interface::LoanedCommandInterface;

  PDController::PDController()
    : controller_interface::ControllerInterface(),
      params_subscriber_(nullptr)
  {
  }

  CallbackReturn PDController::on_init()
  {
    try {
      // definition of the parameters that need to be queried from the
      // controller configuration file with default values
      auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    } catch (const std::exception & e) {
      fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
      return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
  }
  
  CallbackReturn PDController::on_configure(
					    const rclcpp_lifecycle::State & /*previous_state*/)
  {

    active_ = false;
    controller_id_ = 0;
    step_index_ = -1;
    
    // Getting the names of the joints to be controlled
    joint_names_ = get_node()->get_parameter("joints").as_string_array();

    num_joints_ = joint_names_.size();
    kp_.resize(num_joints_,0);
    kd_.resize(num_joints_,0);
    q_desired_.resize(num_joints_);
    v_desired_.resize(num_joints_);
    max_torques_.resize(num_joints_,0);
    joint_limits_.resize(num_joints_, 0);

    if (joint_names_.empty()) {
      RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
      return CallbackReturn::FAILURE;
    }

    params_subscriber_ = get_node()->create_subscription<CmdType>("/params", rclcpp::SystemDefaultsQoS(),[this](const CmdType::SharedPtr msg) { update_params(msg); });
    
    RCLCPP_INFO(get_node()->get_logger(), "configure successful");
    return CallbackReturn::SUCCESS;
  }
  
  // As PD control targets the effort interface, it can be directly defined here 
  // without the need of getting as parameter. The effort interface is then affected to
  // all controlled joints.
  controller_interface::InterfaceConfiguration
  PDController::command_interface_configuration() const
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
  
  // PD control requires both velocity and position states. For this reason 
  // there can be directly defined here without the need of getting as parameters.
  // The state interfaces are then deployed to all targeted joints.
  controller_interface::InterfaceConfiguration
  PDController::state_interface_configuration() const
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

  CallbackReturn PDController::on_activate(
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
  CallbackReturn PDController::on_deactivate(
					     const rclcpp_lifecycle::State & /*previous_state*/)
  {
    for (auto index = 0ul; index < joint_names_.size(); ++index) {
      command_interfaces_[index].set_value(0.0);
    }
    return CallbackReturn::SUCCESS;
  }
  
  // main control loop function getting the state interface and writing to the command interface
  controller_interface::return_type PDController::update(const rclcpp::Time & /*time*/,
							 const rclcpp::Duration & /*period*/)
  {
    double q, v, torque, tau, tp, td;
    int history_index;
    for (int joint = 0; joint < num_joints_; ++joint) {
      q = state_interfaces_[joint * 3].get_value();
      v = state_interfaces_[joint * 3+1].get_value();
      torque = state_interfaces_[joint * 3+2].get_value();

      history_index = step_index_*num_joints_+joint;
      
      if(step_index_ < 0){
	if(active_){
	  q_desired_[joint] = q_trajectory_[joint];
	  v_desired_[joint] = 0.0;
	}else{
	  q_desired_[joint] = q;
	  v_desired_[joint] = 0.0;
	}
      }else{
	q_desired_[joint] = q_trajectory_[history_index];
	v_desired_[joint] = v_trajectory_[history_index];
      }

      tp = kp_[joint] * (q_desired_[joint] - q);
      td = kd_[joint] * (v_desired_[joint] - v);

      tau = tp + td;

      // Limit maximun torque
      if (tau > max_torques_[joint])
	tau = max_torques_[joint];
      else if (tau < -max_torques_[joint])
	tau = -max_torques_[joint];

      command_interfaces_[joint].set_value(tau);
      
      if(step_index_ >= 0){
	commanded_history_[history_index] = tau;
	trajectory_history_[history_index] = q;
	velocity_history_[history_index] = v;
	torque_history_[history_index] = torque;
      }
    }

    if(active_){
      ++step_index_;

      // Check if trajectory is finished
      if(step_index_ >= ((int)q_trajectory_.size() / num_joints_)){
	step_index_ = -1;
	active_ = false;

	// Save data for analysis
	WriteVectorToFile(trajectory_history_, output_path_ + "_followed");
	WriteVectorToFile(velocity_history_, output_path_ + "_velocity");
	WriteVectorToFile(torque_history_, output_path_ + "_torques");
	WriteVectorToFile(commanded_history_, output_path_ + "_commanded");
      }
    }
    
    return controller_interface::return_type::OK;
  }

}  // namespace pd_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(pd_controller::PDController, controller_interface::ControllerInterface)
