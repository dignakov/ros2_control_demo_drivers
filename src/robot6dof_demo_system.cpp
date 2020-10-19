// Copyright 2020 ROS2-Control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ros2_control_demo_driver/robot6dof_demo_system.hpp"
#include <rclcpp/utilities.hpp>
#include <thread>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "abb_libegm/egm_controller_interface.h"
#include "hardware_interface/components/component_info.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
namespace ros2_control_demo_driver
{

return_type Robot6DofSystemHardware::configure(const HardwareInfo & system_info)
{
  if (hardware_interface::BaseSystemHardwareInterface::configure(system_info) != return_type::OK) {
	return return_type::ERROR;
  }
  
  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);
  hw_states_.resize(info_.joints.size());
  hw_commands_.resize(info_.joints.size());
  for (uint i = 0; i < info_.joints.size(); i++) {
    hw_states_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_commands_[i] = std::numeric_limits<double>::quiet_NaN();
  }
  for (const hardware_interface::components::ComponentInfo & joint : info_.joints) {
    if (joint.class_type.compare("ros2_control_components/PositionJoint") != 0) {
      status_ = hardware_interface_status::UNKNOWN;
      // TODO(all): should we return specizalized error?
      return return_type::ERROR;
    }
  }

  //EGM
  RCLCPP_INFO(rclcpp::get_logger("Robot6DofSystemHardware"), "Configuring EGM interface...");
  egm_interface_ = std::make_unique<abb::egm::EGMControllerInterface>(io_service_, 6511);

  	if(!egm_interface_){
  		RCLCPP_FATAL(rclcpp::get_logger("Robot6DofSystemHardware"),
				"Could not create EGMControllerInterface");
      	return return_type::ERROR;
	}

	if(!egm_interface_->isInitialized())
	{
  		RCLCPP_FATAL(rclcpp::get_logger("Robot6DofSystemHardware"),
				"EGM UDP Server interface failed to initialize (e.g. due to port already bound)");
      	return return_type::ERROR; //TODO: this does not seem to have an effect
	}





  status_ = hardware_interface_status::CONFIGURED;
  return return_type::OK;
}

return_type Robot6DofSystemHardware::start()
{
  RCLCPP_INFO(rclcpp::get_logger("Robot6DofSystemHardware"),
    "Starting ...please wait...");

  //for (int i = 0; i <= hw_start_sec_; i++) {
    //rclcpp::sleep_for(std::chrono::seconds(1));
    //RCLCPP_INFO(rclcpp::get_logger("Robot6DofSystemHardware"),
      //"%.1f seconds left...", hw_start_sec_ - i);
  //}


  //EGM
    thread_group_.create_thread(boost::bind(&boost::asio::io_service::run, &io_service_));

	bool wait=true;
	int num_tries = 3;
	int counter = 0;
  	RCLCPP_INFO(rclcpp::get_logger("Robot6DofSystemHardware"), "Connecting to Robot...");
	while(wait && counter++ < num_tries){
        if(egm_interface_->isConnected())
        {
  			RCLCPP_INFO(rclcpp::get_logger("Robot6DofSystemHardware"), "Connected to Robot");
            if(egm_interface_->getStatus().rapid_execution_state() == abb::egm::wrapper::Status_RAPIDExecutionState_RAPID_UNDEFINED)
            {
  				RCLCPP_WARN(rclcpp::get_logger("Robot6DofSystemHardware"),
						"configure RAPID execution state is UNDEFINED (might happen first time after controller start/restart). Try to restart the RAPID program.");
            }
            else
            {
                wait = egm_interface_->getStatus().rapid_execution_state() != abb::egm::wrapper::Status_RAPIDExecutionState_RAPID_RUNNING;
            }
        }
        else {
  			RCLCPP_INFO(rclcpp::get_logger("Robot6DofSystemHardware"), "Not Connected to Robot...");
        }
		rclcpp::sleep_for(500ms);
	}
	if(wait){  // if still not in the right state, exit
  		RCLCPP_FATAL(rclcpp::get_logger("Robot6DofSystemHardware"), "Could NOT Connect to Robot");
		exit(EXIT_FAILURE); // HACK FOR NOW
		return return_type::ERROR; // TODO: this does not seem to stop anything, and the code continues to read/write
	}







  // set some default values
  for (uint i = 0; i < hw_states_.size(); i++) {
    if (std::isnan(hw_states_[i])) {
      hw_states_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  status_ = hardware_interface_status::STARTED;
  RCLCPP_INFO(rclcpp::get_logger("Robot6DofSystemHardware"),
    "System Sucessfully started!");
  return return_type::OK;
}

return_type Robot6DofSystemHardware::stop()
{
  RCLCPP_INFO(rclcpp::get_logger("Robot6DofSystemHardware"),
    "Stopping ...please wait...");

  for (int i = 0; i <= hw_stop_sec_; i++) {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(rclcpp::get_logger("Robot6DofSystemHardware"),
      "%.1f seconds left...", hw_stop_sec_ - i);
  }


  	//EGM
    io_service_.stop();
    thread_group_.join_all();

  status_ = hardware_interface_status::STOPPED;

  RCLCPP_INFO(rclcpp::get_logger("Robot6DofSystemHardware"),
    "System sucessfully stopped!");

  return return_type::OK;
}

return_type Robot6DofSystemHardware::read_joints(
  std::vector<std::shared_ptr<Joint>> & joints) const
{
  if (joints.size() != hw_states_.size()) {
    // TODO(all): shoudl we return "wrong number of joints" error?
    return return_type::ERROR;
  }

  return_type ret = return_type::OK;

  RCLCPP_INFO(rclcpp::get_logger("Robot6DofSystemHardware"),
    "Reading...");

  // TODO(all): Should we check here joint names for the proper order?
  std::vector<double> values;
  values.resize(1);
  for (uint i = 0; i < joints.size(); i++) {
    values[0] = hw_states_[i];
    RCLCPP_INFO(rclcpp::get_logger("Robot6DofSystemHardware"),
      "Got state %.5f for joint %d!", values[0], i);
    ret = joints[i]->set_state(values);
    if (ret != return_type::OK) {
      break;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("Robot6DofSystemHardware"),
    "Joints sucessfully read!");
  return ret;
}

return_type Robot6DofSystemHardware::write_joints(
  const std::vector<std::shared_ptr<Joint>> & joints)
{
  if (joints.size() != hw_commands_.size()) {
    // TODO(all): return wrong number of joints
    return return_type::ERROR;
  }

  return_type ret = return_type::OK;

  RCLCPP_INFO(rclcpp::get_logger("Robot6DofSystemHardware"),
    "Writing...");

  // TODO(all): Should we check here the joint names for the proper order?
  std::vector<double> values;
  for (uint i = 0; i < joints.size(); i++) {
    ret = joints[i]->get_command(values);
    if (ret != return_type::OK) {
      break;
    }
    RCLCPP_INFO(rclcpp::get_logger("Robot6DofSystemHardware"),
      "Got command %.5f for joint %d!", values[0], i);
    // Simulate sending commands to the hardware
    hw_commands_[i] = values[0];
  }
  RCLCPP_INFO(rclcpp::get_logger("Robot6DofSystemHardware"),
    "Joints sucessfully written!");

  // TODO(denis): add this into separate timed loop
  for (uint i = 0; i < hw_states_.size(); i++) {
    // Simulate robot's movement
    hw_states_[i] = hw_commands_[i] + (hw_states_[i] - hw_commands_[i]) / hw_slowdown_;
  }

  return ret;
}

}  // namespace ros2_control_demo_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_driver::Robot6DofSystemHardware,
  hardware_interface::SystemHardwareInterface
)
