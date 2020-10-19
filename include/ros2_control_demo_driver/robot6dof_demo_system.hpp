#ifndef ROS2_CONTROL_DEMO_DRIVER__RRBOT_6DOF_DEMO_SYSTEM_HPP_
#define ROS2_CONTROL_DEMO_DRIVER__RRBOT_6DOF_DEMO_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>
#include "rclcpp/macros.hpp"

#include "hardware_interface/system_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "ros2_control_demo_driver/visibility_control.h"

#include <abb_libegm/egm_controller_interface.h>

using hardware_interface::components::Joint;
using hardware_interface::components::Sensor;
using hardware_interface::HardwareInfo;
using hardware_interface::hardware_interface_status;
using hardware_interface::return_type;

namespace hardware_interface
{

// TODO(all): This could be templated for Joint, Sensor and System Interface
class BaseSystemHardwareInterface : public SystemHardwareInterface
{
public:
  return_type configure(const HardwareInfo & system_info) override
  {
    info_ = system_info;
    status_ = hardware_interface_status::CONFIGURED;
    return return_type::OK;
  }

  std::string get_name() const final
  {
    return info_.name;
  }

  hardware_interface_status get_status() const final
  {
    return status_;
  }

protected:
  HardwareInfo info_;
  hardware_interface_status status_;
};

}  // namespace hardware_interface

namespace ros2_control_demo_driver
{
class Robot6DofSystemHardware : public hardware_interface::BaseSystemHardwareInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Robot6DofSystemHardware);

  return_type configure(const HardwareInfo & system_info) override;

  return_type start() override;

  return_type stop() override;

  // TODO(all): Add a new "sensor not exitst" error?
  return_type read_sensors(std::vector<std::shared_ptr<Sensor>> & /*sensors*/) const override
  {
    return return_type::ERROR;
  }

  return_type read_joints(std::vector<std::shared_ptr<Joint>> & joints) const override;

  return_type write_joints(const std::vector<std::shared_ptr<Joint>> & joints) override;

private:
  // Dummy parameters
  double hw_start_sec_, hw_stop_sec_, hw_slowdown_;
  // Store the command for the simulated robot
  std::vector<double> hw_commands_, hw_states_;


  // lib_eng things
  boost::asio::io_service io_service_;
  boost::thread_group thread_group_; //NOTE
  //abb::egm::EGMControllerInterface *egm_interface_; //(io_service, 6511);
  std::unique_ptr<abb::egm::EGMControllerInterface> egm_interface_;

};

}  // namespace ros2_control_demo_hardware

#endif  // ROS2_CONTROL_DEMO_HARDWARE__RRBOT_SYSTEM_POSITION_ONLY_HPP_
