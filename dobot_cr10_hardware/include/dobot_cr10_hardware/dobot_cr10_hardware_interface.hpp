#ifndef DOBOT_CR10_HARDWARE_INTERFACE_HPP_
#define DOBOT_CR10_HARDWARE_INTERFACE_HPP_

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <vector>
#include <string>
#include <mutex>
#include <iomanip>

namespace dobot_cr10_hardware
{
class DobotCR10HardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DobotCR10HardwareInterface)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool connect_tcp();
  void disconnect_tcp();
  std::string send_command_with_response(const std::string & cmd);
  bool parse_joint_angles(const std::string & response, std::vector<double> & angles);

  std::string robot_ip_ = "192.168.5.1";
  int robot_port_ = 29999;
  int socket_fd_ = -1;
  std::mutex comm_mutex_;

  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;
  std::vector<double> hw_prev_commands_;

  bool hw_connected_ = false;
  const size_t num_joints_ = 6;
};

}  // namespace dobot_cr10_hardware

#endif  // DOBOT_CR10_HARDWARE_INTERFACE_HPP_
