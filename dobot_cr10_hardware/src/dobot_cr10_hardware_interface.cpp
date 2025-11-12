#include "dobot_cr10_hardware/dobot_cr10_hardware_interface.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <iostream>
#include <sstream>
#include <cstring>
#include <unistd.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <cmath>

namespace dobot_cr10_hardware
{

hardware_interface::CallbackReturn DobotCR10HardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  hw_positions_.resize(num_joints_, 0.0);
  hw_velocities_.resize(num_joints_, 0.0);
  hw_commands_.resize(num_joints_, 0.0);
  hw_prev_commands_.resize(num_joints_, 0.0);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DobotCR10HardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
  if (!connect_tcp())
  {
    std::cerr << "Failed to connect to Dobot CR10 at " << robot_ip_ << ":" << robot_port_ << "\n";
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  // Clear any pending data in socket buffer
  char dummy[1024];
  recv(socket_fd_, dummy, sizeof(dummy), MSG_DONTWAIT);
  
  // Enable the robot - try and continue even if response is unexpected
  std::string response = send_command_with_response("EnableRobot()");
  std::cout << "EnableRobot response: '" << response << "'\n";
  
  // Accept if successful (starts with 0,) or continue anyway (might already be enabled)
  if (!response.empty() && response.find("0,") == 0)
  {
    std::cout << "Robot enabled successfully\n";
  }
  else if (response.empty())
  {
    std::cout << "Warning: No response from EnableRobot, continuing anyway...\n";
  }
  else
  {
    std::cout << "Warning: Unexpected EnableRobot response, continuing anyway...\n";
  }
  
  std::cout << "Connected to Dobot CR10\n";
  hw_connected_ = true;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DobotCR10HardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  // Read initial joint positions
  std::string response = send_command_with_response("GetAngle()");
  std::cout << "GetAngle response: '" << response << "'\n";
  
  if (!parse_joint_angles(response, hw_positions_))
  {
    std::cerr << "Failed to read initial joint positions, using zeros\n";
    // Initialize with zeros instead of failing
    for (size_t i = 0; i < num_joints_; ++i)
    {
      hw_positions_[i] = 0.0;
    }
  }
  
  // Initialize commands with current positions
  hw_commands_ = hw_positions_;
  hw_prev_commands_ = hw_positions_;
  
  std::cout << "Dobot hardware activated\n";
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DobotCR10HardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Disable robot
  send_command_with_response("DisableRobot()");
  disconnect_tcp();
  std::cout << "Dobot hardware deactivated and disconnected\n";
  hw_connected_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DobotCR10HardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < num_joints_; ++i)
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        std::string("joint_") + std::to_string(i+1), 
        hardware_interface::HW_IF_POSITION, 
        &hw_positions_[i]));
    
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        std::string("joint_") + std::to_string(i+1), 
        hardware_interface::HW_IF_VELOCITY, 
        &hw_velocities_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DobotCR10HardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < num_joints_; ++i)
  {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        std::string("joint_") + std::to_string(i+1), 
        hardware_interface::HW_IF_POSITION, 
        &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::return_type DobotCR10HardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (!hw_connected_)
    return hardware_interface::return_type::ERROR;

  std::lock_guard<std::mutex> lock(comm_mutex_);

  // Read current joint angles using GetAngle()
  std::string response = send_command_with_response("GetAngle()");
  
  std::vector<double> new_positions(num_joints_);
  if (!parse_joint_angles(response, new_positions))
  {
    // Don't spam errors, just keep using old positions
    return hardware_interface::return_type::OK;
  }

  // Calculate velocities (simple finite difference)
  double dt = period.seconds();
  if (dt > 0.0)
  {
    for (size_t i = 0; i < num_joints_; ++i)
    {
      hw_velocities_[i] = (new_positions[i] - hw_positions_[i]) / dt;
    }
  }
  
  hw_positions_ = new_positions;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DobotCR10HardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!hw_connected_)
    return hardware_interface::return_type::ERROR;

  std::lock_guard<std::mutex> lock(comm_mutex_);

  // Check if commands have changed significantly
  bool commands_changed = false;
  for (size_t i = 0; i < num_joints_; ++i)
  {
    if (std::abs(hw_commands_[i] - hw_prev_commands_[i]) > 0.01)  // 0.01 radian threshold
    {
      commands_changed = true;
      break;
    }
  }

  if (!commands_changed)
  {
    return hardware_interface::return_type::OK;
  }

  // Send joint command using ServoJ
  // ServoJ format: ServoJ(j1,j2,j3,j4,j5,j6)
  std::ostringstream cmd;
  cmd << "ServoJ(" << std::fixed << std::setprecision(2);
  for (size_t i = 0; i < num_joints_; ++i)
  {
    // Convert radians to degrees
    double angle_deg = hw_commands_[i] * 180.0 / M_PI;
    cmd << angle_deg;
    if (i < num_joints_ - 1)
      cmd << ",";
  }
  cmd << ")";

  std::string response = send_command_with_response(cmd.str());
  
  // Check for errors in response - but don't fail, just warn
  if (response.empty())
  {
    // Command sent but no response - continue anyway
    hw_prev_commands_ = hw_commands_;
    return hardware_interface::return_type::OK;
  }
  
  if (response.find("0,") != 0)
  {
    // Non-zero error code, but continue
    hw_prev_commands_ = hw_commands_;
    return hardware_interface::return_type::OK;
  }

  hw_prev_commands_ = hw_commands_;
  return hardware_interface::return_type::OK;
}

bool DobotCR10HardwareInterface::connect_tcp()
{
  socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (socket_fd_ < 0)
  {
    perror("Socket creation failed");
    return false;
  }

  // Set socket timeout
  struct timeval timeout;
  timeout.tv_sec = 1;
  timeout.tv_usec = 0;
  if (setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0)
  {
    perror("Failed to set receive timeout");
  }
  if (setsockopt(socket_fd_, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout)) < 0)
  {
    perror("Failed to set send timeout");
  }

  sockaddr_in server_addr;
  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(robot_port_);

  if (inet_pton(AF_INET, robot_ip_.c_str(), &server_addr.sin_addr) <= 0)
  {
    perror("Invalid IP address");
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  if (connect(socket_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0)
  {
    perror("TCP connection failed");
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  std::cout << "Successfully connected to Dobot at " << robot_ip_ << ":" << robot_port_ << "\n";
  return true;
}

void DobotCR10HardwareInterface::disconnect_tcp()
{
  if (socket_fd_ >= 0)
  {
    close(socket_fd_);
    socket_fd_ = -1;
  }
}

std::string DobotCR10HardwareInterface::send_command_with_response(const std::string & cmd)
{
  if (socket_fd_ < 0)
    return "";

  // Send command
  ssize_t sent = send(socket_fd_, cmd.c_str(), cmd.size(), 0);
  if (sent != (ssize_t)cmd.size())
  {
    return "";
  }

  // Small delay to allow robot to process command
  usleep(50000);  // 50ms

  // Receive response
  char buffer[1024];
  memset(buffer, 0, sizeof(buffer));
  ssize_t received = recv(socket_fd_, buffer, sizeof(buffer)-1, 0);
  if (received <= 0)
  {
    return "";
  }

  buffer[received] = '\0';
  return std::string(buffer);
}

bool DobotCR10HardwareInterface::parse_joint_angles(const std::string & response, std::vector<double> & angles)
{
  // Response format: "0,{j1,j2,j3,j4,j5,j6},GetAngle();"
  // Extract values between { and }
  
  size_t start = response.find('{');
  size_t end = response.find('}');
  
  if (start == std::string::npos || end == std::string::npos || start >= end)
  {
    return false;
  }

  std::string values = response.substr(start + 1, end - start - 1);
  std::istringstream ss(values);
  std::string token;
  
  angles.clear();
  while (std::getline(ss, token, ','))
  {
    try
    {
      // Convert degrees to radians
      double angle_deg = std::stod(token);
      double angle_rad = angle_deg * M_PI / 180.0;
      angles.push_back(angle_rad);
    }
    catch (...)
    {
      return false;
    }
  }

  return angles.size() == num_joints_;
}

}  // namespace dobot_cr10_hardware

PLUGINLIB_EXPORT_CLASS(dobot_cr10_hardware::DobotCR10HardwareInterface, hardware_interface::SystemInterface)
