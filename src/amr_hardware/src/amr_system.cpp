#include "amr_hardware/amr_system.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <cmath>
#include <cstdio>

namespace amr_hardware
{

hardware_interface::CallbackReturn AmrSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;

  can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  struct ifreq ifr{};
  std::strncpy(ifr.ifr_name, "can0", IFNAMSIZ);
  ioctl(can_socket_, SIOCGIFINDEX, &ifr);

  struct sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  bind(can_socket_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr));

  int flags = fcntl(can_socket_, F_GETFL, 0);
  fcntl(can_socket_, F_SETFL, flags | O_NONBLOCK);

  rear_wheel_pos_ = rear_wheel_vel_ = rear_wheel_cmd_ = 0.0;
  front_steer_pos_ = front_steer_vel_ = front_steer_cmd_ = 0.0;

  integral_fwd_ = prev_error_fwd_ = 0.0;
  integral_rev_ = prev_error_rev_ = 0.0;
  measured_speed_ = 0.0;
  current_gear_ = 8;
  cmd_received_ = false;

  RCLCPP_INFO(logger_, "系統初始化完成");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> AmrSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> states;
  states.emplace_back("rear_wheel_joint",  hardware_interface::HW_IF_POSITION, &rear_wheel_pos_);
  states.emplace_back("rear_wheel_joint",  hardware_interface::HW_IF_VELOCITY, &rear_wheel_vel_);
  states.emplace_back("front_steer_joint", hardware_interface::HW_IF_POSITION, &front_steer_pos_);
  states.emplace_back("front_steer_joint", hardware_interface::HW_IF_VELOCITY, &front_steer_vel_);
  return states;
}

std::vector<hardware_interface::CommandInterface> AmrSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> cmds;
  cmds.emplace_back("rear_wheel_joint", hardware_interface::HW_IF_VELOCITY, &rear_wheel_cmd_);
  cmds.emplace_back("front_steer_joint", hardware_interface::HW_IF_POSITION, &front_steer_cmd_);
  return cmds;
}

hardware_interface::return_type AmrSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  struct can_frame frame{};
  constexpr double dt = 0.01;

  while (::read(can_socket_, &frame, sizeof(frame)) > 0)
  {
    if (frame.can_id == 0x101 && frame.can_dlc == 8)
    {
      uint32_t val = (frame.data[7]<<24)|(frame.data[6]<<16)|(frame.data[5]<<8)|frame.data[4];
      double mps = val / 1000.0 / 3600.0;
      double wheel_radius = 0.16485;
      double sign = (current_gear_ == 2) ? -1.0 : 1.0;

      rear_wheel_vel_ = sign * mps / wheel_radius;
      rear_wheel_pos_ += rear_wheel_vel_ * dt;
      front_steer_vel_ = rear_wheel_vel_;

      measured_speed_ = sign * mps;
    }
    else if (frame.can_id == 0x067 && frame.can_dlc >= 4)
    {
      uint16_t raw = (frame.data[3]<<8) | frame.data[2];
      float deg = (static_cast<int>(raw) - 9000) / 10.0f;
      front_steer_pos_ = deg * static_cast<float>(M_PI / 180.0f);
    }
    else if (frame.can_id == 0x0A3 && frame.can_dlc >= 1)
    {
      current_gear_ = frame.data[0];
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AmrSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (std::abs(rear_wheel_cmd_) > 1e-3 || std::abs(front_steer_cmd_) > 1e-3)
    cmd_received_ = true;

  struct can_frame gear_tx{};
  gear_tx.can_id = 0x168;
  gear_tx.can_dlc = 8;
  std::memset(gear_tx.data, 0, 8);
  if (!cmd_received_ || std::abs(rear_wheel_cmd_) < 1e-3)
    gear_tx.data[0] = 8;
  else if (rear_wheel_cmd_ > 0)
    gear_tx.data[0] = 1;
  else
    gear_tx.data[0] = 2;
  (void)::write(can_socket_, &gear_tx, sizeof(gear_tx));

  constexpr double Kp_fwd = 12.0, Ki_fwd = 0.1, Kd_fwd = 0.0;
  constexpr double Kp_rev = 20.0, Ki_rev = 0.2, Kd_rev = 0.0;
  constexpr double dt = 0.05;

  double error = std::abs(rear_wheel_cmd_) - std::abs(measured_speed_);
  double output = 0.0;
  static bool last_cmd_positive = true;

  if (!cmd_received_ || std::abs(rear_wheel_cmd_) < 1e-3) {
    integral_fwd_ = prev_error_fwd_ = 0.0;
    integral_rev_ = prev_error_rev_ = 0.0;
  } else if (rear_wheel_cmd_ > 0) {
    if (!last_cmd_positive) {
      integral_fwd_ = prev_error_fwd_ = 0.0;
    }
    integral_fwd_ += error * dt;
    double derivative = (error - prev_error_fwd_) / dt;
    prev_error_fwd_ = error;
    output = Kp_fwd * error + Ki_fwd * integral_fwd_ + Kd_fwd * derivative;
    last_cmd_positive = true;
  } else {
    if (last_cmd_positive) {
      integral_rev_ = prev_error_rev_ = 0.0;
    }
    integral_rev_ += error * dt;
    double derivative = (error - prev_error_rev_) / dt;
    prev_error_rev_ = error;
    output = Kp_rev * error + Ki_rev * integral_rev_ + Kd_rev * derivative;
    last_cmd_positive = false;
  }

  uint8_t throttle_val = 0;
  if (!cmd_received_ || std::abs(rear_wheel_cmd_) < 1e-3) {
    throttle_val = 0;
  } else if (rear_wheel_cmd_ > 0) {
    throttle_val = static_cast<uint8_t>(std::clamp(output, 20.0, 100.0));
  } else {
    throttle_val = static_cast<uint8_t>(std::clamp(output, 30.0, 100.0));
  }

  struct can_frame thr_tx{};
  thr_tx.can_id = 0x075;
  thr_tx.can_dlc = 8;
  std::memset(thr_tx.data, 0, 8);
  thr_tx.data[0] = throttle_val;
  (void)::write(can_socket_, &thr_tx, sizeof(thr_tx));

  float steer_deg = front_steer_cmd_ * 180.0f / static_cast<float>(M_PI);
  uint16_t steer_raw = static_cast<uint16_t>(9000 + steer_deg * 10.0f);
  struct can_frame st_tx{};
  st_tx.can_id = 0x065;
  st_tx.can_dlc = 8;
  std::memset(st_tx.data, 0, 8);
  st_tx.data[0] = steer_raw & 0xFF;
  st_tx.data[1] = (steer_raw >> 8) & 0xFF;
  (void)::write(can_socket_, &st_tx, sizeof(st_tx));

  struct can_frame lt_tx{};
  lt_tx.can_id = 0x43F;
  lt_tx.can_dlc = 8;
  std::memset(lt_tx.data, 0, 8);
  (void)::write(can_socket_, &lt_tx, sizeof(lt_tx));

  double cmd_kmh = rear_wheel_cmd_ * 0.16485 * 3.6;
  double read_kmh = rear_wheel_vel_ * 0.16485 * 3.6;
  double steer_angle = front_steer_pos_ * 180.0 / M_PI;
  double cmd_steer_angle = front_steer_cmd_ * 180.0 / M_PI;
  const char* gear_str = (current_gear_ == 1) ? "D檔" : (current_gear_ == 2) ? "R檔" : (current_gear_ == 4) ? "N檔" : "P檔";
  RCLCPP_INFO(logger_, "指令速度: %.2f km/h | 指令轉向角: %.2f ° | 讀取轉向角: %.2f ° | 實際速度: %.2f km/h | 檔位: %s",
              cmd_kmh, cmd_steer_angle, steer_angle, read_kmh, gear_str);

  return hardware_interface::return_type::OK;
}

} // namespace amr_hardware

PLUGINLIB_EXPORT_CLASS(amr_hardware::AmrSystem, hardware_interface::SystemInterface)

