#ifndef AMR_HARDWARE__AMR_SYSTEM_HPP_
#define AMR_HARDWARE__AMR_SYSTEM_HPP_

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace amr_hardware
{

class AmrSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(AmrSystem)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  // 狀態
  double rear_wheel_vel_ = 0.0, rear_wheel_pos_ = 0.0;
  double front_steer_pos_ = 0.0, front_steer_vel_ = 0.0;

  // 指令
  double rear_wheel_cmd_ = 0.0;
  double front_steer_cmd_ = 0.0;

  // 控制狀態
  int current_gear_ = 3;
  bool cmd_received_ = false;
  int can_socket_ = -1;
  double integral_ = 0.0;
  double prev_error_ = 0.0;
  double measured_speed_ = 0.0;
  double prev_avg_cmd_vel_ = 0.0;

  // 分開前後進的 PID 狀態
  double integral_fwd_ = 0.0;
  double prev_error_fwd_ = 0.0;
  double integral_rev_ = 0.0;
  double prev_error_rev_ = 0.0;

  rclcpp::Logger logger_ = rclcpp::get_logger("AmrSystem");
};

}  // namespace amr_hardware

#endif  // AMR_HARDWARE__AMR_SYSTEM_HPP_

