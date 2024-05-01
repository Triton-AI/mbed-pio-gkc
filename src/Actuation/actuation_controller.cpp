#include "Actuation/actuation_controller.hpp"
#include "Tools/logger.hpp"
#include "Actuation/vesc_can_tools.hpp"
#include "config.hpp"
#include <algorithm>

namespace tritonai::gkc
{
  ActuationController::ActuationController(ILogger *logger) : logger(logger)
  {
  }

  void ActuationController::set_throttle_cmd(float cmd)
  {
    cmd = ActuationController::clamp(cmd);
    comm_can_set_speed(cmd);
  }

  void ActuationController::estop()
  {
    comm_can_set_current_brake_rel(THROTTLE_CAN_ID, 1.0);
  }

  void ActuationController::set_steering_cmd(float cmd)
  {
    comm_can_set_angle(cmd);
  }

  void ActuationController::set_brake_cmd(float cmd)
  {
    comm_can_set_brake_position(cmd);
  }
} // namespace tritonai::gkc