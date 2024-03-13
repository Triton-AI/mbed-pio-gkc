#include "Actuation/actuation_controller.hpp"
#include "Tools/logger.hpp"
#include "Actuation/vesc_can_tools.hpp"

namespace tritonai::gkc
{
  ActuationController::ActuationController(ILogger *logger) : logger(logger)
  {
  }

  void ActuationController::set_throttle_cmd(float cmd)
  {
    comm_can_set_speed(cmd);
  }

  void ActuationController::set_steering_cmd(float cmd)
  {
    logger->send_log(LogPacket::Severity::BEBUG, "Setting steering to " + std::to_string(cmd));
  }

  void ActuationController::set_brake_cmd(float cmd)
  {
    logger->send_log(LogPacket::Severity::BEBUG, "Setting brake to " + std::to_string(cmd));
  }
} // namespace tritonai::gkc