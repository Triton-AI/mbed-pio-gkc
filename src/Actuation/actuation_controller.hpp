#ifndef ACTUATION_CONTROLLER_HPP_
#define ACTUATION_CONTROLLER_HPP_

#include "Mutex.h"
#include "Queue.h"
#include "Tools/logger.hpp"
#include "mbed.h"
#include "Sensor/sensor_reader.hpp"
#include <cstdint>

namespace tritonai::gkc {
class ActuationController {
public:
  explicit ActuationController(ILogger *logger);

  void set_throttle_cmd(float cmd);
  void set_steering_cmd(float cmd);
  void set_brake_cmd(float cmd);
  void estop();

  float clamp(float val) {
    if (val < -1.0f*THROTTLE_MAX_REVERSE_SPEED)
      return -1.0f*THROTTLE_MAX_REVERSE_SPEED;
    else if (val > THROTTLE_MAX_FORWARD_SPEED)
      return THROTTLE_MAX_FORWARD_SPEED;
    else
      return val;
  }

  ILogger *logger;

};
} // namespace gkc

#endif // ACTUATION_CONTROLLER_HPP_