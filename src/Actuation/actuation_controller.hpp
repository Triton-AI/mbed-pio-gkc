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

  ILogger *logger;

};
} // namespace gkc

#endif // ACTUATION_CONTROLLER_HPP_