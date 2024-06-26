/**
 * @file state_machine.cpp
 * @author Haoru Xue (haoru.xue@autoware.org)
 * @brief
 * @version 0.1
 * @date 2022-02-10
 *
 * @copyright Copyright 2022 Triton AI
 *
 */

#include "state_machine.hpp"
#include <iostream>
/**
 * @brief GkcStateMachine constructor
 * Constructs a GkcStateMachine object, which has the methods for transitioning
 * between states of the GkcLifecycle.
 */
namespace tritonai {
namespace gkc {
GkcStateMachine::GkcStateMachine() : state_(GkcLifecycle::Uninitialized) {
  common_checks();
}

/**
 * @brief Transitions the state machine to the initializing state if possible.
 * Has to check if the current state is uninitialized, if not, it fails.
 * If the current state is uninitialized, sets the state to initializing, calls
 * on_initialize, and sets the state to inactive initialization is successful.
 * @return StateTransitionResult
 */
StateTransitionResult GkcStateMachine::initialize() {
  // Checks that the current state is uninitialized
  if (state_ != GkcLifecycle::Uninitialized) {
    return StateTransitionResult::FAILURE_INVALID_TRANSITION;
  }
  // Sets the state to initializing and calls on_initialize
  state_ = GkcLifecycle::Initializing;
  const auto result = on_initialize(state_);
  switch (result)
  {
  case StateTransitionResult::SUCCESS:
    state_ = GkcLifecycle::Inactive;
    break;
  case StateTransitionResult::EMERGENCY_STOP:
    state_ = GkcLifecycle::Emergency;
    break;
  default:
    state_ = GkcLifecycle::Uninitialized;
    break;
  }
  common_checks();
  return result;
}

/**
 * @brief Transitions the state machine to the inactive state if possible.
 * Has to check if the current state is active, if not, it fails.
 * If the current state is active, calls on_deactivate, and sets the state to
 * inactive if deactivation is successful. If deactivation is not successful,
 * it calls the emergency_stop function and changes states.
 * @return StateTransitionResult
 */

StateTransitionResult GkcStateMachine::deactivate() {
  // Checks that the current state is active
  if (state_ != GkcLifecycle::Active) {
    return StateTransitionResult::FAILURE_INVALID_TRANSITION;
  }
  // Calls on_deactivate and sets state to inactive if transition is successful
  const auto result = on_deactivate(state_);
  switch (result) {
  case StateTransitionResult::SUCCESS:
    state_ = GkcLifecycle::Inactive;
    break;
  case StateTransitionResult::EMERGENCY_STOP:
    state_ = GkcLifecycle::Emergency;
    break;
  default:
    state_ = GkcLifecycle::Active;
    break;
  }
  common_checks();
  return result;
}

/**
 * @brief Transitions the state machine to the active state if possible.
 * Has to check if the current state is inactive, if not, it fails.
 * If the current state is inactive, calls on_activate, and sets the state to
 * active if activation is successful. If activation is not successful, it
 * calls the emergency_stop function and changes states.
 * @return StateTransitionResult
 */

StateTransitionResult GkcStateMachine::activate() {
  // Checks that the current state is inactive
  if (state_ != GkcLifecycle::Inactive) {
    return StateTransitionResult::FAILURE_INVALID_TRANSITION;
  }
  // Calls on_activate and sets state to active if transition is successful
  const auto result = on_activate(state_);
  switch (result) {
  case StateTransitionResult::SUCCESS:
    state_ = GkcLifecycle::Active;
    break;
  case StateTransitionResult::EMERGENCY_STOP:
    state_ = GkcLifecycle::Emergency;
    break;
  default:
    state_ = GkcLifecycle::Inactive;
    break;
  }
  common_checks();
  return result;
}

/**
 * @brief Transitions the state machine to the emergency state.
 * Checks whether the current state is not uninitialized or initializing,
 * if it is, it fails. Otherwise, it calls on_emergency_stop and sets the state
 * to emergency. If the transition fails, it resets the MCU.
 *
 * @return StateTransitionResult
 */
StateTransitionResult GkcStateMachine::emergency_stop() {
  if (state_ == GkcLifecycle::Uninitialized) {
    return StateTransitionResult::FAILURE_INVALID_TRANSITION;
  }
  // Checks that the current state is not uninitialized or initializing
  if (state_  != GkcLifecycle::Emergency) {
    state_ = GkcLifecycle::Emergency;
    return emergency_stop();
  }
  // Calls on_emergency_stop and checks the result
  const auto result = on_emergency_stop(state_);
  switch (result) {
  case StateTransitionResult::SUCCESS:
    state_ = GkcLifecycle::Inactive;
    break;
  case StateTransitionResult::ERROR:
    // std::cout << "Resetting MCU for Estop Failure";
    NVIC_SystemReset();
  default:
    state_ = GkcLifecycle::Emergency;
    break;
  }
  common_checks();
  return result;
}

/**
 * @brief Transitions the state machine to the uninitialized state if possible.
 * Checks whether the current state is emergency, if it is, it fails. Otherwise,
 * it calls on_reinitialize and sets the state to uninitialized if the
 * transition is successful.
 *
 * @return StateTransitionResult
 */
StateTransitionResult GkcStateMachine::reinitialize() {
  // Checks that the current state is emergency
  if (state_ != GkcLifecycle::Uninitialized) {
    return StateTransitionResult::FAILURE_INVALID_TRANSITION;
  }
  state_ = GkcLifecycle::Initializing;
  // Calls on_reinitialize and sets state to uninitialized if transition is
  const auto result = on_reinitialize(state_);
  switch (result) {
  case StateTransitionResult::SUCCESS:
    state_ = GkcLifecycle::Inactive;
    break;
  default:
    state_ = GkcLifecycle::Uninitialized;
    break;
  }
  common_checks();
  return result;
}

GkcLifecycle GkcStateMachine::get_state() const { return state_; }

void GkcStateMachine::common_checks()
{
  if(state_ == GkcLifecycle::Active)
    _led = 0;
  else
    _led = 1;

  return;
}
} // namespace gkc
} // namespace tritonai
