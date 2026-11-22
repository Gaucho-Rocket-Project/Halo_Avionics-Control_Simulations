#pragma once

#include <stdbool.h>

#include "kalman_filter/kalman.h"
#include "lqr/lqr.h"
#include "roll_control/roll.h"

typedef struct {
  float x_m;
  float y_m;
  float z_m;

  float vx_mps;
  float vy_mps;
  float vz_mps;

  // Euler angle representation of rocket angle
  float theta_rad;
  float theta_rate_radps;

  float phi_rad;
  float phi_rate_radps;

  float psi_rad;
  float psi_rate_radps;
} VehicleState;

typedef struct {
  float height_m;
  float vertical_velocity_mps;
  bool shutdown; // trajectory over (python: t > Tf): engine off, gimbal centered
} VehicleGoal;

typedef struct {
  float gimbal_x_rad; // original sims gx/gy
  float gimbal_y_rad;
  float thrust_reduction; // P
  RollCommand roll;       // f
} VehicleCommand;

// Everything LQR_control_algo() carries between ticks (python
// control_state_prev + u_prev). Estimates are kept in each plane's frame,
// i.e. already sign flipped the way the K matrix expects.
typedef struct {
  KfState xz_estimate;
  KfCovariance xz_covariance;

  KfState yz_estimate;
  KfCovariance yz_covariance;

  VehicleCommand command; // last command sent
} VehicleController;

// LQR only, on a state that's taken as already estimated. Roll is passed
// through from previous unchanged.
VehicleCommand vehicle_control_step(const VehicleState *state,
                                    const VehicleCommand *previous,
                                    const VehicleGoal *goal);

// Start the estimate at a known state with zero covariance, and the last
// command at hover throttle (same as halo_runner.py).
void vehicle_controller_init(VehicleController *controller,
                             const VehicleState *initial_state);

// Full control tick (python: Controls.LQR_control_algo): Kalman filter per
// plane, LQR per plane, roll control, shutdown. Call once every lqr_config.dt
// seconds; the Kalman model is discretized at that rate.
VehicleCommand vehicle_step(VehicleController *controller,
                            const VehicleState *measured,
                            const VehicleGoal *goal);
