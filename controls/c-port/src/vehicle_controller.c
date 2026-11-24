#include "vehicle_controller.h"
#include "kalman_filter/kalman.h"
#include "lqr/lqr.h"
#include "roll_control/roll.h"

_Static_assert(KF_STATE_COUNT == STATE_COUNT,
               "Kalman and LQR should share a state vector");
_Static_assert(KF_X == X_HAT && KF_Z == Z_HAT && KF_DX == DX_HAT &&
                   KF_DZ == DZ_HAT && KF_THETA == THETA_HAT &&
                   KF_DTHETA == DTHETA_HAT,
               "Kalman and LQR state order should match");

// K matrix expects this convention:
// XZ: [ 1, 1,  1, 1, -1, -1]
// YZ: [-1, 1, -1, 1, -1, -1]

static void xz_plane(const VehicleState *state, float out[STATE_COUNT]) {
  out[X_HAT] = state->x_m;
  out[Z_HAT] = state->z_m;
  out[DX_HAT] = state->vx_mps;
  out[DZ_HAT] = state->vz_mps;
  out[THETA_HAT] = -state->theta_rad;
  out[DTHETA_HAT] = -state->theta_rate_radps;
}

static void yz_plane(const VehicleState *state, float out[STATE_COUNT]) {
  out[X_HAT] = -state->y_m;
  out[Z_HAT] = state->z_m;
  out[DX_HAT] = -state->vy_mps;
  out[DZ_HAT] = state->vz_mps;
  out[THETA_HAT] = -state->phi_rad;
  out[DTHETA_HAT] = -state->phi_rate_radps;
}

static void plane_goal(const VehicleGoal *goal, float out[STATE_COUNT]) {
  out[X_HAT] = 0.0f;
  out[Z_HAT] = goal->height_m;
  out[DX_HAT] = 0.0f;
  out[DZ_HAT] = goal->vertical_velocity_mps;
  out[THETA_HAT] = 0.0f;
  out[DTHETA_HAT] = 0.0f;
}

// Runs the LQR in both planes on plane-frame states and combines them.
static VehicleCommand two_plane_lqr(const float xz_state[STATE_COUNT],
                                    const float yz_state[STATE_COUNT],
                                    const VehicleCommand *previous,
                                    const VehicleGoal *goal) {
  float target[STATE_COUNT];
  plane_goal(goal, target);

  // Gimbal around y axis
  float xz_output[CONTROL_COUNT];
  float xz_prev_output[CONTROL_COUNT] = {previous->gimbal_y_rad,
                                         previous->thrust_reduction};
  compute_lqr_step(&lqr_config, target, xz_state, xz_prev_output, xz_output);

  // Gimbal around x axis
  float yz_output[CONTROL_COUNT];
  float yz_prev_output[CONTROL_COUNT] = {previous->gimbal_x_rad,
                                         previous->thrust_reduction};
  compute_lqr_step(&lqr_config, target, yz_state, yz_prev_output, yz_output);

  VehicleCommand output = {
      .gimbal_x_rad = yz_output[GAMMA],
      .gimbal_y_rad = xz_output[GAMMA],
      .thrust_reduction = .5f * (yz_output[P] + xz_output[P]),
      .roll = previous->roll,
  };

  return output;
}

VehicleCommand vehicle_control_step(const VehicleState *state,
                                    const VehicleCommand *previous,
                                    const VehicleGoal *goal) {
  float xz_state[STATE_COUNT];
  float yz_state[STATE_COUNT];
  xz_plane(state, xz_state);
  yz_plane(state, yz_state);

  return two_plane_lqr(xz_state, yz_state, previous, goal);
}

void vehicle_controller_init(VehicleController *controller,
                             const VehicleState *initial_state) {
  *controller = (VehicleController){
      .command = {.thrust_reduction = lqr_config.trim[P], .roll = ROLL_OFF},
  };
  xz_plane(initial_state, controller->xz_estimate);
  yz_plane(initial_state, controller->yz_estimate);
}

VehicleCommand vehicle_step(VehicleController *controller,
                            const VehicleState *measured,
                            const VehicleGoal *goal) {
  const VehicleCommand previous = controller->command;

  // Measurement
  KfState xz_observed;
  KfState yz_observed;
  xz_plane(measured, xz_observed);
  yz_plane(measured, yz_observed);

  // Kalman filter, driven by the last command (as deviation from hover)
  float p_from_hover = previous.thrust_reduction - lqr_config.trim[P];

  KfInput xz_input = {[KF_GIMBAL] = previous.gimbal_y_rad,
                      [KF_P] = p_from_hover};
  kalman_step(xz_observed, controller->xz_estimate, controller->xz_covariance,
              xz_input, &kalman_config);

  KfInput yz_input = {[KF_GIMBAL] = previous.gimbal_x_rad,
                      [KF_P] = p_from_hover};
  kalman_step(yz_observed, controller->yz_estimate, controller->yz_covariance,
              yz_input, &kalman_config);

  // LQR on the estimates
  VehicleCommand output = two_plane_lqr(
      controller->xz_estimate, controller->yz_estimate, &previous, goal);

  // Roll control (uses the raw measurement, not filtered)
  output.roll = roll_step(&roll_config, measured->psi_rad,
                          measured->psi_rate_radps, previous.roll);

  if (goal->shutdown) {
    output = (VehicleCommand){
        .gimbal_x_rad = 0.0f,
        .gimbal_y_rad = 0.0f,
        .thrust_reduction = 1.0f, // thrust = T * (1 - 1) = 0
        .roll = ROLL_OFF,
    };
  }

  controller->command = output;
  return output;
}
