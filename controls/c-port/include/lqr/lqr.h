#pragma once

// Trying to represent K matrix in a nice way

// From python file
enum {
  X_HAT,      // Lateral position (since x/y planes are calculated separately)
  Z_HAT,      // Height
  DX_HAT,     // Change in lateral position
  DZ_HAT,     // Change in height
  THETA_HAT,  // Tilt
  DTHETA_HAT, // Change in tilt
  STATE_COUNT
};

enum {
  GAMMA, // Gimbal - deflection angle (rad) in the xz or yz plane
  P,     // Thrust reduction - thrust = max_thrust * (1 - P)
  CONTROL_COUNT
};

_Static_assert(STATE_COUNT == 6, "LQR should have 6 states");
_Static_assert(CONTROL_COUNT == 2, "LQR should have 2 controls");

typedef struct {
  float min;
  float max;
  float max_rate_per_s;
} ControlLimits;

typedef struct {
  float K[CONTROL_COUNT][STATE_COUNT];

  // Operating point the model is linearized about:
  // output = trim - K * (state - target), before limits.
  // python: P = P_hover + delta_P, P_hover = 1 - m0 * g / T
  float trim[CONTROL_COUNT];

  float dt;

  ControlLimits gamma_limits;
  ControlLimits p_limits;
} LQRConfig;

extern const LQRConfig lqr_config;

void compute_lqr_step(const LQRConfig *config, const float target[STATE_COUNT],
                      const float state[STATE_COUNT],
                      const float previous[CONTROL_COUNT],
                      float output[CONTROL_COUNT]);

static inline float clampf(float value, float min, float max) {
  if (value < min)
    return min;
  if (value > max)
    return max;
  return value;
}
