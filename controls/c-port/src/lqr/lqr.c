#include "lqr/lqr.h"
#include <stdio.h>

void compute_lqr_step(const LQRConfig *config, const float target[STATE_COUNT],
                      const float state[STATE_COUNT],
                      const float previous[CONTROL_COUNT],
                      float output[CONTROL_COUNT]) {
  // Array with control limits
  const ControlLimits *limits[CONTROL_COUNT] = {[GAMMA] = &config->gamma_limits,
                                                [P] = &config->p_limits};

  for (int i = 0; i < CONTROL_COUNT; i++) {
    output[i] = config->trim[i];

    for (int j = 0; j < STATE_COUNT; j++) {
      float error = state[j] - target[j];
      output[i] -= config->K[i][j] * error;
    }

    // slew rate limit, then clamp to limits.
    float max_change = limits[i]->max_rate_per_s * config->dt;
    float change = clampf(output[i] - previous[i], -max_change, max_change);

    output[i] = clampf(previous[i] + change, limits[i]->min, limits[i]->max);
  }
}
