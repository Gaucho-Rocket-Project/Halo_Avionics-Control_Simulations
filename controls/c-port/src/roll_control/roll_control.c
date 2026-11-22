#include "roll_control/roll.h"
#include <math.h>

// Copied from python implementation
RollCommand roll_step(const RollConfig *config, float roll_angle_rad,
                      float roll_rate_radps, RollCommand previous) {
  float inverse_acceleration = config->inertia_kgm2 / config->torque_nm;

  float stopping_time_s = inverse_acceleration * roll_rate_radps;

  float root = sqrtf(inverse_acceleration * fabsf(roll_angle_rad) +
                     .5f * stopping_time_s * stopping_time_s);

  int direction = (roll_angle_rad > 0.0f) - (roll_angle_rad < 0.0f);

  float switch_time_s = stopping_time_s * direction + root;
  float finish_time_s = stopping_time_s * direction + 2.f * root;

  RollCommand command = ROLL_OFF;

  float switching_error_rad =
      fabsf(roll_angle_rad + config->rate_weight_s * roll_rate_radps);

  if ((!previous && (switching_error_rad > config->on_threshold_rad)) ||
      (previous && (switching_error_rad >= config->off_threshold_rad))) {
    if (switch_time_s > 0.0f) {
      command = -direction;
    } else if (finish_time_s > 0.0f) {
      command = direction;
    }
  }

  return command;
}
