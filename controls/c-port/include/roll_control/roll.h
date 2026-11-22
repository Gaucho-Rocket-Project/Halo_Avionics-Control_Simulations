#pragma once

typedef enum {
  ROLL_NEGATIVE = -1,
  ROLL_OFF = 0,
  ROLL_POSITIVE = 1
} RollCommand;

typedef struct {
  float inertia_kgm2;
  float torque_nm;
  float dt;

  float rate_weight_s;
  float on_threshold_rad;
  float off_threshold_rad;
} RollConfig;

extern const RollConfig roll_config;

RollCommand roll_step(const RollConfig *config, float roll_angle_rad,
                      float roll_rate_radps, RollCommand previous);
