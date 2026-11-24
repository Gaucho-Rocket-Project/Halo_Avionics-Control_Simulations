#pragma once

#include "vehicle_controller.h"

// Altitude reference profile (python: Controls.z_ref / dz_ref).
// Climb at Z up to V, cruise, ease into a hover at H from Ti to Ti + Th,
// descend at V, and flare to touch down at Tf.
typedef struct {
  float H;  // hover height, m
  float V;  // climb/descent speed, m/s
  float Ti; // hover start, s
  float Th; // hover duration, s
  float Tf; // touchdown time, s (shutdown after this)
  float Z;  // initial climb acceleration, m/s^2
} TrajectoryConfig;

extern const TrajectoryConfig trajectory_config;

// Reference height and vertical speed at time t_s since launch.
// Sets shutdown once t_s > Tf.
VehicleGoal trajectory_goal(const TrajectoryConfig *config, float t_s);
