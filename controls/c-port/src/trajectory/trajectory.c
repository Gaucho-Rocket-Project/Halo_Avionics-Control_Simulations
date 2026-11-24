#include "trajectory/trajectory.h"

// Done in double: dz_ref is a 1 ms finite difference (as in the python), and
// float rounding of a ~50 m height would swamp it.

typedef struct {
  double H, V, Ti, Tf, Z;
  double x1, x2, x4, x5, x6, X, Y;
} Breakpoints;

static Breakpoints breakpoints(const TrajectoryConfig *c) {
  Breakpoints b = {.H = c->H, .V = c->V, .Ti = c->Ti, .Tf = c->Tf, .Z = c->Z};

  b.x1 = b.V / b.Z;
  b.x2 = (2.0 / b.V) * (b.H + b.V * b.x1 - 0.5 * b.Z * b.x1 * b.x1) - b.Ti;
  b.x4 = b.Ti + c->Th;
  b.x5 = b.x4 + (b.Ti - b.x2);
  b.X = b.V / (2.0 * (b.Ti - b.x2));
  b.x6 = (2.0 / b.V) *
             (b.H + b.V * b.x5 - b.X * (b.x5 - b.x4) * (b.x5 - b.x4)) -
         b.Tf;
  b.Y = b.V / (2.0 * (b.Tf - b.x6));
  return b;
}

// The python uses strict < on both sides of every breakpoint, so it returns 0
// exactly at a breakpoint. The pieces meet continuously, so the boundaries are
// just assigned to one side here.
static double z_ref(const Breakpoints *b, double t) {
  if (t < b->x1)
    return 0.5 * b->Z * t * t;
  if (t < b->x2)
    return b->V * (t - b->x1) + 0.5 * b->Z * b->x1 * b->x1;
  if (t < b->Ti)
    return -b->X * (t - b->Ti) * (t - b->Ti) + b->H;
  if (t < b->x4)
    return b->H;
  if (t < b->x5)
    return -b->X * (t - b->x4) * (t - b->x4) + b->H;
  if (t < b->x6)
    return -b->V * (t - b->x5) + b->H - b->X * (b->x5 - b->x4) * (b->x5 - b->x4);
  if (t < b->Tf)
    return b->Y * (t - b->Tf) * (t - b->Tf);
  return 0.0;
}

VehicleGoal trajectory_goal(const TrajectoryConfig *config, float t_s) {
  const Breakpoints b = breakpoints(config);
  const double t = t_s;
  const double z = z_ref(&b, t);

  VehicleGoal goal = {
      .height_m = (float)z,
      .vertical_velocity_mps = (float)((z_ref(&b, t + 0.001) - z) / 0.001),
      .shutdown = t_s > config->Tf,
  };
  return goal;
}
