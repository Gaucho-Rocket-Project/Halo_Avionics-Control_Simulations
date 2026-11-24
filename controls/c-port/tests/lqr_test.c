// AI written tests file, I'll look more through this later myself

/* Desktop checks for the single-plane LQR implementation.
 * Build from flight-control/:
 * cc -std=c11 -Wall -Wextra -Wpedantic desktop/main.c lqr/lqr.c \
 *    lqr/lqr_config.c -lm -o desktop/lqr_check
 *
 * Feedback checks use the supplied Python-generated gains.
 * Limit checks use separate synthetic gains to isolate limiting behavior.
 */
#include <math.h>
#include <stdio.h>

#include "lqr/lqr.h"

static int failures = 0;
static int checks = 0;

static void check_close(const char *name, float actual, float expected) {
  checks++;
  if (!isfinite(actual) || fabsf(actual - expected) > 1e-6f) {
    printf("FAIL %-32s got % .7f, expected % .7f\n", name, actual, expected);
    failures++;
  } else {
    printf("PASS %-32s % .7f\n", name, actual);
  }
}

static void check_feedback(void) {
  /* Wide limits isolate -K * (state - target) from limiting behavior.
   * Gains were generated using RocketSim.py's default configuration.
   */
  const LQRConfig config = {
      .K =
          {
              [GAMMA] =
                  {
                      -7.0710678119e-03f,
                      -9.6187139127e-19f,
                      -2.3728626707e-02f,
                      6.2161582696e-18f,
                      3.5552393520e-01f,
                      2.6485428503e-01f,
                  },
              [P] =
                  {
                      -5.5927676097e-19f,
                      -4.7735163489e-01f,
                      -1.0232161620e-18f,
                      -5.4683648741e-01f,
                      8.9901224699e-18f,
                      -7.2299453409e-18f,
                  },
          },
      .dt = 0.1f,
      .gamma_limits = {-100.0f, 100.0f, 10000.0f},
      .p_limits = {-100.0f, 100.0f, 10000.0f},
  };
  const float target[STATE_COUNT] = {1, 2, 3, 4, 5, 6};
  const float previous[CONTROL_COUNT] = {0};
  float output[CONTROL_COUNT];

  compute_lqr_step(&config, target, target, previous, output);
  check_close("zero error: gamma", output[GAMMA], 0.0f);
  check_close("zero error: P", output[P], 0.0f);

  /* One nonzero error at a time: each output must be -K[row][column]. */
  for (int column = 0; column < STATE_COUNT; column++) {
    float state[STATE_COUNT];
    for (int j = 0; j < STATE_COUNT; j++) {
      state[j] = target[j];
    }
    state[column] += 1.0f;
    compute_lqr_step(&config, target, state, previous, output);

    char name[64];
    snprintf(name, sizeof(name), "column %d: gamma", column);
    check_close(name, output[GAMMA], -config.K[GAMMA][column]);
    snprintf(name, sizeof(name), "column %d: P", column);
    check_close(name, output[P], -config.K[P][column]);
  }

  /* Error = [1, -1, 2, -2, 3, -3]. Hand-calculated dot products. */
  const float state[STATE_COUNT] = {2, 1, 5, 2, 8, 3};
  compute_lqr_step(&config, target, state, previous, output);
  check_close("combined errors: gamma", output[GAMMA], -0.2174806292841f);
  check_close("combined errors: P", output[P], -1.57102460971f);
}

static void check_limits(void) {
  const LQRConfig config = {
      .K =
          {
              [GAMMA] = {[THETA_HAT] = 2.0f},
              [P] = {[Z_HAT] = -0.1f},
          },
      .dt = 0.1f,
      .gamma_limits = {-0.22f, 0.22f, 0.5f},
      .p_limits = {0.0f, 0.4f, 0.5f},
  };
  const float target[STATE_COUNT] = {0};
  /* Positive height error requests less thrust (positive P). */
  float state[STATE_COUNT] = {[Z_HAT] = 10.0f, [THETA_HAT] = 1.0f};
  float previous[CONTROL_COUNT] = {0};
  float output[CONTROL_COUNT];

  puts("\nRepeated ticks: raw requests are gamma = -2 rad and P = 1.");
  for (int tick = 0; tick < 10; tick++) {
    compute_lqr_step(&config, target, state, previous, output);
    printf("tick %2d: gamma = % .4f rad, P = %.4f\n", tick + 1, output[GAMMA],
           output[P]);
    if (tick == 0) {
      check_close("first tick: gamma slew", output[GAMMA], -0.05f);
      check_close("first tick: P slew", output[P], 0.05f);
    }
    /* Keep previous and output in separate arrays. */
    previous[GAMMA] = output[GAMMA];
    previous[P] = output[P];
  }
  check_close("gamma lower bound", output[GAMMA], -0.22f);
  check_close("P upper bound", output[P], 0.4f);

  /* Reverse the errors and allow enough ticks to reach opposite bounds. */
  state[THETA_HAT] = -1.0f;
  state[Z_HAT] = -10.0f;
  for (int tick = 0; tick < 10; tick++) {
    compute_lqr_step(&config, target, state, previous, output);
    previous[GAMMA] = output[GAMMA];
    previous[P] = output[P];
  }
  check_close("gamma upper bound", output[GAMMA], 0.22f);
  check_close("P lower bound", output[P], 0.0f);
}

int main(void) {
  check_feedback();
  check_limits();
  printf("\n%d checks, %d failures\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
