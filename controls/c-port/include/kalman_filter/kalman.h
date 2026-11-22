#pragma once

#include <stdbool.h>

// Same order as the LQR states (X_HAT ... DTHETA_HAT), in one plane's frame
enum { KF_X, KF_Z, KF_DX, KF_DZ, KF_THETA, KF_DTHETA, KF_STATE_COUNT };

typedef float KfState[KF_STATE_COUNT];
typedef float KfCovariance[KF_STATE_COUNT][KF_STATE_COUNT];

// Inputs are deviations from hover: [gimbal, P - P_hover]
enum { KF_GIMBAL, KF_P, KF_INPUT_COUNT };

typedef float KfInput[KF_INPUT_COUNT];

// Discrete-time model (python: Atd, Btd, Q_d, R_d), already discretized at
// the control rate. Regenerate if dt or the vehicle changes.
typedef struct {
  float A[KF_STATE_COUNT][KF_STATE_COUNT];
  float B[KF_STATE_COUNT][KF_INPUT_COUNT];
  KfCovariance Q; // process noise
  KfCovariance R; // measurement noise
} KalmanConfig;

extern const KalmanConfig kalman_config;

// One predict + update step (python: Controls.Kalman_Filter).
// kalman_state and kalman_covariance are read as the previous estimate and
// overwritten with the new one. u is the input applied over the last step.
// Returns false if the innovation covariance couldn't be inverted; the
// estimate is then left at the prediction (no measurement update).
bool kalman_step(const KfState observed_state, KfState kalman_state,
                 KfCovariance kalman_covariance, const KfInput u,
                 const KalmanConfig *config);
