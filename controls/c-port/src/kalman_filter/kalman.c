#include "kalman_filter/kalman.h"
#include "matrix.h"

bool kalman_step(const KfState observed_state, KfState kalman_state,
                 KfCovariance kalman_covariance, const KfInput u,
                 const KalmanConfig *config) {
  // A cross x
  KfState ax;
  matmul(KF_STATE_COUNT, KF_STATE_COUNT, 1, &config->A[0][0], kalman_state, ax);

  // B cross u
  KfState bu;
  matmul(KF_STATE_COUNT, KF_INPUT_COUNT, 1, &config->B[0][0], u, bu);

  // Ax + Bu = apriori_state
  KfState apriori_state;
  matadd(KF_STATE_COUNT, 1, ax, bu, apriori_state);

  // A cross p
  KfCovariance ap;
  matmul(KF_STATE_COUNT, KF_STATE_COUNT, KF_STATE_COUNT, &config->A[0][0],
         &kalman_covariance[0][0], &ap[0][0]);

  // A_T
  KfCovariance a_t;
  transpose(KF_STATE_COUNT, KF_STATE_COUNT, &config->A[0][0], &a_t[0][0]);

  // AP cross A_T
  KfCovariance apa_t;
  matmul(KF_STATE_COUNT, KF_STATE_COUNT, KF_STATE_COUNT, &ap[0][0], &a_t[0][0],
         &apa_t[0][0]);

  // APA_T + Q = apriori_covariance
  KfCovariance apriori_covariance;
  matadd(KF_STATE_COUNT, KF_STATE_COUNT, &apa_t[0][0], &config->Q[0][0],
         &apriori_covariance[0][0]);

  KfCovariance s, s_inv, kalman_gain;

  // apriori_covariance + R_d
  matadd(KF_STATE_COUNT, KF_STATE_COUNT, &apriori_covariance[0][0],
         &config->R[0][0], &s[0][0]);

  // this is all to compute s^-1, might find a better way to do this in the
  // future
  KfCovariance work;
  if (!matinv(KF_STATE_COUNT, &s[0][0], &s_inv[0][0], &work[0][0])) {
    // Can't weigh the measurement; keep the prediction.
    for (int i = 0; i < KF_STATE_COUNT; i++) {
      kalman_state[i] = apriori_state[i];
      for (int j = 0; j < KF_STATE_COUNT; j++) {
        kalman_covariance[i][j] = apriori_covariance[i][j];
      }
    }
    return false;
  }

  // apriori_covariance cross s^-1 = kalman_gain
  matmul(KF_STATE_COUNT, KF_STATE_COUNT, KF_STATE_COUNT,
         &apriori_covariance[0][0], &s_inv[0][0], &kalman_gain[0][0]);

  // observed - apriori = innovation
  KfState innovation;
  matsub(KF_STATE_COUNT, 1, observed_state, apriori_state, innovation);

  // kalman_gain cross innovation = correction
  KfState correction;
  matmul(KF_STATE_COUNT, KF_STATE_COUNT, 1, &kalman_gain[0][0], innovation,
         correction);

  // apriori + correction = aposteriori_state (written back to kalman_state)
  matadd(KF_STATE_COUNT, 1, apriori_state, correction, kalman_state);

  // Joseph form covariance update, same as the python:
  // (I - K) P (I - K)_T + K R K_T

  // I - K
  KfCovariance eye, i_minus_k;
  identity(KF_STATE_COUNT, &eye[0][0]);
  matsub(KF_STATE_COUNT, KF_STATE_COUNT, &eye[0][0], &kalman_gain[0][0],
         &i_minus_k[0][0]);

  // (I - K)_T
  KfCovariance i_minus_k_t;
  transpose(KF_STATE_COUNT, KF_STATE_COUNT, &i_minus_k[0][0],
            &i_minus_k_t[0][0]);

  // (I - K) cross P cross (I - K)_T
  KfCovariance ikp, ikpik_t;
  matmul(KF_STATE_COUNT, KF_STATE_COUNT, KF_STATE_COUNT, &i_minus_k[0][0],
         &apriori_covariance[0][0], &ikp[0][0]);
  matmul(KF_STATE_COUNT, KF_STATE_COUNT, KF_STATE_COUNT, &ikp[0][0],
         &i_minus_k_t[0][0], &ikpik_t[0][0]);

  // K_T
  KfCovariance k_t;
  transpose(KF_STATE_COUNT, KF_STATE_COUNT, &kalman_gain[0][0], &k_t[0][0]);

  // K cross R cross K_T
  KfCovariance kr, krk_t;
  matmul(KF_STATE_COUNT, KF_STATE_COUNT, KF_STATE_COUNT, &kalman_gain[0][0],
         &config->R[0][0], &kr[0][0]);
  matmul(KF_STATE_COUNT, KF_STATE_COUNT, KF_STATE_COUNT, &kr[0][0], &k_t[0][0],
         &krk_t[0][0]);

  // Sum = aposteriori_covariance (written back to kalman_covariance)
  matadd(KF_STATE_COUNT, KF_STATE_COUNT, &ikpik_t[0][0], &krk_t[0][0],
         &kalman_covariance[0][0]);

  return true;
}
