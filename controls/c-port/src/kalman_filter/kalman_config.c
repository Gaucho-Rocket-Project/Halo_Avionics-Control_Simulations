#include "kalman_filter/kalman.h"

// Generated from halo_controls.py Controls._build_model() with the
// tests/hash_tests/run.py BASE parameters (T = 3000, m0 = 285, g = 9.8,
// dNoz = 0.7, l = 2.5, control_frequency = 15). Same config as lqr_config.c.

#define KF_DT (1.f / 15.f)

const KalmanConfig kalman_config = {
    // Atd = expm([[Atc, Btc], [0, 0]] * dt)[:6, :6], rounded to 4 decimals
    .A =
        {
            {1.0f, 0.0f, 0.0667f, 0.0f, -0.0218f, -0.0005f},
            {0.0f, 1.0f, 0.0f, 0.0667f, 0.0f, 0.0f},
            {0.0f, 0.0f, 1.0f, 0.0f, -0.6533f, -0.0218f},
            {0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f},
            {0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0667f},
            {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f},
        },

    // Btd = expm(...)[:6, 6:], columns [gimbal, P - P_hover]
    .B =
        {
            {-0.0219f, 0.0f},
            {0.0f, -0.0234f},
            {-0.6597f, 0.0f},
            {0.0f, -0.7018f},
            {0.0293f, 0.0f},
            {0.8781f, 0.0f},
        },

    // Q_d = I + Q_kalman * dt, Q_kalman = diag(1e-10, 1e-10, 1e-5, 1e-5,
    // 1e-10, 1e-7). (The python adds the identity, so this is ~I.)
    .Q =
        {
            [KF_X] = {[KF_X] = 1.0f + 1e-10f * KF_DT},
            [KF_Z] = {[KF_Z] = 1.0f + 1e-10f * KF_DT},
            [KF_DX] = {[KF_DX] = 1.0f + 1e-5f * KF_DT},
            [KF_DZ] = {[KF_DZ] = 1.0f + 1e-5f * KF_DT},
            [KF_THETA] = {[KF_THETA] = 1.0f + 1e-10f * KF_DT},
            [KF_DTHETA] = {[KF_DTHETA] = 1.0f + 1e-7f * KF_DT},
        },

    // R_d = R_kalman / dt, R_kalman = diag(1e-5 * 6)
    .R =
        {
            [KF_X] = {[KF_X] = 1e-5f / KF_DT},
            [KF_Z] = {[KF_Z] = 1e-5f / KF_DT},
            [KF_DX] = {[KF_DX] = 1e-5f / KF_DT},
            [KF_DZ] = {[KF_DZ] = 1e-5f / KF_DT},
            [KF_THETA] = {[KF_THETA] = 1e-5f / KF_DT},
            [KF_DTHETA] = {[KF_DTHETA] = 1e-5f / KF_DT},
        },
};
