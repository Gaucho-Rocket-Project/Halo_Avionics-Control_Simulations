#include "lqr/lqr.h"

const LQRConfig lqr_config = {

    // Gain matrix: X, Z, dX, dZ, theta, dTheta (x2)
    // Gains generated from python file
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

    // P_hover = 1 - m0 * g / T = 1 - 285 * 9.8 / 3000
    .trim = {[GAMMA] = 0.0f, [P] = 1.0f - 285.0f * 9.8f / 3000.0f},

    // From python
    .dt = (1.f / 15.f),

    // Original simulator limits; confirm against the actual actuators.
    .gamma_limits =
        {
            .min = -0.22f,          // rad
            .max = 0.22f,           // rad
            .max_rate_per_s = 0.5f, // rad/s
        },

    .p_limits = {
        .min = 0.0f,
        .max = 0.4f,            // thrust = T * (1 - P)
        .max_rate_per_s = 0.5f, // fraction/s
    }};
