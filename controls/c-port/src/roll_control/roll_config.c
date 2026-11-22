#include "roll_control/roll.h"

// From halo_controls.py Roll_control() with the hash test BASE parameters.
const RollConfig roll_config = {
    .inertia_kgm2 = 0.5f * 285.0f * 0.2f * 0.2f, // Izz = 1/2 m0 r^2
    .torque_nm = 3.23f,                          // Tau0
    .dt = 1.f / 15.f,

    .rate_weight_s = 0.45f,     // k
    .on_threshold_rad = 0.01f,  // e_on
    .off_threshold_rad = 0.003f // e_off = e_on * 0.3
};
