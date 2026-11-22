/* Test the real vehicle wrapper with a deliberately synthetic configuration.
 * This executable supplies lqr_config instead of linking lqr_config.c.
 * Distinct gains make axis/sign errors and thrust averaging observable.
 */
#include <math.h>
#include <stdio.h>

#include "vehicle_controller.h"

const LQRConfig lqr_config = {
    .K = {
        [GAMMA] = {1, 0, 2, 0, 3, 4},
        [P] = {0.1f, -0.2f, 0, -0.3f, 0, 0},
    },
    .dt = 0.1f,
    .gamma_limits = {.min = -0.22f, .max = 0.22f, .max_rate_per_s = 0.5f},
    .p_limits = {.min = 0, .max = 0.4f, .max_rate_per_s = 0.5f},
};

static int checks;
static int failures;

static void check(const char *name, float actual, float expected)
{
    checks++;
    if (!isfinite(actual) || fabsf(actual - expected) > 1e-6f) {
        printf("FAIL %s: got %.7f, expected %.7f\n", name, actual, expected);
        failures++;
    }
}

static void check_command(const char *name, VehicleCommand actual,
                          float gx, float gy, float p)
{
    char label[100];
    snprintf(label, sizeof(label), "%s gx", name);
    check(label, actual.gimbal_x_rad, gx);
    snprintf(label, sizeof(label), "%s gy", name);
    check(label, actual.gimbal_y_rad, gy);
    snprintf(label, sizeof(label), "%s P", name);
    check(label, actual.thrust_reduction, p);
}

static void check_axis_mapping(void)
{
    const VehicleGoal goal = {0};
    const VehicleCommand previous = {0};

    /* Raw commands stay below the 0.05-per-tick slew limit.
     * Negative x gives positive gy and a 0.001 XZ thrust request.
     * The YZ thrust request is zero, so the applied average is 0.0005.
     */
    VehicleState state = {.x_m = -0.01f};
    check_command("x position", vehicle_control_step(&state, &previous, &goal),
                  0, 0.01f, 0.0005f);
    state = (VehicleState){.y_m = 0.01f};
    check_command("y position", vehicle_control_step(&state, &previous, &goal),
                  0.01f, 0, 0.0005f);

    state = (VehicleState){.vx_mps = 0.01f};
    check_command("x velocity", vehicle_control_step(&state, &previous, &goal),
                  0, -0.02f, 0);
    state = (VehicleState){.vy_mps = 0.01f};
    check_command("y velocity", vehicle_control_step(&state, &previous, &goal),
                  0.02f, 0, 0);

    state = (VehicleState){.theta_rad = 0.01f};
    check_command("theta", vehicle_control_step(&state, &previous, &goal),
                  0, 0.03f, 0);
    state = (VehicleState){.phi_rad = 0.01f};
    check_command("phi", vehicle_control_step(&state, &previous, &goal),
                  0.03f, 0, 0);
    state = (VehicleState){.theta_rate_radps = 0.01f};
    check_command("theta rate", vehicle_control_step(&state, &previous, &goal),
                  0, 0.04f, 0);
    state = (VehicleState){.phi_rate_radps = 0.01f};
    check_command("phi rate", vehicle_control_step(&state, &previous, &goal),
                  0.04f, 0, 0);

    /* Roll is intentionally not controlled by this wrapper. */
    state = (VehicleState){.psi_rad = 0.1f, .psi_rate_radps = 0.2f};
    check_command("roll ignored", vehicle_control_step(&state, &previous, &goal),
                  0, 0, 0);
}

static void check_vertical_reference(void)
{
    const VehicleCommand previous = {0};
    VehicleGoal goal = {.height_m = 10, .vertical_velocity_mps = 1};
    VehicleState state = {.z_m = 10, .vz_mps = 1};
    check_command("at reference", vehicle_control_step(&state, &previous, &goal),
                  0, 0, 0);
    state.z_m += 0.1f; /* -(-0.2 * 0.1) = 0.02 in both planes. */
    check_command("height error", vehicle_control_step(&state, &previous, &goal),
                  0, 0, 0.02f);
    state.z_m = goal.height_m;
    state.vz_mps += 0.1f;
    check_command("vertical speed", vehicle_control_step(&state, &previous, &goal),
                  0, 0, 0.03f);
}

static void check_history_and_limits(void)
{
    const VehicleGoal goal = {0};
    VehicleState state = {0};
    VehicleCommand previous = {
        .gimbal_x_rad = 0.15f, .gimbal_y_rad = -0.12f,
        .thrust_reduction = 0.2f,
    };
    check_command("return toward zero",
                  vehicle_control_step(&state, &previous, &goal),
                  0.10f, -0.07f, 0.15f);

    /* Distinct throttle requests verify each plane starts from the same
     * applied P: XZ requests 0.3, YZ requests 0.2, from previous P = 0.2.
     * Limited requests are 0.25 and 0.2, giving applied P = 0.225.
     */
    state = (VehicleState){.x_m = -1, .z_m = 1};
    previous = (VehicleCommand){.thrust_reduction = 0.2f};
    check_command("shared previous P",
                  vehicle_control_step(&state, &previous, &goal),
                  0, 0.05f, 0.225f);

    state = (VehicleState){.theta_rad = 1, .phi_rad = -1, .z_m = 10};
    previous = (VehicleCommand){0};
    const float expected_gamma[] = {0.05f, 0.10f, 0.15f, 0.20f, 0.22f,
                                    0.22f, 0.22f, 0.22f};
    const float expected_p[] = {0.05f, 0.10f, 0.15f, 0.20f,
                                0.25f, 0.30f, 0.35f, 0.40f};
    for (int tick = 0; tick < 8; tick++) {
        VehicleCommand output = vehicle_control_step(&state, &previous, &goal);
        char label[40];
        snprintf(label, sizeof(label), "sequence tick %d", tick + 1);
        check_command(label, output, -expected_gamma[tick],
                      expected_gamma[tick], expected_p[tick]);
        previous = output;
    }
}

int main(void)
{
    check_axis_mapping();
    check_vertical_reference();
    check_history_and_limits();
    printf("Vehicle controller: %d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
