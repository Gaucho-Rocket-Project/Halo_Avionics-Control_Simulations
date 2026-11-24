/* Behavioral checks for the original Python roll algorithm's C port.
 * Synthetic inertia/torque give angular acceleration = 1 rad/s^2.
 * These tests establish algorithm behavior, not flight suitability.
 */
#include <math.h>
#include <stdio.h>

#include "roll_control/roll.h"

static int checks;
static int failures;

static const RollConfig config = {
    .inertia_kgm2 = 1.0f,
    .torque_nm = 1.0f,
    .dt = 1.0f / 15.0f,
    .rate_weight_s = 2.0f,
    .on_threshold_rad = 0.02f,
    .off_threshold_rad = 0.006f,
};

static void check(const char *name, const RollConfig *cfg, float angle,
                  float rate, RollCommand previous, RollCommand expected)
{
    RollCommand actual = roll_step(cfg, angle, rate, previous);
    checks++;
    if (actual != expected) {
        printf("FAIL %s: got %d, expected %d\n", name, (int)actual,
               (int)expected);
        failures++;
    } else {
        printf("PASS %s\n", name);
    }
}

static void check_direction_and_braking(void)
{
    check("positive angle at rest", &config, 0.1f, 0, ROLL_OFF, ROLL_NEGATIVE);
    check("negative angle at rest", &config, -0.1f, 0, ROLL_OFF, ROLL_POSITIVE);
    check("positive angle moving away", &config, 0.1f, 1, ROLL_OFF,
          ROLL_NEGATIVE);
    check("negative angle moving away", &config, -0.1f, -1, ROLL_OFF,
          ROLL_POSITIVE);
    check("slow return still corrects", &config, 0.1f, -0.1f, ROLL_OFF,
          ROLL_NEGATIVE);
    check("mirrored slow return", &config, -0.1f, 0.1f, ROLL_OFF,
          ROLL_POSITIVE);
    check("fast return brakes", &config, 0.1f, -1, ROLL_OFF, ROLL_POSITIVE);
    check("mirrored fast return brakes", &config, -0.1f, 1, ROLL_OFF,
          ROLL_NEGATIVE);

    /* ratio=1, angle=.5, rate=-1: root=1, switch time=0,
     * finish time=1. Exactly zero switch time must select braking.
     */
    check("zero switch time", &config, 0.5f, -1, ROLL_OFF, ROLL_POSITIVE);
    check("mirrored zero switch time", &config, -0.5f, 1, ROLL_OFF,
          ROLL_NEGATIVE);

    RollConfig stronger_thruster = config;
    stronger_thruster.torque_nm = 10.0f;
    check("torque changes switching decision", &stronger_thruster,
          0.5f, -1, ROLL_OFF, ROLL_NEGATIVE);
    RollConfig lower_inertia = config;
    lower_inertia.inertia_kgm2 = 0.1f;
    check("inertia changes switching decision", &lower_inertia,
          0.5f, -1, ROLL_OFF, ROLL_NEGATIVE);
}

static void check_hysteresis(void)
{
    /* Adjacent floats avoid arbitrary epsilons around strict boundaries. */
    float below_on = nextafterf(config.on_threshold_rad, 0.0f);
    float above_on = nextafterf(config.on_threshold_rad, INFINITY);
    float below_off = nextafterf(config.off_threshold_rad, 0.0f);
    float above_off = nextafterf(config.off_threshold_rad, INFINITY);

    check("below activation", &config, below_on, 0, ROLL_OFF, ROLL_OFF);
    check("at activation stays off", &config, config.on_threshold_rad, 0,
          ROLL_OFF, ROLL_OFF);
    check("above activation", &config, above_on, 0, ROLL_OFF, ROLL_NEGATIVE);
    check("negative activation boundary", &config, -config.on_threshold_rad, 0,
          ROLL_OFF, ROLL_OFF);
    check("negative above activation", &config, -above_on, 0, ROLL_OFF,
          ROLL_POSITIVE);
    check("below deactivation", &config, below_off, 0, ROLL_NEGATIVE, ROLL_OFF);
    check("at deactivation remains active", &config, config.off_threshold_rad,
          0, ROLL_NEGATIVE, ROLL_NEGATIVE);
    check("above deactivation", &config, above_off, 0, ROLL_NEGATIVE,
          ROLL_NEGATIVE);
    check("positive previous also remains active", &config,
          -config.off_threshold_rad, 0, ROLL_POSITIVE, ROLL_POSITIVE);

    check("deadband with previous off", &config, 0.01f, 0, ROLL_OFF, ROLL_OFF);
    check("deadband with previous active", &config, 0.01f, 0, ROLL_NEGATIVE,
          ROLL_NEGATIVE);
    check("opposite previous direction", &config, 0.01f, 0, ROLL_POSITIVE,
          ROLL_NEGATIVE);
    check("rate cancellation suppresses correction", &config, 0.1f, -0.05f,
          ROLL_NEGATIVE, ROLL_OFF);
}

static void check_zero_angle(void)
{
    check("zero state", &config, 0, 0, ROLL_OFF, ROLL_OFF);
    check("zero state turns off", &config, 0, 0, ROLL_POSITIVE, ROLL_OFF);
    /* Preserve the original algorithm's zero-angle behavior explicitly.
     * It returns OFF despite nonzero rate; this is not a claim that the
     * behavior provides adequate rotational damping.
     */
    check("zero angle positive rate", &config, 0, 1, ROLL_OFF, ROLL_OFF);
    check("zero angle negative rate", &config, 0, -1, ROLL_NEGATIVE, ROLL_OFF);
    check("negative zero angle", &config, -0.0f, 1, ROLL_POSITIVE, ROLL_OFF);
}

int main(void)
{
    check_direction_and_braking();
    check_hysteresis();
    check_zero_angle();
    printf("Roll controller: %d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
