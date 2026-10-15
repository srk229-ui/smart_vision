#include "control.h"
#include "pid.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static ControlParams params;
static uint32_t version;
static uint32_t frame;

static ControlSnapshot snapshot(void)
{
    ControlSnapshot s;
    control_get_snapshot(&s);
    return s;
}

static void setup(void)
{
    control_init(92.5f, 85.0f, 100.0f, -1.0f);
    control_set_output_enabled(1u, 1u);
    control_tick(0, 0);
    frame = 0u;
    control_get_params(&params, &version);
    params.target_left = 10.0f;
    params.target_right = 20.0f;
    params.calibration_confirmed = 1u;
    assert(control_apply_params(&params, version));
}

static void publish(float confidence, float error, uint8_t valid)
{
    VisionResult vision;
    memset(&vision, 0, sizeof(vision));
    vision.frame_id = ++frame;
    vision.capture_time_ms = control_now_ms();
    vision.confidence = confidence;
    vision.error_px = error;
    vision.valid = valid;
    vision.track_lost = !valid;
    control_publish_vision(&vision);
}

static void track_ready(void)
{
    publish(1.0f, 0.0f, 1u);
    control_tick(0, 0);
    publish(1.0f, 0.0f, 1u);
    control_tick(0, 0);
    publish(1.0f, 0.0f, 1u);
}

int main(void)
{
    ControlSnapshot s;
    VisionResult old;
    unsigned i;
    control_init(92.5f, 85.0f, 100.0f, -1.0f);
    control_set_output_enabled(1u, 1u);
    control_tick(0, 0);
    assert(snapshot().state == CONTROL_READY && snapshot().pwm_left == 0.0f);
    assert(!control_request_run(CONTROL_MODE_BENCH));

    setup();
    assert(!control_request_run(CONTROL_MODE_TRACK));
    track_ready();
    assert(control_request_run(CONTROL_MODE_TRACK));
    control_tick(0, 0);
    s = snapshot();
    assert(fabsf(s.target_left - 0.5f) < 0.0001f && s.pwm_left > 0.0f);
    control_get_params(&params, &version);
    assert(!control_apply_params(&params, version));
    assert(!control_request_run(CONTROL_MODE_TRACK));
    while (snapshot().state == CONTROL_RUNNING) control_tick(0, 0);
    s = snapshot();
    assert(s.state == CONTROL_FAULT && s.fault == CONTROL_FAULT_VISION_TIMEOUT);
    assert(s.pwm_left == 0.0f && s.pwm_right == 0.0f && !s.servo_enabled);
    assert(!control_reset_fault());
    control_tick(0, 0);
    assert(control_reset_fault());
    assert(!control_request_run(CONTROL_MODE_TRACK));

    setup(); track_ready(); assert(control_request_run(CONTROL_MODE_TRACK));
    publish(0.0f, 0.0f, 0u); publish(0.0f, 0.0f, 0u); publish(0.0f, 0.0f, 0u);
    control_tick(0, 0);
    assert(snapshot().fault == CONTROL_FAULT_TRACK_LOST);

    setup(); track_ready(); assert(control_request_run(CONTROL_MODE_TRACK));
    memset(&old, 0, sizeof(old)); old.frame_id = frame;
    old.capture_time_ms = control_now_ms(); old.confidence = 1.0f; old.valid = 1u;
    for (i = 0; i < 12u; ++i) { control_tick(0, 0); old.capture_time_ms = control_now_ms(); control_publish_vision(&old); }
    assert(snapshot().fault == CONTROL_FAULT_VISION_TIMEOUT);

    setup(); assert(control_request_run(CONTROL_MODE_BENCH));
    for (i = 0; i < 100u; ++i) control_tick(0, 0);
    s = snapshot(); assert(fabsf(s.target_left - 10.0f) < 0.0001f);
    assert(fabsf(s.target_right - 20.0f) < 0.0001f);
    control_request_stop(); s = snapshot();
    assert(s.pwm_left == 0.0f && s.target_left == 0.0f && speed_pid_l.integrator == 0.0f);
    assert(!control_request_run(CONTROL_MODE_BENCH));
    control_tick(0, 0); assert(control_request_run(CONTROL_MODE_BENCH));
    for (i = 0; i < 800u; ++i) control_tick(0, 0);
    assert(snapshot().state == CONTROL_READY && snapshot().pwm_left == 0.0f);

    setup(); assert(control_request_run(CONTROL_MODE_BENCH));
    control_report_link_fault();
    assert(snapshot().fault == CONTROL_FAULT_LINK && snapshot().pwm_left == 0.0f);
    control_tick(0, 0); assert(control_reset_fault());
    assert(snapshot().state == CONTROL_READY);

    setup(); control_get_params(&params, &version); params.kp_left = NAN;
    assert(!control_apply_params(&params, version));
    control_get_params(&params, &version); assert(!control_apply_params(&params, version - 1u));
    params.look_rows[0] = 120u; assert(!control_apply_params(&params, version));
    control_get_params(&params, &version); params.vision_timeout_ms = 5000u;
    assert(!control_apply_params(&params, version));

    setup(); track_ready(); assert(control_request_run(CONTROL_MODE_TRACK));
    publish(1.0f, 20.0f, 1u); control_tick(0, 0);
    assert(snapshot().servo_cmd > 92.5f && snapshot().servo_cmd <= 93.11f);
    for (i = 0; i < 40u; ++i) { publish(1.0f, 100.0f, 1u); control_tick(0, 0); }
    assert(snapshot().servo_cmd <= 100.0f);

    setup(); control_set_output_enabled(0u, 0u);
    assert(!control_request_run(CONTROL_MODE_BENCH));
    setup(); control_set_output_enabled(1u, 0u); track_ready();
    assert(!control_request_run(CONTROL_MODE_TRACK));
    assert(control_request_run(CONTROL_MODE_BENCH)); control_tick(0, 0);
    assert(!snapshot().servo_enabled);

    setup(); control_get_params(&params, &version);
    params.pwm_max = 10.5f; params.integral_max = 10.0f;
    assert(!control_apply_params(&params, version));
    params.pwm_max = 10.0f; params.kp_left = 0.0f; params.ki_left = 3.0f;
    params.target_left = 1.0f;
    assert(control_apply_params(&params, version)); assert(control_request_run(CONTROL_MODE_BENCH));
    for (i = 0; i < 400u; ++i) control_tick(0, 0);
    assert(snapshot().pwm_left == 10.0f && snapshot().saturated_left);

    setup(); track_ready(); control_get_params(&params, &version);
    params.look_weights[0] = 2.0f;
    assert(control_apply_params(&params, version));
    assert(!control_request_run(CONTROL_MODE_TRACK));
    track_ready(); assert(control_request_run(CONTROL_MODE_TRACK));

    puts("PASS test_control: startup, version, slope, vision, loss, BENCH, stop, fault, limits");
    return 0;
}
