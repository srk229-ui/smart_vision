#include "control.h"
#include "calibration_config.h"
#include "pid.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static ControlSnapshot snapshot(void)
{
    ControlSnapshot s;
    control_get_snapshot(&s);
    return s;
}

static void setup(void)
{
    control_init(92.5f, 85.0f, 100.0f, -1.0f);
    control_configure_servo_pwm(10000u, 50u);
    control_set_output_enabled(0u, 1u);
    control_tick(0, 0);
}

int main(void)
{
    ControlParams p;
    uint32_t version;
    ControlSnapshot s;
    unsigned i;
    setup();
    control_get_params(&p, &version);
    assert(!control_request_servo_cal(1526.0f, 1444.0f, 1610.0f, 500.0f, version));
    control_set_calibration_reserved(1u);
    assert(!control_apply_params(&p, version));
    assert(!control_request_run(CONTROL_MODE_BENCH));
#if CALIBRATION_DEBUG_ENABLE
    assert(!control_request_servo_cal(NAN, 1444.0f, 1610.0f, 500.0f, version));
    assert(!control_request_servo_cal(1526.0f, 500.0f, 2500.0f, 500.0f, version));
    assert(!control_request_servo_cal(1526.0f, 1444.0f, 1610.0f, 700.0f, version));
    assert(!control_request_servo_cal(1526.0f, 1444.0f, 1610.0f, 500.0f, version + 1u));
    assert(!control_request_servo_cal(1445.5f, 1445.0f, 1445.9f, 500.0f, version));
    assert(control_request_servo_cal(1446.0f, 1445.0f, 1447.0f, 500.0f, version));
    assert(snapshot().servo_pulse_us_nominal == 1446.0f); // 量化后的实际命令不得低于人工下限1445us。
    assert(control_set_servo_cal_pulse_us(1445.0f));
    control_tick(0, 0);
    assert(snapshot().servo_pulse_us_nominal >= 1445.0f);
    control_request_stop(); control_tick(0, 0);
    assert(control_request_servo_cal(1526.0f, 1444.0f, 1610.0f, 500.0f, version));
    assert(!control_request_calibration_bench());
    assert(!control_set_servo_cal_pulse_us(1700.0f));
    assert(control_set_servo_cal_pulse_us(1600.0f));
    for (i = 0u; i < 25u; ++i)
    {
        control_tick(12, 18);
        s = snapshot();
        assert(s.pwm_left == 0.0f && s.pwm_right == 0.0f);
        assert(s.integral_left == 0.0f && s.integral_right == 0.0f);
        assert(s.servo_enabled && s.servo_pulse_us_requested <= 1600.0f);
        assert(s.servo_duty_cmd <= 800u && s.servo_pulse_us_nominal == s.servo_duty_cmd * 2.0f);
    }
    assert(snapshot().servo_pulse_us_requested == 1600.0f);
    control_set_output_enabled(0u, 0u);
    assert(snapshot().state == CONTROL_READY && snapshot().servo_duty_cmd == 0u);
    control_set_output_enabled(0u, 1u);
    assert(!control_request_servo_cal(1526.0f, 1444.0f, 1610.0f, 500.0f, version));
    control_tick(0, 0);
    assert(control_request_servo_cal(1526.0f, 1444.0f, 1610.0f, 500.0f, version));
    control_report_link_fault();
    assert(snapshot().state == CONTROL_FAULT && !snapshot().servo_enabled);
    assert(!control_set_servo_cal_pulse_us(1500.0f));
    assert(!control_reset_fault());
    control_tick(0, 0);
    assert(control_reset_fault());
#else
    (void)i;
    assert(!control_request_servo_cal(1526.0f, 1444.0f, 1610.0f, 500.0f, version));
    assert(!control_request_calibration_bench());
#endif
    // 原BENCH输出必须与原宏逐值一致，新增us接口不能悄悄改变TRACK/BENCH量化。
    control_set_calibration_reserved(0u);
    for (i = 0u; i <= 150u; ++i)
    {
        float center = 85.0f + (float)i * 0.1f;
        control_init(center, 85.0f, 100.0f, -1.0f);
        control_configure_servo_pwm(10000u, 50u);
        control_set_output_enabled(1u, 1u);
        control_tick(0, 0);
        control_get_params(&p, &version);
        p.target_left = 5.0f; p.calibration_confirmed = 1u;
        assert(control_apply_params(&p, version));
        assert(control_request_run(CONTROL_MODE_BENCH));
        control_tick(0, 0);
        s = snapshot();
        assert(s.servo_duty_cmd == (uint32_t)((float)10000u / (1000.0 / (float)50u) *
                                             (0.5 + (float)center / 90.0)));
        control_request_stop();
        assert(snapshot().servo_pulse_us_requested == 0.0f && snapshot().servo_duty_cmd == 0u);
    }
    setup(); control_report_initialization_fault(); control_tick(0, 0);
    assert(!control_reset_fault() && snapshot().state == CONTROL_FAULT);
    puts("PASS servo calibration control: exclusive ownership, zero motors, limits, stop, legacy PWM");
    return 0;
}
