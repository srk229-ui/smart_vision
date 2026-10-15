#include "pid.h"
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static unsigned int checks;

// 使用输出行为验证控制器，失败时给出对应测试位置。
#define CHECK(condition) do { \
    ++checks; \
    if (!(condition)) { \
        fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
        exit(EXIT_FAILURE); \
    } \
} while (0)

#define CHECK_NEAR(actual, expected) CHECK(fabsf((actual) - (expected)) < 0.0001f)

static void check_history_cleared(const pid_param_t *pid)
{
    CHECK_NEAR(pid->out_p, 0.0f);
    CHECK_NEAR(pid->out_i, 0.0f);
    CHECK_NEAR(pid->out_d, 0.0f);
    CHECK_NEAR(pid->out_raw, 0.0f);
    CHECK_NEAR(pid->out, 0.0f);
    CHECK_NEAR(pid->integrator, 0.0f);
    CHECK_NEAR(pid->last_error, 0.0f);
    CHECK_NEAR(pid->last_derivative, 0.0f);
    CHECK(pid->last_t == 0);
}

static void test_upper_saturation_recovery(void)
{
    pid_param_t pid;
    unsigned int index;

    Pid_Param_Init(&pid, 2.0f, 10.0f, 0.0f, 100.0f, 10.0f);
    for (index = 0; index < 10000; ++index)
    {
        CHECK_NEAR(PidLocCtrl(&pid, 10.0f, 0.010f), 10.0f);
    }
    CHECK(pid.out_raw > pid.outmax);
    CHECK_NEAR(pid.out_i, 0.0f);
    CHECK_NEAR(PidLocCtrl(&pid, 2.0f, 0.010f), 4.2f);
    CHECK_NEAR(PidLocCtrl(&pid, 0.0f, 0.010f), 0.2f);
}

static void test_lower_saturation_and_unwinding(void)
{
    pid_param_t pid;
    unsigned int index;

    Pid_Param_Init(&pid, 2.0f, 10.0f, 0.0f, 100.0f, 10.0f);
    for (index = 0; index < 100; ++index)
    {
        CHECK_NEAR(PidLocCtrl(&pid, -10.0f, 0.010f), 0.0f);
    }
    CHECK(pid.out_raw < 0.0f);
    CHECK_NEAR(pid.integrator, 0.0f);
    CHECK_NEAR(PidLocCtrl(&pid, 1.0f, 0.010f), 2.1f);

    Pid_Param_Init(&pid, 0.0f, 10.0f, 0.0f, 100.0f, 10.0f);
    for (index = 0; index < 5; ++index)
    {
        (void)PidLocCtrl(&pid, 1.0f, 0.100f);
    }
    CHECK_NEAR(pid.out_i, 5.0f);
    CHECK_NEAR(PidLocCtrl(&pid, -1.0f, 0.100f), 4.0f);
    pid.outmax = 2.0f;
    CHECK_NEAR(PidLocCtrl(&pid, -1.0f, 0.100f), 2.0f);
    CHECK_NEAR(pid.out_i, 3.0f);
    CHECK_NEAR(PidLocCtrl(&pid, -1.0f, 0.100f), 2.0f);
    CHECK_NEAR(PidLocCtrl(&pid, -1.0f, 0.100f), 1.0f);
}

static void test_integral_limit_and_disabled_ki(void)
{
    pid_param_t pid;
    unsigned int index;

    Pid_Param_Init(&pid, 0.0f, 100.0f, 0.0f, 2.0f, 1000.0f);
    for (index = 0; index < 100; ++index)
    {
        (void)PidLocCtrl(&pid, 1.0f, 0.010f);
    }
    CHECK_NEAR(pid.out_i, 2.0f);
    CHECK_NEAR(pid.integrator, 2.0f);
    pid.ki = 0.0f;
    CHECK_NEAR(PidLocCtrl(&pid, 0.0f, 0.010f), 0.0f);
    CHECK_NEAR(pid.integrator, 0.0f);
    for (index = 0; index < 10000; ++index)
    {
        CHECK_NEAR(PidLocCtrl(&pid, 100.0f, 0.010f), 0.0f);
    }
    pid.ki = 1.0f;
    CHECK_NEAR(PidLocCtrl(&pid, 0.0f, 0.010f), 0.0f);
    CHECK_NEAR(PidLocCtrl(&pid, 1.0f, 0.010f), 0.010f);
}

static void test_integral_crossing_output_limits(void)
{
    pid_param_t pid;
    unsigned int index;

    // 积分步长不能整除输出范围时仍应到达限位，但不能在限位外积累。
    Pid_Param_Init(&pid, 0.0f, 3.0f, 0.0f, 100.0f, 10.0f);
    for (index = 0; index < 100; ++index)
    {
        (void)PidLocCtrl(&pid, 1.0f, 0.100f);
    }
    CHECK_NEAR(pid.out, 10.0f);
    CHECK_NEAR(pid.out_i, 10.0f);
    CHECK_NEAR(PidLocCtrl(&pid, -100.0f, 0.100f), 0.0f);
    CHECK_NEAR(pid.out_i, 0.0f);
    CHECK_NEAR(PidLocCtrl(&pid, 0.0f, 0.100f), 0.0f);
}

static void test_seconds_and_variable_dt(void)
{
    pid_param_t fast;
    pid_param_t slow;
    pid_param_t varying;
    unsigned int index;

    // 同一秒内不同采样周期应产生相同的积分输出贡献。
    Pid_Param_Init(&fast, 0.0f, 4.0f, 0.0f, 100.0f, 100.0f);
    Pid_Param_Init(&slow, 0.0f, 4.0f, 0.0f, 100.0f, 100.0f);
    for (index = 0; index < 100; ++index)
    {
        (void)PidLocCtrl(&fast, 2.0f, 0.010f);
    }
    for (index = 0; index < 50; ++index)
    {
        (void)PidLocCtrl(&slow, 2.0f, 0.020f);
    }
    CHECK_NEAR(fast.out, 8.0f);
    CHECK_NEAR(slow.out, 8.0f);
    CHECK_NEAR(fast.out, slow.out);

    Pid_Param_Init(&varying, 0.0f, 10.0f, 0.0f, 100.0f, 100.0f);
    CHECK_NEAR(PidLocCtrl(&varying, 1.0f, 0.010f), 0.1f);
    CHECK_NEAR(PidLocCtrl(&varying, 1.0f, 0.100f), 1.1f);

    Pid_Param_Init(&varying, 0.0f, 0.0f, 1.0f, 100.0f, 100.0f);
    (void)PidLocCtrl(&varying, 0.0f, 0.100f);
    CHECK_NEAR(PidLocCtrl(&varying, 1.0f, 0.100f), 10.0f);
    CHECK_NEAR(PidLocCtrl(&varying, 1.0f, 0.010f), 0.0f);
}

static void test_reset_and_independent_wheels(void)
{
    pid_param_t pid;
    float left_integral;
    float left_output;

    Pid_Param_Init(&pid, 2.0f, 3.0f, 0.0f, 4.0f, 5.0f);
    (void)PidLocCtrl(&pid, 1.0f, 0.010f);
    pid.last_derivative = 3.0f;
    pid.last_t = 10;
    Pid_Reset(&pid);
    check_history_cleared(&pid);
    CHECK_NEAR(pid.kp, 2.0f);
    CHECK_NEAR(pid.ki, 3.0f);
    CHECK_NEAR(pid.kd, 0.0f);
    CHECK_NEAR(pid.imax, 4.0f);
    CHECK_NEAR(pid.outmax, 5.0f);

    CHECK_NEAR(speed_target, 0.0f);
    My_Pid_Init();
    CHECK_NEAR(speed_pid_l.kd, 0.0f);
    CHECK_NEAR(speed_pid_r.kd, 0.0f);
    Pid_Param_Init(&speed_pid_l, 1.0f, 2.0f, 0.0f, 100.0f, 100.0f);
    Pid_Param_Init(&speed_pid_r, 3.0f, 4.0f, 0.0f, 100.0f, 100.0f);
    CHECK_NEAR(PidLocCtrl(&speed_pid_l, 1.0f, 0.010f), 1.02f);
    left_integral = speed_pid_l.integrator;
    left_output = speed_pid_l.out;
    CHECK_NEAR(PidLocCtrl(&speed_pid_r, 2.0f, 0.010f), 6.08f);
    CHECK_NEAR(speed_pid_l.integrator, left_integral);
    CHECK_NEAR(speed_pid_l.out, left_output);
    Pid_Reset(&speed_pid_r);
    CHECK_NEAR(speed_pid_l.out, left_output);
    check_history_cleared(&speed_pid_r);
}

static void test_invalid_inputs(void)
{
    pid_param_t pid;
    const float invalid_dt[] = { 0.0f, -0.010f, NAN, INFINITY };
    const float invalid_error[] = { NAN, INFINITY, -INFINITY };
    unsigned int index;

    for (index = 0; index < sizeof(invalid_dt) / sizeof(invalid_dt[0]); ++index)
    {
        Pid_Param_Init(&pid, 2.0f, 3.0f, 0.0f, 100.0f, 100.0f);
        (void)PidLocCtrl(&pid, 1.0f, 0.010f);
        CHECK_NEAR(PidLocCtrl(&pid, 1.0f, invalid_dt[index]), 0.0f);
        check_history_cleared(&pid);
        CHECK_NEAR(pid.kp, 2.0f);
    }
    for (index = 0; index < sizeof(invalid_error) / sizeof(invalid_error[0]); ++index)
    {
        Pid_Param_Init(&pid, 2.0f, 3.0f, 0.0f, 100.0f, 100.0f);
        CHECK_NEAR(PidLocCtrl(&pid, invalid_error[index], 0.010f), 0.0f);
        check_history_cleared(&pid);
    }
    Pid_Param_Init(&pid, NAN, 3.0f, 0.0f, 100.0f, 100.0f);
    CHECK_NEAR(PidLocCtrl(&pid, 1.0f, 0.010f), 0.0f);
    check_history_cleared(&pid);
    Pid_Param_Init(&pid, 2.0f, 3.0f, 0.0f, -1.0f, 100.0f);
    CHECK_NEAR(PidLocCtrl(&pid, 1.0f, 0.010f), 0.0f);
    check_history_cleared(&pid);
    Pid_Param_Init(&pid, 2.0f, 3.0f, 0.0f, 100.0f, -1.0f);
    CHECK_NEAR(PidLocCtrl(&pid, 1.0f, 0.010f), 0.0f);
    check_history_cleared(&pid);
    Pid_Param_Init(&pid, FLT_MAX, 0.0f, 0.0f, 100.0f, 100.0f);
    CHECK_NEAR(PidLocCtrl(&pid, FLT_MAX, 0.010f), 0.0f);
    check_history_cleared(&pid);
    Pid_Param_Init(&pid, 2.0f, 3.0f, 0.0f, 100.0f, 100.0f);
    pid.integrator = NAN;
    CHECK_NEAR(PidLocCtrl(&pid, 1.0f, 0.010f), 0.0f);
    check_history_cleared(&pid);
    CHECK_NEAR(PidLocCtrl(NULL, 1.0f, 0.010f), 0.0f);
    Pid_Reset(NULL);
    Pid_Param_Init(NULL, 1.0f, 1.0f, 0.0f, 10.0f, 10.0f);
}

static void test_incremental_compatibility(void)
{
    pid_param_t pid;

    Pid_Param_Init(&pid, 1.0f, 2.0f, 0.0f, 100.0f, 10.0f);
    CHECK_NEAR(PidIncCtrl(&pid, 1.0f, 0.010f), 1.02f);
    CHECK_NEAR(PidIncCtrl(&pid, 1.0f, 0.010f), 1.04f);
    CHECK_NEAR(PidIncCtrl(&pid, -100.0f, 0.010f), 0.0f);
    CHECK(pid.out_raw < 0.0f);
    CHECK_NEAR(PidIncCtrl(&pid, 1.0f, 0.0f), 0.0f);
    check_history_cleared(&pid);
    CHECK_NEAR(PidIncCtrl(&pid, INFINITY, 0.010f), 0.0f);
    check_history_cleared(&pid);
    CHECK_NEAR(PidIncCtrl(NULL, 1.0f, 0.010f), 0.0f);
}

int main(void)
{
    test_upper_saturation_recovery();
    test_lower_saturation_and_unwinding();
    test_integral_limit_and_disabled_ki();
    test_integral_crossing_output_limits();
    test_seconds_and_variable_dt();
    test_reset_and_independent_wheels();
    test_invalid_inputs();
    test_incremental_compatibility();
    printf("PASS test_pid: %u checks\n", checks);
    return EXIT_SUCCESS;
}
