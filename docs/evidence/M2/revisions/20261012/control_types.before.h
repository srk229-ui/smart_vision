#ifndef CONTROL_TYPES_H
#define CONTROL_TYPES_H

#include <stdint.h>

#define CONTROL_PERIOD_MS          (10u)
#define CONTROL_IMAGE_WIDTH        (188u)
#define CONTROL_IMAGE_HEIGHT       (120u)
#define CONTROL_BAND_COUNT         (5u)
#define CONTROL_BENCH_MAX_MS        (8000u)

typedef enum
{
    CONTROL_INIT = 0,
    CONTROL_READY,
    CONTROL_RUNNING,
    CONTROL_FAULT
} ControlState;

typedef enum
{
    CONTROL_MODE_TRACK = 0,
    CONTROL_MODE_BENCH
} ControlMode;

typedef enum
{
    CONTROL_FAULT_NONE = 0,
    CONTROL_FAULT_VISION_TIMEOUT,
    CONTROL_FAULT_TRACK_LOST,
    CONTROL_FAULT_LINK,
    CONTROL_FAULT_INVALID_VALUE
} ControlFault;

typedef struct
{
    uint32_t frame_id;
    uint32_t capture_time_ms;
    float error_px;
    float confidence;
    uint8_t threshold;
    uint8_t valid;
    uint8_t track_lost;
    uint8_t left[CONTROL_IMAGE_HEIGHT];
    uint8_t middle[CONTROL_IMAGE_HEIGHT];
    uint8_t right[CONTROL_IMAGE_HEIGHT];
    /* 行质量：0 无效，1 使用标定宽度重建，2 左右边均实测。 */
    uint8_t row_quality[CONTROL_IMAGE_HEIGHT];
    uint16_t valid_rows;
    uint16_t reconstructed_rows;
} VisionResult;

typedef struct
{
    float target_left;
    float target_right;
    float kp_left;
    float ki_left;
    float kp_right;
    float ki_right;
    float pwm_max;
    float integral_max;
    /* 速度单位为 counts/10ms，斜坡单位为该速度单位每秒。 */
    float accel_counts_per_s;
    float servo_center;
    float servo_min;
    float servo_max;
    float servo_direction;
    float steer_kp;
    float steer_kd;
    float servo_rate_deg_s;
    float curve_speed_factor;
    uint8_t look_rows[CONTROL_BAND_COUNT];
    float look_weights[CONTROL_BAND_COUNT];
    uint16_t vision_timeout_ms;
    uint8_t lost_frame_limit;
    /* 必须由现场操作者确认硬件方向、舵机限位及试验条件。 */
    uint8_t calibration_confirmed;
} ControlParams;

typedef struct
{
    uint32_t sample_id;
    uint32_t timestamp_ms;
    uint32_t frame_id;
    uint32_t params_version;
    uint32_t params_applied_ms;
    float target_left;
    float target_right;
    int16_t measured_left;
    int16_t measured_right;
    float pwm_left;
    float pwm_right;
    float raw_pwm_left;
    float raw_pwm_right;
    float integral_left;
    float integral_right;
    float vision_error_px;
    float confidence;
    float servo_cmd;
    uint32_t vision_age_ms;
    ControlState state;
    ControlMode mode;
    ControlFault fault;
    uint8_t threshold;
    uint8_t saturated_left;
    uint8_t saturated_right;
    uint8_t servo_enabled;
} ControlSnapshot;

#endif
