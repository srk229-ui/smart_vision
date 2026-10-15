#include "steering_map.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

static SteeringMap active_map; // 停止态应用的平均实测表；不自动参与当前命令度控制。
static uint32_t (*critical_enter)(void); // 保存中断屏蔽状态的短临界区回调。
static void (*critical_leave)(uint32_t); // 恢复原状态的配对回调。

uint8_t steering_wheel_map_validate(const SteeringWheelMap *map)
{
    uint8_t i;
    float trend;
    if (!map || (map->side != STEERING_SIDE_LEFT && map->side != STEERING_SIDE_RIGHT) ||
        map->point_count < 2u || map->point_count > STEERING_MAP_MAX_POINTS) return 0u;
    trend = map->points[1].wheel_deg - map->points[0].wheel_deg;
    if (!isfinite(trend) || trend == 0.0f) return 0u;
    for (i = 0u; i < map->point_count; ++i)
    {
        const SteeringWheelMapPoint *point = &map->points[i];
        if (!isfinite(point->pulse_us) || !isfinite(point->wheel_deg) || point->pulse_us < 500.0f ||
            point->pulse_us > 2500.0f || fabsf(point->wheel_deg) >= 89.0f) return 0u;
        if (i > 0u)
        {
            float change = point->wheel_deg - map->points[i - 1u].wheel_deg;
            // 相邻脉宽必须严格递增，角度必须始终同向；拒绝零长度和非单调段。
            if (point->pulse_us <= map->points[i - 1u].pulse_us || change == 0.0f ||
                (change > 0.0f) != (trend > 0.0f)) return 0u;
        }
    }
    return 1u;
}

uint8_t steering_wheel_map_us_to_angle(const SteeringWheelMap *map, float pulse_us, float *angle_deg)
{
    uint8_t i;
    if (!angle_deg || !steering_wheel_map_validate(map) || !isfinite(pulse_us) ||
        pulse_us < map->points[0].pulse_us || pulse_us > map->points[map->point_count - 1u].pulse_us) return 0u;
    for (i = 1u; i < map->point_count; ++i)
    {
        if (pulse_us <= map->points[i].pulse_us)
        {
            const SteeringWheelMapPoint *a = &map->points[i - 1u];
            const SteeringWheelMapPoint *b = &map->points[i];
            float fraction = (pulse_us - a->pulse_us) / (b->pulse_us - a->pulse_us);
            *angle_deg = a->wheel_deg + fraction * (b->wheel_deg - a->wheel_deg); // 保留相对车身纵轴的原始角，不扣除+5度偏置。
            return 1u;
        }
    }
    return 0u;
}

uint8_t steering_wheel_map_angle_to_us(const SteeringWheelMap *map, float angle_deg, float *pulse_us)
{
    uint8_t i;
    if (!pulse_us || !steering_wheel_map_validate(map) || !isfinite(angle_deg)) return 0u;
    for (i = 1u; i < map->point_count; ++i)
    {
        const SteeringWheelMapPoint *a = &map->points[i - 1u];
        const SteeringWheelMapPoint *b = &map->points[i];
        if (angle_deg >= fminf(a->wheel_deg, b->wheel_deg) && angle_deg <= fmaxf(a->wheel_deg, b->wheel_deg))
        {
            *pulse_us = a->pulse_us + (angle_deg - a->wheel_deg) * (b->pulse_us - a->pulse_us) / (b->wheel_deg - a->wheel_deg);
            return 1u; // 严格单调保证分母非零；只反查实测角区间，越界不夹紧。
        }
    }
    return 0u;
}

// 外推边界针对原表车身纵轴角，不扣中位；角域必须远离tan奇点，不能接受NaN或反序区间。
static uint8_t extrapolation_bounds_valid(float low, float high)
{
    return (uint8_t)(isfinite(low) && isfinite(high) && low < high && low > -89.0f && high < 89.0f);
}

uint8_t steering_wheel_map_us_to_angle_extrapolate(const SteeringWheelMap *map, float pulse_us,
    float angle_min_deg, float angle_max_deg, float *angle_deg)
{
    uint8_t i;
    const SteeringWheelMapPoint *a, *b;
    float value;
    if (!angle_deg || !steering_wheel_map_validate(map) || !extrapolation_bounds_valid(angle_min_deg, angle_max_deg) ||
        !isfinite(pulse_us) || pulse_us < 500.0f || pulse_us > 2500.0f) return 0u;
    // 小于首点选首段，大于末点选末段；实测范围内选包含输入的原线段。
    for (i = 1u; i < map->point_count - 1u && pulse_us > map->points[i].pulse_us; ++i) { }
    a = &map->points[i - 1u]; b = &map->points[i];
    value = a->wheel_deg + (pulse_us - a->pulse_us) * (b->wheel_deg - a->wheel_deg) / (b->pulse_us - a->pulse_us);
    // 反解端点转回float可有数个尾数误差；1e-4deg不足以放过任何本工程2us的PWM网格。
    if (!isfinite(value) || value < angle_min_deg - 0.0001f || value > angle_max_deg + 0.0001f) return 0u;
    *angle_deg = fminf(fmaxf(value, angle_min_deg), angle_max_deg);
    return 1u;
}

uint8_t steering_wheel_map_angle_to_us_extrapolate(const SteeringWheelMap *map, float angle_deg,
    float angle_min_deg, float angle_max_deg, float *pulse_us)
{
    uint8_t i, increasing;
    const SteeringWheelMapPoint *a, *b;
    float value;
    if (!pulse_us || !steering_wheel_map_validate(map) || !extrapolation_bounds_valid(angle_min_deg, angle_max_deg) ||
        !isfinite(angle_deg) || angle_deg < angle_min_deg || angle_deg > angle_max_deg) return 0u;
    increasing = (uint8_t)(map->points[1].wheel_deg > map->points[0].wheel_deg);
    // 递增表按角增大找段，递减表按角减小找段；越过两端仍只延续相邻端段。
    for (i = 1u; i < map->point_count - 1u &&
         (increasing ? angle_deg > map->points[i].wheel_deg : angle_deg < map->points[i].wheel_deg); ++i) { }
    a = &map->points[i - 1u]; b = &map->points[i];
    value = a->pulse_us + (angle_deg - a->wheel_deg) * (b->pulse_us - a->pulse_us) / (b->wheel_deg - a->wheel_deg);
    if (!isfinite(value) || value < 500.0f || value > 2500.0f) return 0u;
    *pulse_us = value; // 不写回原表，也不将外推端点冒充实测节点。
    return 1u;
}

uint8_t steering_map_validate(const SteeringMap *map)
{
    uint8_t i;
    uint8_t center_found = 0u;
    float trend_left;
    float trend_right;
    if (!map || map->calibrated != 1u || map->point_count < 3u || map->point_count > STEERING_MAP_MAX_POINTS ||
        !isfinite(map->center_us) || !isfinite(map->pulse_min_us) || !isfinite(map->pulse_max_us) ||
        map->pulse_min_us < 500.0f || map->pulse_max_us > 2500.0f || map->pulse_min_us >= map->pulse_max_us ||
        map->center_us <= map->pulse_min_us || map->center_us >= map->pulse_max_us) return 0u;
    if (fabsf(map->points[0].pulse_us - map->pulse_min_us) > 0.01f ||
        fabsf(map->points[map->point_count - 1u].pulse_us - map->pulse_max_us) > 0.01f) return 0u;
    trend_left = map->points[1].wheel_left_deg - map->points[0].wheel_left_deg;
    trend_right = map->points[1].wheel_right_deg - map->points[0].wheel_right_deg;
    if (trend_left == 0.0f || trend_right == 0.0f || (trend_left > 0.0f) != (trend_right > 0.0f)) return 0u;
    for (i = 0u; i < map->point_count; ++i)
    {
        const SteeringMapPoint *point = &map->points[i];
        if (!isfinite(point->pulse_us) || !isfinite(point->wheel_left_deg) || !isfinite(point->wheel_right_deg) ||
            point->pulse_us < 500.0f || point->pulse_us > 2500.0f ||
            fabsf(point->wheel_left_deg) >= 89.0f || fabsf(point->wheel_right_deg) >= 89.0f) return 0u;
        if (i > 0u)
        {
            float left_change = point->wheel_left_deg - map->points[i - 1u].wheel_left_deg;
            float right_change = point->wheel_right_deg - map->points[i - 1u].wheel_right_deg;
            if (point->pulse_us <= map->points[i - 1u].pulse_us || left_change == 0.0f || right_change == 0.0f ||
                (left_change > 0.0f) != (trend_left > 0.0f) || (right_change > 0.0f) != (trend_right > 0.0f)) return 0u;
        }
        if (fabsf(point->pulse_us - map->center_us) <= 0.01f)
        {
            if (fabsf(point->wheel_left_deg) > 0.01f || fabsf(point->wheel_right_deg) > 0.01f) return 0u;
            center_found = 1u;
        }
    }
    return center_found;
}

uint8_t steering_map_us_to_angles(const SteeringMap *map, float pulse_us, float *left_deg, float *right_deg)
{
    uint8_t i;
    if (!left_deg || !right_deg || !steering_map_validate(map) || !isfinite(pulse_us) ||
        pulse_us < map->points[0].pulse_us || pulse_us > map->points[map->point_count - 1u].pulse_us) return 0u;
    for (i = 1u; i < map->point_count; ++i)
    {
        if (pulse_us <= map->points[i].pulse_us)
        {
            const SteeringMapPoint *a = &map->points[i - 1u];
            const SteeringMapPoint *b = &map->points[i];
            float fraction = (pulse_us - a->pulse_us) / (b->pulse_us - a->pulse_us); // 严格升序保证分母正。
            *left_deg = a->wheel_left_deg + fraction * (b->wheel_left_deg - a->wheel_left_deg);
            *right_deg = a->wheel_right_deg + fraction * (b->wheel_right_deg - a->wheel_right_deg);
            return 1u;
        }
    }
    return 0u;
}

uint8_t steering_map_angle_to_us(const SteeringMap *map, SteeringSide side, float angle_deg, float *pulse_us)
{
    uint8_t i;
    if (!pulse_us || !steering_map_validate(map) || !isfinite(angle_deg) ||
        (side != STEERING_SIDE_LEFT && side != STEERING_SIDE_RIGHT)) return 0u;
    for (i = 1u; i < map->point_count; ++i)
    {
        const SteeringMapPoint *a = &map->points[i - 1u];
        const SteeringMapPoint *b = &map->points[i];
        float angle_a = side == STEERING_SIDE_LEFT ? a->wheel_left_deg : a->wheel_right_deg;
        float angle_b = side == STEERING_SIDE_LEFT ? b->wheel_left_deg : b->wheel_right_deg;
        if (angle_deg >= fminf(angle_a, angle_b) && angle_deg <= fmaxf(angle_a, angle_b))
        {
            *pulse_us = a->pulse_us + (angle_deg - angle_a) * (b->pulse_us - a->pulse_us) / (angle_b - angle_a);
            return 1u; // 两轮各自严格单调，区间内反插值结果必在已验证的安全脉宽范围。
        }
    }
    return 0u;
}

void steering_map_set_critical_hooks(uint32_t (*enter)(void), void (*leave)(uint32_t))
{
    if ((enter == NULL) != (leave == NULL)) return;
    critical_enter = enter;
    critical_leave = leave;
}

void steering_map_init(void)
{
    memset(&active_map, 0, sizeof(active_map));
    active_map.version = 1u;
}

void steering_map_get(SteeringMap *map)
{
    uint32_t key = critical_enter ? critical_enter() : 0u;
    if (map) *map = active_map;
    if (critical_leave) critical_leave(key);
}

uint8_t steering_map_apply(const SteeringMap *map, uint32_t expected_version)
{
    uint32_t key;
    if (!steering_map_validate(map)) return 0u;
    key = critical_enter ? critical_enter() : 0u;
    if (expected_version != active_map.version)
    {
        if (critical_leave) critical_leave(key);
        return 0u;
    }
    active_map = *map;
    active_map.version = expected_version + 1u; // 使用实际运行版本，不能由离线候选伪造或回退。
    if (critical_leave) critical_leave(key);
    return 1u;
}
