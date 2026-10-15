#include "calibration_debug.h"
#include <math.h>
#include <string.h>

uint32_t calibration_debug_crc32(const void *bytes, uint32_t length)
{
    const uint8_t *data = (const uint8_t *)bytes;
    uint32_t crc = 0xffffffffu;
    uint32_t i;
    while (length-- != 0u)
    {
        crc ^= *data++;
        for (i = 0u; i < 8u; ++i) crc = (crc >> 1) ^ ((crc & 1u) ? 0xedb88320u : 0u);
    }
    return crc ^ 0xffffffffu; // 标准反射IEEE CRC32，与Python zlib一致。
}

#if CALIBRATION_DEBUG_ENABLE
#include "control.h"
#include "steering_map.h"
#if defined(CALIBRATION_DEBUG_HOST)
#define CAL_DTCM(var) var // 宿主只验证协议和实际C逻辑，不声明MCU内存可达性。
#define CAL_BARRIER() ((void)0) // 宿主单线程mock不产生总线写入。
#else
#include "zf_common_headfile.h"
#define CAL_DTCM(var) AT_DTCM_SECTION_ALIGN(var, 32) // 映射到现有DTCM段，最终地址由ELF符号确定。
#define CAL_BARRIER() __DMB() // 在seq/ACK发布前完成CPU内存访问顺序。
#endif

CAL_DTCM(volatile CalibrationDebugMailbox calibration_debug_mailbox);
CAL_DTCM(CalibrationDebugRecord calibration_debug_log[CALIBRATION_DEBUG_LOG_CAPACITY]);
const char calibration_debug_build_stamp[] = __DATE__ " " __TIME__;

typedef struct
{
    uint32_t session_id; // 请求所属主机会话。
    uint32_t seq; // 主机最后提交的请求序号。
    uint32_t opcode; // 准备、显式START、查询等操作。
    uint32_t args[8]; // 固定44字节CRC输入中的载荷。
} CalibrationRequest;
typedef char CalibrationRequestSizeCheck[(sizeof(CalibrationRequest) == 44u) ? 1 : -1];

static uint32_t (*critical_enter)(void);
static void (*critical_leave)(uint32_t);
static uint32_t owner_session; // 当前准备拥有者，停止后仍可读取本组结果但不能自动再启动。
static uint32_t last_request_seq; // 已消费请求，CRC错误也消费避免循环执行。
static uint32_t last_stop_seq; // 独立STOP已消费值；任何新值均只产生停止。
static uint32_t last_heartbeat_seq; // 当前会话最后新鲜心跳，重复/回退不续租。
static uint32_t heartbeat_us; // 最后接受新心跳的真实us时间。
static uint32_t active_start_us; // 手转或执行器活动起点，SET不修改。
static uint32_t prepared_params_version; // START要求参数仍与准备ACK一致。
static uint32_t measurement_error_start; // 本组开始异常计数，出现新增异常立即终止。
static uint32_t last_logged_sample; // 起点已包含的样本不可再次写入日志，随后每个新样本至多写一次。
static uint32_t prepared_stop_generation; // 捕获普通V1 STOP/FAULT，手转READY态也不能漏掉停止。
static uint32_t pending_marker_mask; // 主循环待领取TFT请求位。
static uint32_t requested_marker_mask; // 同一标记不重复请求，防止覆盖手机参考事件。
static uint32_t completed_marker_mask; // 已完成的软件标记位，支持时间戳为0。
static uint8_t heartbeat_seen; // 当前会话至少收到一次非零心跳。
static uint8_t restore_pending; // RPM临时参数必须恢复后才可释放准备权。
static uint8_t freeze_pending; // 停机周期实际写零后再冻结有限记录。
static uint8_t recording; // 本次SERVO/RPM记录是否已开始。
static uint8_t servo_event_pending; // 显式SET后的首周期须留档；稳定脉宽不逐周期占用日志。
static ControlParams saved_params; // RPM准备前完整参数，用于每一结束/失败路径还原。
static float prepared_servo[4]; // 中位/最小/最大us及us/s，不修改TRACK命令度参数。

static uint32_t lock_debug(void) { return critical_enter != NULL ? critical_enter() : 0u; }
static void unlock_debug(uint32_t key) { if (critical_leave != NULL) critical_leave(key); }
static uint32_t float_bits(float value) { uint32_t bits; memcpy(&bits, &value, sizeof(bits)); return bits; }
static float bits_float(uint32_t bits) { float value; memcpy(&value, &bits, sizeof(value)); return value; }
static uint32_t pair16(int16_t left, int16_t right)
{
    return (uint32_t)(uint16_t)left | ((uint32_t)(uint16_t)right << 16);
}
static uint8_t is_prepared(uint32_t phase)
{
    return (uint8_t)(phase >= CAL_PHASE_PREPARED_COUNT && phase <= CAL_PHASE_PREPARED_RPM);
}
static uint8_t is_active(uint32_t phase)
{
    return (uint8_t)(phase >= CAL_PHASE_ACTIVE_COUNT && phase <= CAL_PHASE_ACTIVE_RPM);
}
static void snapshot_begin(void)
{
    ++calibration_debug_mailbox.snapshot_seq;
    CAL_BARRIER();
}
static void snapshot_end(void)
{
    CAL_BARRIER();
    ++calibration_debug_mailbox.snapshot_seq;
}
static void write_total(volatile uint32_t *lo, volatile uint32_t *hi, int64_t total)
{
    uint64_t value = (uint64_t)total;
    *lo = (uint32_t)value;
    *hi = (uint32_t)(value >> 32); // wire保留int64二补码，主机不得分别有符号相加。
}
static void save_endpoint(uint8_t start, const EncoderMeasurementSnapshot *measurement)
{
    if (measurement == NULL) return;
    if (start)
    {
        calibration_debug_mailbox.count_start_us = measurement->now_us;
        write_total(&calibration_debug_mailbox.count_start_left_lo, &calibration_debug_mailbox.count_start_left_hi, measurement->total_left);
        write_total(&calibration_debug_mailbox.count_start_right_lo, &calibration_debug_mailbox.count_start_right_hi, measurement->total_right);
    }
    else
    {
        calibration_debug_mailbox.count_end_us = measurement->now_us;
        write_total(&calibration_debug_mailbox.count_end_left_lo, &calibration_debug_mailbox.count_end_left_hi, measurement->total_left);
        write_total(&calibration_debug_mailbox.count_end_right_lo, &calibration_debug_mailbox.count_end_right_hi, measurement->total_right);
    }
}

static uint8_t restore_parameters(void)
{
    ControlParams current_params;
    uint32_t version;
    if (!restore_pending) return 1u;
    control_get_params(&current_params, &version);
    if (!control_apply_calibration_params(&saved_params, version))
    {
        control_report_invalid_fault(); // 还原失败时保留独占权，不能暴露临时确认供普通启动。
        return 0u;
    }
    restore_pending = 0u;
    return 1u;
}

static void finish_trial(uint32_t reason, const EncoderMeasurementSnapshot *measurement)
{
    uint32_t previous_phase = calibration_debug_mailbox.phase;
    control_request_stop(); // 只改统一软件状态，实际零PWM仍由本次/后续ISR写入。
    calibration_debug_mailbox.end_reason = reason;
    if (is_active(previous_phase)) save_endpoint(0u, measurement);
    calibration_debug_mailbox.phase =
        reason == CAL_END_MANUAL || (reason == CAL_END_TIME_LIMIT && previous_phase != CAL_PHASE_ACTIVE_COUNT) ?
        CAL_PHASE_COMPLETE : CAL_PHASE_ABORTED;
    pending_marker_mask = 0u; // 停止后不再触发旧TFT动作。
    servo_event_pending = 0u; // 终止周期由freeze_pending强制记录，不把旧SET事件带入下一组。
    if (recording || previous_phase == CAL_PHASE_ACTIVE_COUNT)
    {
        calibration_debug_mailbox.log_end_reason = reason;
        freeze_pending = 1u; // after_tick记录停止周期并在ISR实际写零后冻结。
    }
    if (restore_parameters()) control_set_calibration_reserved(0u);
    prepared_stop_generation = control_stop_generation(); // 本函数自己的STOP/还原故障不在下一轮重复触发。
}

static void publish_ack(uint32_t seq, uint32_t session, uint32_t status)
{
    calibration_debug_mailbox.ack_status = status;
    calibration_debug_mailbox.ack_session_id = session;
    CAL_BARRIER();
    calibration_debug_mailbox.ack_seq = seq; // 最后发布ACK，主机此后才可复用载荷区。
}

static uint8_t consume_stop(const EncoderMeasurementSnapshot *measurement)
{
    uint32_t stop = calibration_debug_mailbox.stop_seq;
    if (stop == last_stop_seq) return 0u;
    last_stop_seq = stop;
    if (is_prepared(calibration_debug_mailbox.phase) || is_active(calibration_debug_mailbox.phase))
        finish_trial(CAL_END_STOP, measurement);
    else
    {
        control_request_stop(); // 结束后的重复STOP仍清输出，但不改写已完成手转/日志的结束事实。
        if (restore_parameters()) control_set_calibration_reserved(0u);
        prepared_stop_generation = control_stop_generation();
    }
    // STOP抢先消费当前提交，尚未执行START不能在下一轮延迟执行。
    if (calibration_debug_mailbox.request_seq != 0u && calibration_debug_mailbox.request_seq != last_request_seq)
    {
        last_request_seq = calibration_debug_mailbox.request_seq;
        publish_ack(last_request_seq, calibration_debug_mailbox.session_id, CAL_ACK_STOPPED);
    }
    CAL_BARRIER();
    calibration_debug_mailbox.stop_ack_seq = stop; // 主机重开必须匹配本次STOP，旧冻结状态不能作为确认。
    return 1u;
}

static uint8_t consume_external_stop(const EncoderMeasurementSnapshot *measurement)
{
    if ((!is_prepared(calibration_debug_mailbox.phase) && !is_active(calibration_debug_mailbox.phase)) ||
        (control_stop_generation() == prepared_stop_generation && control_get_state() != CONTROL_FAULT))
        return 0u;
    finish_trial(CAL_END_CONTROL, measurement); // 普通STOP必须中止准备态及无电机的手转组。
    if (calibration_debug_mailbox.request_seq != 0u && calibration_debug_mailbox.request_seq != last_request_seq)
    {
        last_request_seq = calibration_debug_mailbox.request_seq;
        publish_ack(last_request_seq, calibration_debug_mailbox.session_id, CAL_ACK_STOPPED);
    }
    return 1u;
}

static void accept_heartbeat(uint32_t now_us)
{
    uint32_t session_before = calibration_debug_mailbox.heartbeat_session_id;
    uint32_t seq = calibration_debug_mailbox.heartbeat_seq;
    CAL_BARRIER();
    if (owner_session != 0u && session_before == owner_session &&
        calibration_debug_mailbox.heartbeat_session_id == session_before && seq != 0u &&
        (!heartbeat_seen || ((uint32_t)(seq - last_heartbeat_seq) != 0u &&
                            (uint32_t)(seq - last_heartbeat_seq) < 0x80000000u)))
    {
        last_heartbeat_seq = seq;
        heartbeat_us = now_us;
        heartbeat_seen = 1u;
    }
}

static void begin_trial(uint32_t phase, uint32_t request_seq, uint32_t now_us,
                        const EncoderMeasurementSnapshot *measurement)
{
    ControlSnapshot output;
    control_get_snapshot(&output);
    calibration_debug_mailbox.phase = phase;
    calibration_debug_mailbox.end_reason = CAL_END_NONE;
    active_start_us = now_us;
    measurement_error_start = measurement->error_count;
    last_logged_sample = measurement->sample_id;
    pending_marker_mask = requested_marker_mask = completed_marker_mask = 0u;
    memset((void *)calibration_debug_mailbox.marker_start_us, 0, sizeof(calibration_debug_mailbox.marker_start_us));
    memset((void *)calibration_debug_mailbox.marker_end_us, 0, sizeof(calibration_debug_mailbox.marker_end_us));
    ++calibration_debug_mailbox.log_seq;
    calibration_debug_mailbox.log_run_id = request_seq;
    calibration_debug_mailbox.log_kind = phase;
    calibration_debug_mailbox.log_params_version = output.params_version;
    calibration_debug_mailbox.log_calibration_version = measurement->calibration_version;
    calibration_debug_mailbox.log_count = 0u;
    calibration_debug_mailbox.log_frozen = 0u;
    calibration_debug_mailbox.log_end_reason = CAL_END_NONE;
    CAL_BARRIER();
    ++calibration_debug_mailbox.log_seq;
    save_endpoint(1u, measurement); // 日志第一条之前的累计基准，用于主机复算与终点核对。
    calibration_debug_mailbox.count_end_us = 0u;
    recording = (uint8_t)(phase != CAL_PHASE_ACTIVE_COUNT);
    servo_event_pending = (uint8_t)(phase == CAL_PHASE_ACTIVE_SERVO);
    freeze_pending = 0u;
}

static uint32_t prepare_request(const CalibrationRequest *request, uint32_t now_us)
{
    ControlParams params;
    uint32_t version;
    uint32_t phase;
    if (control_get_state() != CONTROL_READY || calibration_debug_busy()) return CAL_ACK_BAD_STATE;
    if (request->session_id == 0u) return CAL_ACK_BAD_SESSION;
    control_get_params(&params, &version);
    if (request->opcode == CAL_CMD_PREPARE_SERVO)
    {
        uint32_t i;
        if (request->args[4] != version) return CAL_ACK_STALE_VERSION;
        if (request->args[5] != 1u) return CAL_ACK_BAD_ARGUMENT;
        for (i = 0u; i < 4u; ++i)
        {
            prepared_servo[i] = bits_float(request->args[i]);
            if (!isfinite(prepared_servo[i])) return CAL_ACK_BAD_ARGUMENT;
        }
        if (prepared_servo[1] < 500.0f || prepared_servo[2] > 2500.0f ||
            prepared_servo[1] >= prepared_servo[2] || prepared_servo[0] < prepared_servo[1] ||
            prepared_servo[0] > prepared_servo[2] || prepared_servo[3] <= 0.0f || prepared_servo[3] > 4000.0f)
            return CAL_ACK_BAD_ARGUMENT;
        phase = CAL_PHASE_PREPARED_SERVO;
    }
    else if (request->opcode == CAL_CMD_PREPARE_RPM)
    {
        float left = bits_float(request->args[0]);
        float right = bits_float(request->args[1]);
        if (request->args[2] != version) return CAL_ACK_STALE_VERSION;
        if (request->args[3] != 1u || !isfinite(left) || !isfinite(right) ||
            left < 0.0f || right < 0.0f || left > CONTROL_TARGET_COUNTS_MAX || right > CONTROL_TARGET_COUNTS_MAX || (left == 0.0f && right == 0.0f))
            return CAL_ACK_BAD_ARGUMENT;
        saved_params = params;
        params.target_left = left;
        params.target_right = right;
        params.calibration_confirmed = 1u; // 仅临时架空许可，所有结束/失败路径恢复原组。
        phase = CAL_PHASE_PREPARED_RPM;
    }
    else
    {
        if (request->args[0] != 1u) return CAL_ACK_BAD_ARGUMENT;
        phase = CAL_PHASE_PREPARED_COUNT;
    }
    control_request_stop();
    control_set_calibration_reserved(1u);
    if (!control_get_calibration_reserved()) return CAL_ACK_CONTROL_REJECTED;
    if (phase == CAL_PHASE_PREPARED_RPM)
    {
        if (!control_apply_calibration_params(&params, version))
        {
            control_set_calibration_reserved(0u);
            return CAL_ACK_CONTROL_REJECTED;
        }
        restore_pending = 1u;
        control_get_params(&params, &version);
    }
    owner_session = request->session_id;
    prepared_params_version = version;
    last_heartbeat_seq = 0u;
    heartbeat_seen = 0u;
    heartbeat_us = now_us; // 准备后即使尚未收到心跳，也只有500ms准备窗口。
    accept_heartbeat(now_us);
    calibration_debug_mailbox.prepare_token = request->seq;
    calibration_debug_mailbox.phase = phase;
    calibration_debug_mailbox.end_reason = CAL_END_NONE;
    prepared_stop_generation = control_stop_generation();
    return CAL_ACK_OK;
}

static uint32_t process_request(const CalibrationRequest *request, uint32_t now_us,
                                const EncoderMeasurementSnapshot *measurement)
{
    uint32_t phase = calibration_debug_mailbox.phase;
    ControlParams params;
    uint32_t version;
    if (request->opcode >= CAL_CMD_PREPARE_COUNT && request->opcode <= CAL_CMD_PREPARE_RPM)
        return prepare_request(request, now_us);
    if (request->session_id == 0u) return CAL_ACK_BAD_SESSION;
    // 纯查表允许未准备会话，既不抢占执行器，也不续租活动心跳。
    if (request->opcode == CAL_CMD_QUERY_US || request->opcode == CAL_CMD_QUERY_ANGLE)
    {
        SteeringMap map;
        float first = 0.0f;
        float second = 0.0f;
        uint8_t accepted;
        steering_map_get(&map);
        calibration_debug_mailbox.snapshot[CAL_SNAP_MAP_VERSION] = map.version;
        calibration_debug_mailbox.snapshot[CAL_SNAP_MAP_VALID] = map.calibrated;
        accepted = request->opcode == CAL_CMD_QUERY_US ?
            steering_map_us_to_angles(&map, bits_float(request->args[0]), &first, &second) :
            steering_map_angle_to_us(&map, (SteeringSide)request->args[1], bits_float(request->args[0]), &first);
        if (!accepted) return CAL_ACK_UNCALIBRATED;
        calibration_debug_mailbox.snapshot[CAL_SNAP_RESULT_0] = float_bits(first);
        calibration_debug_mailbox.snapshot[CAL_SNAP_RESULT_1] = float_bits(second);
        return CAL_ACK_OK;
    }
    if (request->opcode == CAL_CMD_CLEAR_FAULT)
    {
        if (calibration_debug_busy()) return CAL_ACK_BAD_STATE;
        return control_reset_fault() ? CAL_ACK_OK : CAL_ACK_CONTROL_REJECTED;
    }
    if (request->opcode == CAL_CMD_APPLY_ENCODER)
    {
        float left = bits_float(request->args[0]);
        float right = bits_float(request->args[1]);
        // 完整重复实测collection由主机核验；固件只允许停车、人工确认且版本匹配的有限正N。
        if (calibration_debug_busy() || control_get_state() != CONTROL_READY) return CAL_ACK_BAD_STATE;
        if (request->args[3] != 1u || !isfinite(left) || !isfinite(right) || left <= 0.0f || right <= 0.0f) return CAL_ACK_BAD_ARGUMENT;
        if (request->args[2] != measurement->calibration_version) return CAL_ACK_STALE_VERSION;
        return encoder_measurement_set_counts_per_rev(left, right, request->args[2]) ? CAL_ACK_OK : CAL_ACK_BAD_ARGUMENT;
    }
    if (request->session_id != owner_session) return CAL_ACK_BAD_SESSION;
    if (request->opcode == CAL_CMD_START || request->opcode == CAL_CMD_COUNT_START)
    {
        if (!is_prepared(phase) || request->args[0] != calibration_debug_mailbox.prepare_token) return CAL_ACK_BAD_STATE;
        if (!heartbeat_seen || (uint32_t)(now_us - heartbeat_us) >= CALIBRATION_LINK_TIMEOUT_US) return CAL_ACK_BAD_STATE;
        if (!measurement->valid) return CAL_ACK_MEASUREMENT_INVALID;
        control_get_params(&params, &version);
        if (version != prepared_params_version) return CAL_ACK_STALE_VERSION;
        if (phase == CAL_PHASE_PREPARED_COUNT)
        {
            if (request->opcode != CAL_CMD_COUNT_START) return CAL_ACK_BAD_STATE;
            if (measurement->stationary_us < CALIBRATION_STATIONARY_US) return CAL_ACK_NOT_STATIONARY;
            begin_trial(CAL_PHASE_ACTIVE_COUNT, request->seq, now_us, measurement);
        }
        else
        {
            uint8_t accepted;
            if (request->opcode != CAL_CMD_START) return CAL_ACK_BAD_STATE;
            accepted = phase == CAL_PHASE_PREPARED_RPM ? control_request_calibration_bench() :
                control_request_servo_cal(prepared_servo[0], prepared_servo[1], prepared_servo[2], prepared_servo[3], version);
            if (!accepted)
            {
                finish_trial(CAL_END_CONTROL, measurement);
                return CAL_ACK_CONTROL_REJECTED;
            }
            begin_trial(phase == CAL_PHASE_PREPARED_RPM ? CAL_PHASE_ACTIVE_RPM : CAL_PHASE_ACTIVE_SERVO,
                        request->seq, now_us, measurement);
        }
        return CAL_ACK_OK;
    }
    if (request->opcode == CAL_CMD_SET_SERVO)
    {
        float pulse = bits_float(request->args[0]);
        if (phase != CAL_PHASE_ACTIVE_SERVO) return CAL_ACK_BAD_STATE;
        if (!isfinite(pulse) || !control_set_servo_cal_pulse_us(pulse)) return CAL_ACK_BAD_ARGUMENT;
        servo_event_pending = 1u; // 保留本次目标改变后的首周期，后续只记录实际整数占空变化。
        return CAL_ACK_OK;
    }
    if (request->opcode == CAL_CMD_SERVO_END)
    {
        if (phase != CAL_PHASE_ACTIVE_SERVO || request->args[0] != calibration_debug_mailbox.prepare_token) return CAL_ACK_BAD_STATE;
        if (!measurement->valid || measurement->error_count != measurement_error_start) return CAL_ACK_MEASUREMENT_INVALID;
        finish_trial(CAL_END_MANUAL, measurement);
        return CAL_ACK_OK;
    }
    if (request->opcode == CAL_CMD_COUNT_END)
    {
        if (phase != CAL_PHASE_ACTIVE_COUNT || request->args[0] != calibration_debug_mailbox.prepare_token) return CAL_ACK_BAD_STATE;
        if (!measurement->valid || measurement->error_count != measurement_error_start) return CAL_ACK_MEASUREMENT_INVALID;
        if (measurement->stationary_us < CALIBRATION_STATIONARY_US) return CAL_ACK_NOT_STATIONARY;
        finish_trial(CAL_END_MANUAL, measurement);
        return CAL_ACK_OK;
    }
    if (request->opcode == CAL_CMD_MARKER)
    {
        uint32_t bit;
        if (phase != CAL_PHASE_ACTIVE_RPM || request->args[0] >= CALIBRATION_DEBUG_MARKERS) return CAL_ACK_BAD_STATE;
        bit = 1u << request->args[0];
        if ((requested_marker_mask & bit) != 0u) return CAL_ACK_BAD_STATE;
        requested_marker_mask |= bit;
        pending_marker_mask |= bit;
        return CAL_ACK_OK;
    }
    if (request->opcode == CAL_CMD_RELEASE)
    {
        if (freeze_pending) return CAL_ACK_BAD_STATE; // 等待实际零输出周期，不能清掉尚未冻结的数据。
        if (is_prepared(phase) || is_active(phase)) finish_trial(CAL_END_STOP, measurement);
        else
        {
            control_request_stop();
            if (restore_parameters()) control_set_calibration_reserved(0u);
        }
        if (restore_pending) return CAL_ACK_CONTROL_REJECTED;
        calibration_debug_mailbox.phase = CAL_PHASE_IDLE;
        return CAL_ACK_OK;
    }
    return CAL_ACK_BAD_COMMAND;
}

void calibration_debug_set_critical_hooks(uint32_t (*enter)(void), void (*leave)(uint32_t))
{
    critical_enter = enter;
    critical_leave = leave;
}

void calibration_debug_init(void)
{
    memset((void *)&calibration_debug_mailbox, 0, sizeof(calibration_debug_mailbox));
    memset(calibration_debug_log, 0, sizeof(calibration_debug_log));
    owner_session = last_request_seq = last_stop_seq = last_heartbeat_seq = 0u;
    heartbeat_us = active_start_us = prepared_params_version = measurement_error_start = 0u;
    last_logged_sample = 0u;
    prepared_stop_generation = control_stop_generation();
    pending_marker_mask = requested_marker_mask = completed_marker_mask = 0u;
    heartbeat_seen = restore_pending = freeze_pending = recording = servo_event_pending = 0u;
    calibration_debug_mailbox.abi = CALIBRATION_DEBUG_ABI;
    calibration_debug_mailbox.mailbox_bytes = sizeof(calibration_debug_mailbox);
    calibration_debug_mailbox.build_id = calibration_debug_crc32(calibration_debug_build_stamp, sizeof(calibration_debug_build_stamp) - 1u);
    calibration_debug_mailbox.record_bytes = sizeof(CalibrationDebugRecord);
    calibration_debug_mailbox.record_capacity = CALIBRATION_DEBUG_LOG_CAPACITY;
    calibration_debug_mailbox.features = 15u; // 位3提供舵机事件记录和人工END；邮箱布局保持ABI1，主机按能力区分旧固件。
    calibration_debug_mailbox.log_frozen = 1u;
    CAL_BARRIER();
    calibration_debug_mailbox.magic = CALIBRATION_DEBUG_MAGIC; // 初始化完成后才公开有效magic。
}

uint8_t calibration_debug_busy(void)
{
    uint32_t phase = calibration_debug_mailbox.phase;
    return (uint8_t)(is_prepared(phase) || is_active(phase) || restore_pending || freeze_pending);
}

void calibration_debug_poll(uint32_t now_us)
{
    CalibrationRequest request;
    EncoderMeasurementSnapshot measurement;
    uint32_t seq_before;
    uint32_t crc;
    uint32_t i;
    uint32_t status;
    uint32_t key = lock_debug();
    calibration_debug_mailbox.command_ready = 1u; // 仅实际进入主循环邮箱服务后公开就绪，PIT推进不能替代此门槛。
    encoder_measurement_get_snapshot(&measurement);
    // 空轮次不修改seqlock，否则裸机快速空转会使SWD无法完成一次稳定快照读取。
    if (calibration_debug_mailbox.stop_seq != last_stop_seq ||
        ((is_prepared(calibration_debug_mailbox.phase) || is_active(calibration_debug_mailbox.phase)) &&
         (control_stop_generation() != prepared_stop_generation || control_get_state() == CONTROL_FAULT)))
    {
        snapshot_begin();
        (void)(consume_stop(&measurement) || consume_external_stop(&measurement));
        snapshot_end(); unlock_debug(key); return;
    }
    accept_heartbeat(now_us);
    seq_before = calibration_debug_mailbox.request_seq;
    if (seq_before == 0u || seq_before == last_request_seq) { unlock_debug(key); return; }
    if ((uint32_t)(seq_before - last_request_seq) >= 0x80000000u)
    {
        snapshot_begin();
        publish_ack(seq_before, calibration_debug_mailbox.session_id, CAL_ACK_BAD_ARGUMENT);
        snapshot_end(); unlock_debug(key); return; // 回退序号不得重放旧START，也不倒退已消费基线。
    }
    request.session_id = calibration_debug_mailbox.session_id;
    request.seq = seq_before;
    request.opcode = calibration_debug_mailbox.opcode;
    for (i = 0u; i < 8u; ++i) request.args[i] = calibration_debug_mailbox.args[i];
    crc = calibration_debug_mailbox.request_crc;
    CAL_BARRIER();
    if (calibration_debug_mailbox.request_seq != seq_before) { unlock_debug(key); return; }
    snapshot_begin();
    last_request_seq = seq_before;
    status = calibration_debug_crc32(&request, sizeof(request)) != crc ? CAL_ACK_BAD_CRC :
        process_request(&request, now_us, &measurement);
    if (status == CAL_ACK_OK && request.opcode == CAL_CMD_APPLY_ENCODER)
    {
        encoder_measurement_get_snapshot(&measurement);
        calibration_debug_mailbox.measurement_version = measurement.calibration_version;
        calibration_debug_mailbox.counts_per_rev_left_bits = float_bits(measurement.counts_per_rev_left);
        calibration_debug_mailbox.counts_per_rev_right_bits = float_bits(measurement.counts_per_rev_right);
    }
    publish_ack(request.seq, request.session_id, status);
    snapshot_end();
    unlock_debug(key);
}

void calibration_debug_before_tick(uint32_t now_us, const EncoderMeasurementSnapshot *measurement)
{
    uint32_t phase;
    uint32_t elapsed;
    if (measurement == NULL) return;
    snapshot_begin();
    if (consume_stop(measurement) || consume_external_stop(measurement)) { snapshot_end(); return; }
    accept_heartbeat(now_us);
    phase = calibration_debug_mailbox.phase;
    if (!is_prepared(phase) && !is_active(phase)) { snapshot_end(); return; } // 待冻结只阻止重准备，不改写已锁定原因。
    if ((uint32_t)(now_us - heartbeat_us) >= CALIBRATION_LINK_TIMEOUT_US)
    {
        control_report_link_fault();
        finish_trial(CAL_END_LINK_TIMEOUT, measurement);
        snapshot_end(); return;
    }
    // 首次基线和准备态不触发测量保护；只有已人工开始的组才要求每次采样有效。
    if (is_active(phase) && (!measurement->valid || measurement->error_count != measurement_error_start))
    {
        control_report_invalid_fault();
        finish_trial(CAL_END_MEASUREMENT, measurement);
        snapshot_end(); return;
    }
    elapsed = (uint32_t)(now_us - active_start_us);
    // 手转和舵机只由显式结束/STOP或异常终止；八秒上限仅适用于架空RPM。
    if (phase == CAL_PHASE_ACTIVE_RPM && elapsed >= CALIBRATION_RPM_MAX_US)
    {
        finish_trial(CAL_END_TIME_LIMIT, measurement);
        snapshot_end(); return;
    }
    if (phase == CAL_PHASE_ACTIVE_RPM && recording && calibration_debug_mailbox.log_count >= CALIBRATION_DEBUG_LOG_CAPACITY)
    {
        finish_trial(CAL_END_LOG_FULL, measurement); // 本次PWM计算前停止，不能等写过非零后才宣告冻结。
        snapshot_end(); return;
    }
    if (phase == CAL_PHASE_ACTIVE_RPM)
    {
        uint32_t due = (elapsed >= 2000000u ? 1u : 0u) | (elapsed >= 6000000u ? 2u : 0u);
        pending_marker_mask |= due & ~requested_marker_mask;
        requested_marker_mask |= due; // 手机慢动作参考标记自动在2s/6s附近请求，不要求主机输入。
    }
    snapshot_end();
}

void calibration_debug_after_tick(uint32_t now_us, const EncoderMeasurementSnapshot *measurement,
                                  const ControlSnapshot *output, uint16_t servo_duty)
{
    uint32_t flags;
    uint32_t phase;
    if (measurement == NULL || output == NULL) return;
    snapshot_begin();
    phase = calibration_debug_mailbox.phase;
    if ((phase == CAL_PHASE_ACTIVE_SERVO || phase == CAL_PHASE_ACTIVE_RPM) && output->state != CONTROL_RUNNING)
        finish_trial(CAL_END_CONTROL, measurement);
    flags = (measurement->valid ? 1u : 0u) | (measurement->jitter_warning ? 2u : 0u) |
        (measurement->rpm_valid_100 ? 4u : 0u) | (measurement->rpm_valid_500 ? 8u : 0u) |
        (output->servo_enabled ? 16u : 0u) | (output->state == CONTROL_FAULT ? 32u : 0u);
    if (recording && !calibration_debug_mailbox.log_frozen && measurement->sample_id != last_logged_sample)
    {
        uint32_t index = calibration_debug_mailbox.log_count;
        uint8_t servo_events = (uint8_t)(calibration_debug_mailbox.log_kind == CAL_PHASE_ACTIVE_SERVO);
        // SERVO只记录首次、显式SET首周期、整数duty变化和停机周期；慢速斜坡不因float每tick变化而填满。
        uint8_t event_due = (uint8_t)(!servo_events || index == 0u || servo_event_pending || freeze_pending ||
            (index <= CALIBRATION_DEBUG_LOG_CAPACITY && calibration_debug_log[index - 1u].servo_duty != servo_duty));
        uint32_t limit = servo_events && !freeze_pending ? CALIBRATION_DEBUG_LOG_CAPACITY - 1u : CALIBRATION_DEBUG_LOG_CAPACITY;
        if (event_due && index < limit)
        {
            CalibrationDebugRecord *record = &calibration_debug_log[index];
            record->sample_id = measurement->sample_id;
            record->now_us = now_us;
            record->dt_us = measurement->dt_us;
            record->raw_left = measurement->raw_delta_left;
            record->raw_right = measurement->raw_delta_right;
            record->delta_left = measurement->delta_left;
            record->delta_right = measurement->delta_right;
            record->pwm_left = (uint16_t)output->pwm_left;
            record->pwm_right = (uint16_t)output->pwm_right;
            record->servo_duty = servo_duty;
            record->flags = (uint16_t)flags;
            record->servo_request_us = output->servo_pulse_us_requested;
            CAL_BARRIER();
            calibration_debug_mailbox.log_count = index + 1u;
            last_logged_sample = measurement->sample_id;
            servo_event_pending = 0u;
        }
        else if (event_due && !freeze_pending)
            finish_trial(CAL_END_LOG_FULL, measurement); // SERVO最后一槽留给后续实际零输出周期；不覆盖已有事件。
    }
    if (freeze_pending && output->state != CONTROL_RUNNING && output->pwm_left == 0.0f &&
        output->pwm_right == 0.0f && !output->servo_enabled && servo_duty == 0u)
    {
        if (recording && last_logged_sample == measurement->sample_id)
            save_endpoint(0u, measurement); // 主循环STOP后仍记录惯性增量，冻结终点必须包含最后实际零输出样本。
        ++calibration_debug_mailbox.log_seq;
        CAL_BARRIER();
        calibration_debug_mailbox.log_frozen = 1u;
        CAL_BARRIER();
        ++calibration_debug_mailbox.log_seq;
        freeze_pending = recording = 0u;
    }
    calibration_debug_mailbox.snapshot[CAL_SNAP_NOW_US] = now_us;
    calibration_debug_mailbox.snapshot[CAL_SNAP_SAMPLE_ID] = measurement->sample_id;
    calibration_debug_mailbox.snapshot[CAL_SNAP_DT_US] = measurement->dt_us;
    calibration_debug_mailbox.snapshot[CAL_SNAP_RAW_PAIR] = pair16(measurement->raw_delta_left, measurement->raw_delta_right);
    calibration_debug_mailbox.snapshot[CAL_SNAP_DELTA_PAIR] = pair16(measurement->delta_left, measurement->delta_right);
    write_total(&calibration_debug_mailbox.snapshot[CAL_SNAP_TOTAL_LEFT_LO], &calibration_debug_mailbox.snapshot[CAL_SNAP_TOTAL_LEFT_HI], measurement->total_left);
    write_total(&calibration_debug_mailbox.snapshot[CAL_SNAP_TOTAL_RIGHT_LO], &calibration_debug_mailbox.snapshot[CAL_SNAP_TOTAL_RIGHT_HI], measurement->total_right);
    calibration_debug_mailbox.snapshot[CAL_SNAP_RPM_LEFT] = float_bits(measurement->rpm_left_100);
    calibration_debug_mailbox.snapshot[CAL_SNAP_RPM_RIGHT] = float_bits(measurement->rpm_right_100);
    calibration_debug_mailbox.snapshot[CAL_SNAP_FLAGS] = flags;
    calibration_debug_mailbox.snapshot[CAL_SNAP_CONTROL_STATE] = (uint32_t)output->state;
    calibration_debug_mailbox.snapshot[CAL_SNAP_CONTROL_MODE] = (uint32_t)output->mode;
    calibration_debug_mailbox.snapshot[CAL_SNAP_CONTROL_FAULT] = (uint32_t)output->fault;
    calibration_debug_mailbox.snapshot[CAL_SNAP_PARAMS_VERSION] = output->params_version;
    calibration_debug_mailbox.snapshot[CAL_SNAP_TARGET_LEFT] = float_bits(output->target_left);
    calibration_debug_mailbox.snapshot[CAL_SNAP_TARGET_RIGHT] = float_bits(output->target_right);
    calibration_debug_mailbox.snapshot[CAL_SNAP_PWM_PAIR] = (uint32_t)(uint16_t)output->pwm_left | ((uint32_t)(uint16_t)output->pwm_right << 16);
    calibration_debug_mailbox.snapshot[CAL_SNAP_SERVO_DUTY] = servo_duty;
    calibration_debug_mailbox.snapshot[CAL_SNAP_SERVO_REQUEST_US] = float_bits(output->servo_pulse_us_requested);
    calibration_debug_mailbox.snapshot[CAL_SNAP_SERVO_NOMINAL_US] = float_bits(output->servo_pulse_us_nominal);
    calibration_debug_mailbox.snapshot[CAL_SNAP_HEARTBEAT_AGE_US] = owner_session != 0u ? (uint32_t)(now_us - heartbeat_us) : 0u;
    calibration_debug_mailbox.snapshot[CAL_SNAP_ACTIVE_ELAPSED_US] = is_active(calibration_debug_mailbox.phase) ? (uint32_t)(now_us - active_start_us) : 0u;
    calibration_debug_mailbox.snapshot[CAL_SNAP_STATIONARY_US] = measurement->stationary_us;
    calibration_debug_mailbox.snapshot[CAL_SNAP_LOG_COUNT] = calibration_debug_mailbox.log_count;
    calibration_debug_mailbox.snapshot[CAL_SNAP_LOG_FROZEN] = calibration_debug_mailbox.log_frozen;
    calibration_debug_mailbox.snapshot[CAL_SNAP_MARKER_MASK] = completed_marker_mask;
    calibration_debug_mailbox.rpm_left_500_bits = float_bits(measurement->rpm_left_500);
    calibration_debug_mailbox.rpm_right_500_bits = float_bits(measurement->rpm_right_500);
    calibration_debug_mailbox.measurement_version = measurement->calibration_version;
    calibration_debug_mailbox.counts_per_rev_left_bits = float_bits(measurement->counts_per_rev_left);
    calibration_debug_mailbox.counts_per_rev_right_bits = float_bits(measurement->counts_per_rev_right);
    calibration_debug_mailbox.nominal_us_per_duty_bits = float_bits(control_servo_us_per_duty());
    snapshot_end();
}

int32_t calibration_debug_take_marker_request(void)
{
    uint32_t key = lock_debug();
    uint32_t i;
    for (i = 0u; i < CALIBRATION_DEBUG_MARKERS; ++i)
    {
        if ((pending_marker_mask & (1u << i)) != 0u)
        {
            pending_marker_mask &= ~(1u << i);
            unlock_debug(key);
            return (int32_t)i;
        }
    }
    unlock_debug(key);
    return -1;
}

void calibration_debug_marker_completed(uint32_t index, uint32_t start_us, uint32_t end_us)
{
    uint32_t key;
    if (index >= CALIBRATION_DEBUG_MARKERS) return;
    key = lock_debug();
    snapshot_begin();
    if (calibration_debug_mailbox.phase == CAL_PHASE_ACTIVE_RPM && (requested_marker_mask & (1u << index)) != 0u)
    {
        calibration_debug_mailbox.marker_start_us[index] = start_us;
        calibration_debug_mailbox.marker_end_us[index] = end_us;
        completed_marker_mask |= 1u << index;
        calibration_debug_mailbox.snapshot[CAL_SNAP_MARKER_MASK] = completed_marker_mask;
    }
    snapshot_end();
    unlock_debug(key);
}

#else
void calibration_debug_set_critical_hooks(uint32_t (*enter)(void), void (*leave)(uint32_t)) { (void)enter; (void)leave; }
void calibration_debug_init(void) { }
void calibration_debug_poll(uint32_t now_us) { (void)now_us; }
void calibration_debug_before_tick(uint32_t now_us, const EncoderMeasurementSnapshot *measurement) { (void)now_us; (void)measurement; }
void calibration_debug_after_tick(uint32_t now_us, const EncoderMeasurementSnapshot *measurement, const ControlSnapshot *output, uint16_t servo_duty)
{ (void)now_us; (void)measurement; (void)output; (void)servo_duty; }
uint8_t calibration_debug_busy(void) { return 0u; }
int32_t calibration_debug_take_marker_request(void) { return -1; }
void calibration_debug_marker_completed(uint32_t index, uint32_t start_us, uint32_t end_us) { (void)index; (void)start_us; (void)end_us; }
#endif
