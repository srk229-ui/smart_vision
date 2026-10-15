#ifndef CALIBRATION_DEBUG_H
#define CALIBRATION_DEBUG_H // M3运行态内存标定接口防重复包含。

#include <stddef.h>
#include <stdint.h>
#include "calibration_config.h"
#include "control_types.h"
#include "encoder_measurement.h"

#define CALIBRATION_DEBUG_MAGIC (0x43414c33u) // 邮箱标识，主机须同时核对ELF、ABI和构建CRC。
#define CALIBRATION_DEBUG_ABI (1u) // 小端固定字段协议版本，布局变化必须递增。
#define CALIBRATION_DEBUG_LOG_CAPACITY (1024u) // 最多1024条32字节记录；RPM逐周期，舵机按事件并预留停止槽。
#define CALIBRATION_DEBUG_MARKERS (8u) // 最多8个TFT标记，索引0至7，不冒充实际屏幕光学时刻。

typedef enum
{
    CAL_CMD_PREPARE_COUNT = 1, // args0=人工确认1；无输出手转准备，仅由显式开始/结束界定测量。
    CAL_CMD_PREPARE_SERVO = 2, // args0至3=float中位/最小/最大us/速率us/s；4=参数版本；5=人工确认1。
    CAL_CMD_PREPARE_RPM = 3, // args0/1=float左右counts/10ms目标；2=参数版本；3=人工确认1。
    CAL_CMD_START = 4, // args0=prepare_token，只能人工另一次提交，启动舵机或RPM。
    CAL_CMD_SET_SERVO = 5, // args0=float请求us，只在本会话SERVO_CAL活动时生效，不续期。
    CAL_CMD_COUNT_START = 6, // args0=prepare_token；两轮连续100ms零增量时锁定起点。
    CAL_CMD_COUNT_END = 7, // args0=prepare_token；静止100ms后冻结净计数终点。
    CAL_CMD_MARKER = 8, // args0=0至7标记号；RPM活动时请求一次主循环TFT标记。
    CAL_CMD_APPLY_ENCODER = 9, // args0/1=float左右有效counts/rev；2=测量参数版本；3=人工确认1。
    CAL_CMD_QUERY_US = 10, // args0=float us；结果放ACK结果位，不作为执行器命令。
    CAL_CMD_QUERY_ANGLE = 11, // args0=float轮角deg；1=LEFT0/RIGHT1；结果为表推算us。
    CAL_CMD_RELEASE = 12, // 停止并释放本会话准备权，不启动正常循迹。
    CAL_CMD_CLEAR_FAULT = 13, // 人工清故障请求；仍须跨过零输出周期，不自动恢复活动。
    CAL_CMD_SERVO_END = 14 // args0=prepare_token；人工结束本会话舵机组，实际零输出周期后冻结。
} CalibrationDebugCommand;

typedef enum
{
    CAL_PHASE_IDLE = 0, // 无标定准备或活动，普通控制仍按原规则启动。
    CAL_PHASE_PREPARED_COUNT = 1, // 手转准备已确认，保持零执行器输出。
    CAL_PHASE_PREPARED_SERVO = 2, // 舵机范围已准备，尚未允许脉冲输出。
    CAL_PHASE_PREPARED_RPM = 3, // 临时RPM参数已应用，尚未启动BENCH。
    CAL_PHASE_ACTIVE_COUNT = 4, // 正在手转净计数，不开启执行器。
    CAL_PHASE_ACTIVE_SERVO = 5, // 舵机单独活动，统一控制器保持两电机为零。
    CAL_PHASE_ACTIVE_RPM = 6, // 已人工启动的八秒BENCH，逐周期记录测量。
    CAL_PHASE_COMPLETE = 7, // 终点或时限结束，数据冻结并释放准备权。
    CAL_PHASE_ABORTED = 8 // STOP、失联、测量异常或控制故障，数据冻结且不自动恢复。
} CalibrationDebugPhase;

typedef enum
{
    CAL_ACK_OK = 0, // 命令接受；START接受不证明引脚或机械已动作。
    CAL_ACK_BAD_CRC = 1, // 完整请求CRC错误，不执行载荷。
    CAL_ACK_BAD_SESSION = 2, // 零会话或非准备拥有者会话，不执行。
    CAL_ACK_BAD_STATE = 3, // 当前阶段或控制状态不支持请求。
    CAL_ACK_BAD_ARGUMENT = 4, // 非法、非有限或未人工确认的输入。
    CAL_ACK_STALE_VERSION = 5, // 控制/测量参数版本过期。
    CAL_ACK_CONTROL_REJECTED = 6, // 统一控制API拒绝启动、切参或清故障。
    CAL_ACK_NOT_STATIONARY = 7, // 两轮尚未连续100ms严格零增量。
    CAL_ACK_MEASUREMENT_INVALID = 8, // 计数/时间异常，手转组不能接受。
    CAL_ACK_UNCALIBRATED = 9, // 无有效实测转向表或查询越界。
    CAL_ACK_BAD_COMMAND = 10, // 未定义操作码，不能当作START。
    CAL_ACK_STOPPED = 11 // STOP优先取消尚未执行请求。
} CalibrationDebugAck;

typedef enum
{
    CAL_END_NONE = 0, // 仍准备/活动，尚无冻结原因。
    CAL_END_MANUAL = 1, // 手转或舵机人工正常结束，实际零输出周期后冻结。
    CAL_END_STOP = 2, // 人工STOP或释放，不证明数据为有效实测。
    CAL_END_LINK_TIMEOUT = 3, // 当前会话500ms无新心跳，锁存链路故障。
    CAL_END_TIME_LIMIT = 4, // RPM八秒上限，不自动续接；手转与舵机无固定时限。
    CAL_END_MEASUREMENT = 5, // 异常计数/采样时间，锁存非法值故障。
    CAL_END_CONTROL = 6, // 控制器自行停止或故障，包括V1 STOP。
    CAL_END_LOG_FULL = 7 // 有限日志容量耗尽，停止且不覆盖旧数据。
} CalibrationDebugEnd;

typedef enum
{
    CAL_SNAP_NOW_US = 0, // 本次GPT1低32位时间us。
    CAL_SNAP_SAMPLE_ID = 1, // 唯一编码器采样序号。
    CAL_SNAP_DT_US = 2, // 本次真实采样间隔us。
    CAL_SNAP_RAW_PAIR = 3, // 左低16/右高16位原始带符号差分。
    CAL_SNAP_DELTA_PAIR = 4, // 左低16/右高16位方向校正差分。
    CAL_SNAP_TOTAL_LEFT_LO = 5, // 左净累计int64低32位。
    CAL_SNAP_TOTAL_LEFT_HI = 6, // 左净累计int64高32位。
    CAL_SNAP_TOTAL_RIGHT_LO = 7, // 右净累计int64低32位。
    CAL_SNAP_TOTAL_RIGHT_HI = 8, // 右净累计int64高32位。
    CAL_SNAP_RPM_LEFT = 9, // float位型，约100ms左输出轮RPM；有效性查flags。
    CAL_SNAP_RPM_RIGHT = 10, // float位型，约100ms右输出轮RPM。
    CAL_SNAP_FLAGS = 11, // 位0样本有效/1抖动/2RPM100有效/3RPM500有效/4舵机输出/5控制故障。
    CAL_SNAP_CONTROL_STATE = 12, // ControlState数值。
    CAL_SNAP_CONTROL_MODE = 13, // ControlMode数值。
    CAL_SNAP_CONTROL_FAULT = 14, // ControlFault数值。
    CAL_SNAP_PARAMS_VERSION = 15, // 实际控制参数版本。
    CAL_SNAP_TARGET_LEFT = 16, // float位型，左有效counts/10ms目标。
    CAL_SNAP_TARGET_RIGHT = 17, // float位型，右有效counts/10ms目标。
    CAL_SNAP_PWM_PAIR = 18, // 左低16/右高16位实际下发整数PWM。
    CAL_SNAP_SERVO_DUTY = 19, // 本次统一ISR使用的舵机占空计数。
    CAL_SNAP_SERVO_REQUEST_US = 20, // float位型，请求脉宽us，不是实测波形。
    CAL_SNAP_SERVO_NOMINAL_US = 21, // float位型，按整数duty反算的标称脉宽us。
    CAL_SNAP_HEARTBEAT_AGE_US = 22, // 当前会话最后新鲜心跳至本次采样的时间us。
    CAL_SNAP_ACTIVE_ELAPSED_US = 23, // 本次活动GPT真实时间差us，SET不复位。
    CAL_SNAP_STATIONARY_US = 24, // 两轮连续严格零增量时长us。
    CAL_SNAP_LOG_COUNT = 25, // 已存样本数，最大1024。
    CAL_SNAP_LOG_FROZEN = 26, // 1为冻结，不可将活动中读回当完整试验。
    CAL_SNAP_MARKER_MASK = 27, // 位0至7表示对应TFT软件标记已完成，时间0也可能为有效值。
    CAL_SNAP_MAP_VERSION = 28, // 停车查询时记录的转向表版本。
    CAL_SNAP_MAP_VALID = 29, // 表经软件校验且人工测量确认为1。
    CAL_SNAP_RESULT_0 = 30, // QUERY返回float位型：左轮角或反查us。
    CAL_SNAP_RESULT_1 = 31 // QUERY_US返回右轮角float位型。
} CalibrationDebugSnapshotIndex;

typedef struct
{
    uint32_t sample_id; // 偏移0，唯一编码器采样序号。
    uint32_t now_us; // 偏移4，本次GPT1低32位us。
    uint32_t dt_us; // 偏移8，真实采样间隔us。
    int16_t raw_left; // 偏移12，左未经符号校正的模差分counts。
    int16_t raw_right; // 偏移14，右原始模差分counts。
    int16_t delta_left; // 偏移16，左方向校正净增量counts。
    int16_t delta_right; // 偏移18，右方向校正净增量counts。
    uint16_t pwm_left; // 偏移20，左实际PWM整数命令。
    uint16_t pwm_right; // 偏移22，右实际PWM整数命令。
    uint16_t servo_duty; // 偏移24，舵机实际整数占空命令。
    uint16_t flags; // 偏移26，含义同CAL_SNAP_FLAGS低16位。
    float servo_request_us; // 偏移28，请求脉宽us，不是前轮实际角度。
} CalibrationDebugRecord;

typedef struct
{
    uint32_t magic; // 偏移0，固定邮箱magic。
    uint32_t abi; // 偏移4，固定wire ABI版本。
    uint32_t mailbox_bytes; // 偏移8，完整邮箱416字节。
    uint32_t build_id; // 偏移12，构建stamp去终止NUL后的IEEE CRC32。
    uint32_t record_bytes; // 偏移16，记录32字节。
    uint32_t record_capacity; // 偏移20，记录容量1024。
    uint32_t features; // 偏移24，位0邮箱、位1 STOP确认、位2主循环就绪、位3舵机无时限/事件记录/END。
    uint32_t command_ready; // 偏移28，初始化返回且主循环进入标定poll后为1；feature位2有效时使用。
    uint32_t session_id; // 偏移32，主机非零会话；CRC覆盖32至75。
    uint32_t request_seq; // 偏移36，主机最后单独提交的非零32位序号。
    uint32_t opcode; // 偏移40，CalibrationDebugCommand。
    uint32_t args[8]; // 偏移44，uint32或float位型，未用项必须0。
    uint32_t request_crc; // 偏移76，标准IEEE CRC32覆盖session/seq/opcode/args共44字节。
    uint32_t stop_seq; // 偏移80，独立32位STOP提交，优先于普通命令。
    uint32_t heartbeat_session_id; // 偏移84，心跳所属会话，先于heartbeat_seq写入。
    uint32_t heartbeat_seq; // 偏移88，100ms更新的新递增序号，旧序号不能续租。
    uint32_t reserved_host; // 偏移92，主机保留为0。
    uint32_t ack_seq; // 偏移96，固件最后发布；主机匹配后才可编辑下一个请求。
    uint32_t ack_status; // 偏移100，CalibrationDebugAck。
    uint32_t ack_session_id; // 偏移104，已消费请求的会话。
    uint32_t prepare_token; // 偏移108，成功准备请求序号，START须回带。
    uint32_t phase; // 偏移112，CalibrationDebugPhase。
    uint32_t end_reason; // 偏移116，CalibrationDebugEnd。
    uint32_t snapshot_seq; // 偏移120，固件快照seqlock；奇数/变化时重读。
    uint32_t stop_ack_seq; // 偏移124，已产生STOP效果的stop_seq；仍须等待实际零输出周期冻结。
    uint32_t snapshot[32]; // 偏移128，索引定义见CalibrationDebugSnapshotIndex。
    uint32_t count_start_us; // 偏移256，静止门槛通过后的起点采样时间。
    uint32_t count_end_us; // 偏移260，静止门槛通过后的终点采样时间。
    uint32_t count_start_left_lo; // 偏移264，左起点int64低32位。
    uint32_t count_start_left_hi; // 偏移268，左起点int64高32位。
    uint32_t count_start_right_lo; // 偏移272，右起点int64低32位。
    uint32_t count_start_right_hi; // 偏移276，右起点int64高32位。
    uint32_t count_end_left_lo; // 偏移280，左终点int64低32位。
    uint32_t count_end_left_hi; // 偏移284，左终点int64高32位。
    uint32_t count_end_right_lo; // 偏移288，右终点int64低32位。
    uint32_t count_end_right_hi; // 偏移292，右终点int64高32位。
    uint32_t marker_start_us[CALIBRATION_DEBUG_MARKERS]; // 偏移296，TFT开始软件时戳；未完成为0。
    uint32_t marker_end_us[CALIBRATION_DEBUG_MARKERS]; // 偏移328，TFT调用结束时戳；不代表光学亮灭。
    uint32_t log_seq; // 偏移360，冻结元数据seqlock，奇数时不能接受读回。
    uint32_t log_run_id; // 偏移364，本次成功人工启动序号。
    uint32_t log_kind; // 偏移368，ACTIVE_COUNT/SERVO/RPM数值。
    uint32_t log_params_version; // 偏移372，启动实际控制参数版本。
    uint32_t log_calibration_version; // 偏移376，启动counts/rev版本。
    uint32_t log_count; // 偏移380，已存记录数，手转组仅保留起止摘要。
    uint32_t log_frozen; // 偏移384，冻结完成为1，否则0。
    uint32_t log_end_reason; // 偏移388，冻结原因。
    uint32_t rpm_left_500_bits; // 偏移392，float位型，左输出轮约500ms RPM。
    uint32_t rpm_right_500_bits; // 偏移396，float位型，右输出轮约500ms RPM。
    uint32_t measurement_version; // 偏移400，当前counts/rev参数版本，APPLY时回带。
    uint32_t counts_per_rev_left_bits; // 偏移404，float位型，左轮实测N，0表示未标定。
    uint32_t counts_per_rev_right_bits; // 偏移408，float位型，右轮实测N。
    uint32_t nominal_us_per_duty_bits; // 偏移412，float位型，脉宽标称us/占空计数系数。
} CalibrationDebugMailbox;

typedef char CalibrationDebugRecordSizeCheck[(sizeof(CalibrationDebugRecord) == 32u) ? 1 : -1]; // C99静态校验记录大小。
typedef char CalibrationDebugMailboxSizeCheck[(sizeof(CalibrationDebugMailbox) == 416u) ? 1 : -1]; // C99静态校验邮箱大小。
typedef char CalibrationDebugHostOffsetCheck[(offsetof(CalibrationDebugMailbox, request_crc) == 76u) ? 1 : -1]; // 校验主机提交位置。
typedef char CalibrationDebugSnapshotOffsetCheck[(offsetof(CalibrationDebugMailbox, snapshot) == 128u) ? 1 : -1]; // 校验实时快照位置。

#if CALIBRATION_DEBUG_ENABLE
extern volatile CalibrationDebugMailbox calibration_debug_mailbox; // 专用DTCM邮箱，只允许主机写host字段。
extern CalibrationDebugRecord calibration_debug_log[CALIBRATION_DEBUG_LOG_CAPACITY]; // 冻结后由主机只读导出。
extern const char calibration_debug_build_stamp[]; // 当前ELF构建标识；主机读取初始化内容并计算CRC。
#endif

// 函数简介     安装诊断模块短临界区，初始化时调用
// 参数说明     enter/leave    保存和恢复原中断状态的成对函数，可同时为空
// 返回参数     void
void calibration_debug_set_critical_hooks(uint32_t (*enter)(void), void (*leave)(uint32_t));
// 函数简介     初始化邮箱和有限记录，上电不准备、不启动
// 参数说明     void
// 返回参数     void
void calibration_debug_init(void);
// 函数简介     主循环消费完整邮箱请求，通过统一控制API执行
// 参数说明     now_us          GPT1真实us时间，允许uint32回绕
// 返回参数     void
void calibration_debug_poll(uint32_t now_us);
// 函数简介     PIT输出计算前检查STOP、心跳、时限和测量异常
// 参数说明     now_us/measurement    本次唯一采样时刻及一致快照
// 返回参数     void
void calibration_debug_before_tick(uint32_t now_us, const EncoderMeasurementSnapshot *measurement);
// 函数简介     PIT写PWM后记录同一采样及实际输出并发布seqlock快照
// 参数说明     now_us/measurement/control/servo_duty    时间、采样、统一输出及整数占空
// 返回参数     void
void calibration_debug_after_tick(uint32_t now_us, const EncoderMeasurementSnapshot *measurement,
                                  const ControlSnapshot *control, uint16_t servo_duty);
// 函数简介     判断本地标定是否占有准备或活动权
// 参数说明     void
// 返回参数     uint8_t         忙1/空闲0；关闭构建恒为0
uint8_t calibration_debug_busy(void);
// 函数简介     主循环领取一次TFT标记请求
// 参数说明     void
// 返回参数     int32_t         标记0至7，无请求为-1
int32_t calibration_debug_take_marker_request(void);
// 函数简介     记录TFT标记调用的起止软件时间
// 参数说明     index/start_us/end_us    已领取标记及真实us时间
// 返回参数     void
void calibration_debug_marker_completed(uint32_t index, uint32_t start_us, uint32_t end_us);
// 函数简介     计算协议标准IEEE CRC32
// 参数说明     bytes/length    连续小端wire字节和长度
// 返回参数     uint32_t        zlib兼容CRC32
uint32_t calibration_debug_crc32(const void *bytes, uint32_t length);

#endif
