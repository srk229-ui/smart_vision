#include "local_buttons.h"

#define LOCAL_BUTTON_CONFIRM_SCANS (2u) // 外部每五毫秒调用一次，连续两次同态采样确认按下或释放。

typedef struct
{
    uint8_t pressed; // 已确认的按下状态；确认释放前仍用于阻止启动。
    uint8_t armed; // 已确认释放且允许下一次按下事件为 1；上电及事件产生后为 0。
    uint8_t press_scans; // 连续按下采样数，饱和到两次，长按不会溢出或重复产生事件。
    uint8_t release_scans; // 连续释放采样数，饱和到两次，两次之间出现按下则清零。
} LocalButtonState;

static LocalButtonState start_button; // KEY1 独立消抖状态，仅由调用扫描函数的上下文更新。
static LocalButtonState stop_button; // KEY2 独立消抖状态，停止屏蔽使用本次输入与确认状态。

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新单个按键的连续采样和重新武装状态
// 参数说明     button          本按键的持久消抖状态
// 参数说明     pressed         已归一化的按下标志，1 为按下、0 为释放
// 返回参数     uint8_t         本次首次确认按下为 1，否则为 0
// 备注信息     未确认释放前的上电长按不产生事件，被外层屏蔽的事件同样只消费一次
//-------------------------------------------------------------------------------------------------------------------
static uint8_t scan_button(LocalButtonState *button, uint8_t pressed)
{
    if (pressed)
    {
        button->release_scans = 0u;
        if (button->press_scans < LOCAL_BUTTON_CONFIRM_SCANS) ++button->press_scans;
        if (button->press_scans == LOCAL_BUTTON_CONFIRM_SCANS)
        {
            button->pressed = 1u;
            if (button->armed)
            {
                button->armed = 0u; // 一次按下只消费一次，必须重新确认释放才能再次触发。
                return 1u;
            }
        }
    }
    else
    {
        button->press_scans = 0u;
        if (button->release_scans < LOCAL_BUTTON_CONFIRM_SCANS) ++button->release_scans;
        if (button->release_scans == LOCAL_BUTTON_CONFIRM_SCANS)
        {
            button->pressed = 0u;
            button->armed = 1u;
        }
    }
    return 0u;
}

void local_buttons_init(void)
{
    LocalButtonState initial = {0u, 0u, 0u, 0u}; // 上电不武装，先确认释放，避免按住启动键上电即发车。
    start_button = initial;
    stop_button = initial;
}

uint8_t local_buttons_scan(uint8_t start_pressed, uint8_t stop_pressed)
{
    uint8_t start_event = scan_button(&start_button, (uint8_t)(start_pressed != 0u));
    uint8_t stop_event = scan_button(&stop_button, (uint8_t)(stop_pressed != 0u));

    if (stop_event) return LOCAL_BUTTON_EVENT_STOP; // 同一扫描确认双键时只返回停止，启动事件已消费。
    // 停止的首次低电平及释放消抖期间都屏蔽启动，避免在停止键状态转换边界意外发车。
    if (start_event && !stop_pressed && !stop_button.pressed) return LOCAL_BUTTON_EVENT_START;
    return 0u;
}
