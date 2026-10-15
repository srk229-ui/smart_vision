#include "local_buttons.h"

#include <assert.h>
#include <stdio.h>

static void ready(void)
{
    local_buttons_init();
    assert(local_buttons_scan(0u, 0u) == 0u);
    assert(local_buttons_scan(0u, 0u) == 0u);
}

static void expect_hold(uint8_t start, uint8_t stop)
{
    unsigned i;
    for (i = 0u; i < 1000u; ++i) assert(local_buttons_scan(start, stop) == 0u);
}

int main(void)
{
    // 上电按住任一按键均不触发，短暂释放不足两次也不能武装。
    local_buttons_init();
    expect_hold(1u, 1u);
    assert(local_buttons_scan(0u, 0u) == 0u);
    expect_hold(1u, 1u);
    assert(local_buttons_scan(0u, 0u) == 0u);
    assert(local_buttons_scan(0u, 0u) == 0u);
    assert(local_buttons_scan(1u, 0u) == 0u);
    assert(local_buttons_scan(1u, 0u) == LOCAL_BUTTON_EVENT_START);
    expect_hold(1u, 0u);

    // 按下抖动不能触发；确认按下只发一次，释放抖动不能重新武装。
    ready();
    assert(local_buttons_scan(1u, 0u) == 0u);
    assert(local_buttons_scan(0u, 0u) == 0u);
    assert(local_buttons_scan(1u, 0u) == 0u);
    assert(local_buttons_scan(1u, 0u) == LOCAL_BUTTON_EVENT_START);
    expect_hold(1u, 0u);
    assert(local_buttons_scan(0u, 0u) == 0u);
    expect_hold(1u, 0u);
    assert(local_buttons_scan(0u, 0u) == 0u);
    assert(local_buttons_scan(0u, 0u) == 0u);
    assert(local_buttons_scan(3u, 0u) == 0u);
    assert(local_buttons_scan(3u, 0u) == LOCAL_BUTTON_EVENT_START);

    // 停止在按下确认时触发，不等松手，长按不重复；非零输入按布尔处理。
    ready();
    assert(local_buttons_scan(0u, 2u) == 0u);
    assert(local_buttons_scan(0u, 2u) == LOCAL_BUTTON_EVENT_STOP);
    expect_hold(0u, 2u);
    assert(local_buttons_scan(0u, 0u) == 0u);
    assert(local_buttons_scan(0u, 0u) == 0u);
    assert(local_buttons_scan(0u, 1u) == 0u);
    assert(local_buttons_scan(0u, 1u) == LOCAL_BUTTON_EVENT_STOP);

    // 同时确认双键只停止；随后单独松开停止键不会重放被丢弃的启动。
    ready();
    assert(local_buttons_scan(1u, 1u) == 0u);
    assert(local_buttons_scan(1u, 1u) == LOCAL_BUTTON_EVENT_STOP);
    expect_hold(1u, 1u);
    assert(local_buttons_scan(1u, 0u) == 0u);
    assert(local_buttons_scan(1u, 0u) == 0u);
    expect_hold(1u, 0u);

    // 停止键按住时新按启动被消费且不发车，停止释放后仍需重新按启动。
    ready();
    assert(local_buttons_scan(0u, 1u) == 0u);
    assert(local_buttons_scan(0u, 1u) == LOCAL_BUTTON_EVENT_STOP);
    assert(local_buttons_scan(1u, 1u) == 0u);
    assert(local_buttons_scan(1u, 1u) == 0u);
    assert(local_buttons_scan(1u, 0u) == 0u);
    assert(local_buttons_scan(1u, 0u) == 0u);
    expect_hold(1u, 0u);
    assert(local_buttons_scan(0u, 0u) == 0u);
    assert(local_buttons_scan(0u, 0u) == 0u);
    assert(local_buttons_scan(1u, 0u) == 0u);
    assert(local_buttons_scan(1u, 0u) == LOCAL_BUTTON_EVENT_START);

    // 停止首次采样按下即屏蔽同次确认的启动，不等待停止自身第二次采样。
    ready();
    assert(local_buttons_scan(1u, 0u) == 0u);
    assert(local_buttons_scan(1u, 1u) == 0u);
    assert(local_buttons_scan(1u, 1u) == LOCAL_BUTTON_EVENT_STOP);

    // 停止已确认按下、仅释放一次时仍屏蔽启动，直到重新按启动才有新事件。
    ready();
    assert(local_buttons_scan(0u, 1u) == 0u);
    assert(local_buttons_scan(0u, 1u) == LOCAL_BUTTON_EVENT_STOP);
    assert(local_buttons_scan(1u, 1u) == 0u);
    assert(local_buttons_scan(1u, 0u) == 0u);
    assert(local_buttons_scan(1u, 0u) == 0u);
    expect_hold(1u, 0u);

    // 重新初始化清除武装状态，不能继承上次会话的按下资格。
    ready();
    local_buttons_init();
    expect_hold(1u, 0u);
    puts("PASS local buttons: debounce, held keys, boot release and stop priority");
    return 0;
}
