"""生成离线教学时间轴：自包含HTML与两张原生SVG，不读取车辆数据。"""

from html import escape
from pathlib import Path
import argparse
import json


ROOT = Path(__file__).resolve().parents[1]
COLORS = {
    "bg": "#0b1220", "panel": "#131f32", "line": "#2c3c54",
    "ink": "#ecf4ff", "muted": "#99adc7", "camera": "#68d7e5",
    "csi": "#f29ab6", "pit": "#ffb277", "vision": "#b6a1ff",
    "io": "#79aaff", "servo": "#8ae0b2", "motor": "#e99bd2",
    "other": "#566a86", "cursor": "#ffffff",
}
WIDTH = 1480
PLOT_LEFT = 310
PLOT_RIGHT = 1430


def build_model():
    """固定示例事件；耗时、相位与脉宽均为教学假设。"""
    camera = [{"source": f"F{i}", "start": i * 20,
               "end": (i + 1) * 20} for i in range(3)]
    ticks = []
    services = []
    for due in range(0, 61, 10):
        delay = 0.05 if due in (20, 40, 60) else 0
        source = "F-1" if due <= 20 else "F0" if due <= 40 else "F1"
        tick = {"due": due, "start": round(due + delay, 2),
                "end": round(due + delay + 0.20, 2), "source": source}
        ticks.append(tick)
        if delay:
            services.append({"kind": "csi", "start": due,
                             "end": round(due + delay, 2),
                             "label": f"CSI：完成帧F{due // 20 - 1}"})
        services.append({"kind": "pit", "start": tick["start"],
                         "end": tick["end"],
                         "label": f"PIT {due} ms：视觉依据{source}，本次编码器用于速度PI"})
    vision = [{"source": "F0", "start": 20.25, "end": 22.25},
              {"source": "F1", "start": 40.25, "end": 42.25}]
    for frame in vision:
        services.append({"kind": "vision", "start": frame["start"],
                         "end": frame["end"], "label": f"视觉处理{frame['source']}：发布后更新转向目标"})
        services.append({"kind": "io", "start": frame["end"],
                         "end": frame["end"] + 1,
                         "label": "一次运行收发服务示例：1 ms，非每帧波形发送"})
    pulses = [{"start": 5, "end": 6.5, "source": "F-1"},
              {"start": 25, "end": 26.5, "source": "F-1"},
              {"start": 45, "end": 46.5, "source": "F0"}]
    return {"camera": camera, "ticks": ticks, "vision": vision,
            "services": sorted(services, key=lambda event: event["start"]),
            "pulses": pulses, "plotLeft": PLOT_LEFT, "plotRight": PLOT_RIGHT}


MODEL = build_model()


def state_at(time_ms, model=MODEL):
    """完成事件在结束时成立，执行与高脉冲区间采用左闭右开。"""
    time_ms = max(0, min(60, float(time_ms)))

    def latest(events, predicate):
        return next((event for event in reversed(events) if predicate(event)), None)

    return {
        "time": time_ms,
        "camera": latest(model["camera"], lambda event: event["end"] <= time_ms),
        "vision": latest(model["vision"], lambda event: event["end"] <= time_ms),
        "pit": latest(model["ticks"], lambda event: event["end"] <= time_ms),
        "servo": latest(model["pulses"], lambda event: event["start"] <= time_ms),
        "activity": next((event for event in model["services"]
                          if event["start"] <= time_ms < event["end"]), None),
        "pending": next((event for event in model["ticks"]
                         if event["due"] <= time_ms < event["start"]), None),
    }


def text(x, y, content, size=15, fill="ink", anchor="start", weight=400, extra=""):
    color = COLORS.get(fill, fill)
    return (f'<text x="{x:.2f}" y="{y:.2f}" font-size="{size}" '
            f'fill="{color}" text-anchor="{anchor}" font-weight="{weight}" {extra}>'
            f'{escape(str(content))}</text>')


def rect(x, y, width, height, fill, stroke=None, radius=8, extra=""):
    color = COLORS.get(fill, fill)
    border = COLORS.get(stroke, stroke) if stroke else "none"
    return (f'<rect x="{x:.2f}" y="{y:.2f}" width="{width:.2f}" '
            f'height="{height:.2f}" rx="{radius}" fill="{color}" '
            f'stroke="{border}" {extra}/>')


def line(x1, y1, x2, y2, color="line", width=1, extra=""):
    return (f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" '
            f'stroke="{COLORS.get(color, color)}" stroke-width="{width}" {extra}/>')


def begin_svg(height, title, description, prefix):
    return [f'<svg xmlns="http://www.w3.org/2000/svg" width="{WIDTH}" height="{height}" '
            f'viewBox="0 0 {WIDTH} {height}" role="img" aria-labelledby="{prefix}-title {prefix}-desc">',
            f'<title id="{prefix}-title">{escape(title)}</title><desc id="{prefix}-desc">{escape(description)}</desc>',
            f'<defs><pattern id="{prefix}-other-hatch" width="9" height="9" patternUnits="userSpaceOnUse">'
            '<rect width="9" height="9" fill="#233149"/>'
            '<path d="M-2 2L2-2M0 9L9 0M7 11L11 7" stroke="#41546f" stroke-width="1"/>'
            f'</pattern><pattern id="{prefix}-motor-pattern" width="5" height="30" patternUnits="userSpaceOnUse">'
            '<rect width="5" height="30" fill="#261f36"/>'
            '<path d="M1 26V4H3V26" fill="none" stroke="#e99bd2" stroke-width="0.8" opacity="0.55"/>'
            '</pattern></defs>',
            '<g font-family="Segoe UI, Microsoft YaHei, PingFang SC, sans-serif">',
            rect(0, 0, WIDTH, height, "bg", radius=16)]


def lane_header(title, subtitle, kind, y, color):
    return [text(28, y + 23, title, 17, "ink", weight=650),
            text(28, y + 45, subtitle, 12, "muted"),
            rect(234, y + 7, 54, 23, "panel", "line", 6),
            text(261, y + 23, kind, 11, color, "middle", 600)]


def cursor_svg(x, top, bottom, time_ms=26, prefix="overview"):
    return [f'<g id="{prefix}-cursor" transform="translate({x:.2f} 0)">',
            line(0, top, 0, bottom, "cursor", 1.5, 'stroke-dasharray="4 5" opacity="0.9"'),
            rect(-47, top - 31, 94, 24, "ink", radius=6),
            text(0, top - 14, f"{time_ms:.2f} ms", 13, "bg", "middle", 700,
                 f'id="{prefix}-cursor-label"'), '</g>']


def overview_svg():
    """总览共享横轴，按教学示例时间比例画CPU条块与硬件周期。"""
    def x(value):
        return PLOT_LEFT + value / 60 * (PLOT_RIGHT - PLOT_LEFT)

    parts = begin_svg(922, "采集、计算与硬件脉冲的公共时间轴",
                      "离线教学示例：26毫秒时视觉F0已发布，但最近完成PIT和最近舵机脉冲仍使用F-1作为视觉依据。", "overview")
    parts += [text(28, 36, "同一条时间轴，看见帧来源逐级向后传递", 25, weight=700),
              rect(1230, 16, 222, 29, "panel", "line", 6),
              text(1341, 36, "离线示意 · 未实测", 14, "pit", "middle", 650),
              text(28, 64, "车已 RUNNING / TRACK；窗口前 F-1 有效。横轴是独立教学经过时间，不是 control_now_ms。", 14, "muted"),
              rect(28, 83, 1424, 71, "panel", "line"),
              text(44, 103, "代码已配置/据其换算（未实测）：PIT 10 ms / 100 Hz；相机50 FPS / 20 ms；舵机50 Hz / 20 ms；电机17 kHz / 约58.8 μs。", 12, "camera", weight=600),
              text(44, 124, "教学假设：初始F-1；CSI 0.05 ms；整轮PIT CPU 0.20 ms；视觉CPU 2 ms；收发1 ms；舵机相位5/25/45 ms、脉宽1.5 ms。", 12, "pit"),
              text(44, 145, "按假设推演：视觉22.25/42.25 ms发布、余量6.75 ms、26/46 ms帧来源。代码预算：单轮运行收发截止2 ms，物理最坏耗时待测。", 12, "vision")]
    legend = [("camera", "相机 · 硬件"), ("csi", "CSI · CPU中断"),
              ("pit", "PIT · CPU中断"), ("vision", "视觉 · CPU"),
              ("io", "收发 · CPU"), ("servo", "舵机 · 硬件"), ("motor", "电机 · 硬件")]
    for index, (color, label) in enumerate(legend):
        left = 31 + index * 204
        parts += [rect(left, 172, 11, 11, color, radius=3), text(left + 19, 183, label, 13, "muted")]
    parts += [text(28, 226, "教学经过时间  /  ms", 13, "muted")]
    for value in range(0, 61, 10):
        parts += [line(x(value), 234, x(value), 793, "line", 1,
                       'stroke-dasharray="3 5"' if value % 20 else ''),
                  text(x(value), 226, value, 14, "muted", "middle")]

    parts += lane_header("相机帧周期", "代码50 FPS → 20 ms（换算，未测）", "硬件", 254, "camera")
    for frame in MODEL["camera"]:
        parts += [rect(x(frame["start"]), 256, x(frame["end"]) - x(frame["start"]), 51,
                       "#243449", "#4d7087", 6),
                  text(x((frame["start"] + frame["end"]) / 2), 278,
                       f"{frame['source']} 帧周期 / 采集传输", 15, "camera", "middle", 650),
                  text(x((frame["start"] + frame["end"]) / 2), 298,
                       f"{frame['start']}—{frame['end']} ms", 12, "muted", "middle"),
                  line(x(frame["end"]), 253, x(frame["end"]), 312, "camera", 2)]

    parts += lane_header("CPU执行", "CPU条块耗时：教学假设，非实测", "CPU", 332, "pit")
    parts += [rect(PLOT_LEFT, 349, PLOT_RIGHT - PLOT_LEFT, 35, "url(#overview-other-hatch)", radius=4)]
    for event in MODEL["services"]:
        if event["start"] >= 60:
            continue
        left = x(event["start"])
        width = x(min(event["end"], 60)) - left
        parts += [f'<g><title>{escape(event["label"])}；{event["start"]:.2f}—{event["end"]:.2f} ms</title>',
                  rect(left, 349, width, 35, event["kind"], radius=0)]
        if event["kind"] == "vision":
            parts.append(text(left + width / 2, 372, "F0" if event["start"] < 30 else "F1", 12, "bg", "middle", 700))
        if event["kind"] == "io":
            parts.append(text(left + width / 2, 371, "IO", 10, "bg", "middle", 700))
        parts.append('</g>')
    parts += [text(PLOT_LEFT + 10, 407, "窄条按比例绘制；20 / 40 / 60 ms边界示例先CSI再PIT。20—30 ms详见下方放大图。", 12, "muted")]

    parts += lane_header("已发布视觉", "发布即更新转向目标（TRACK）", "CPU", 437, "vision")
    for start, end, source in [(0, 22.25, "F-1"), (22.25, 42.25, "F0"), (42.25, 60, "F1")]:
        parts += [rect(x(start), 450, x(end) - x(start), 41, "#282440", "vision", 5),
                  text(x((start + end) / 2), 476, f"视觉依据 {source}", 15, "vision", "middle", 650)]
    for frame in MODEL["vision"]:
        parts += [line(x(frame["end"]), 493, x(frame["end"]), 502, "vision", 2),
                  text(x(frame["end"]) + 6, 515, f"{frame['source']} 发布 {frame['end']} ms（推演）", 11, "vision")]

    parts += lane_header("最近完成PIT", "代码10 ms / 100 Hz（配置，未测）", "CPU", 535, "pit")
    for start, end, source in [(0.20, 30.20, "F-1"), (30.20, 50.20, "F0"), (50.20, 60, "F1")]:
        parts += [rect(x(start), 551, x(end) - x(start), 37, "#3a2b25", "pit", 5),
                  text(x((start + end) / 2), 575, f"所用视觉 {source}", 15, "pit", "middle", 650)]
    for tick in MODEL["ticks"]:
        parts += [line(x(tick["due"]), 546, x(tick["due"]), 595, "pit", 1.5),
                  text(x(tick["due"]), 612, f"P{tick['due']}", 11, "pit", "middle")]

    parts += lane_header("舵机控制脉冲", "代码50 Hz → 20 ms（换算，未测）", "硬件", 636, "servo")
    parts += [rect(PLOT_LEFT, 648, PLOT_RIGHT - PLOT_LEFT, 55, "panel", radius=4),
              line(PLOT_LEFT, 690, PLOT_RIGHT, 690, "servo", 1.5)]
    for pulse in MODEL["pulses"]:
        left, right = x(pulse["start"]), x(pulse["end"])
        parts += [f'<path d="M {left:.2f} 690 V 656 H {right:.2f} V 690" '
                  'fill="none" stroke="#8ae0b2" stroke-width="2"/>',
                  rect(left, 656, right - left, 34, "#1c4d3a", radius=0),
                  text(left + 4, 721, f"{pulse['start']} ms相位（假设）· 来源{pulse['source']}", 12, "servo")]

    parts += lane_header("电机PWM", "代码17 kHz → 58.8 μs（换算，未测）", "硬件", 741, "motor")
    parts += [rect(PLOT_LEFT, 751, PLOT_RIGHT - PLOT_LEFT, 33, "url(#overview-motor-pattern)", "motor", 4),
              rect(PLOT_LEFT + 226, 753, 665, 29, "#241f32", radius=4),
              text((PLOT_LEFT + PLOT_RIGHT) / 2, 773, "17 kHz → 每10 ms约170周期（换算，未实测）；密纹示意连续硬件PWM", 13, "motor", "middle")]
    parts += cursor_svg(x(26), 243, 795)
    parts += [rect(28, 816, 1424, 82, "panel", "line"),
              text(44, 843, "26 ms（按假设推演）：采集/视觉已到F0；最近完成PIT20仍用F-1；25 ms舵机脉冲也采用F-1。", 15, weight=650,
                   extra='id="cursor-caption"'),
              text(44, 868, "PIT帧号只标降速/转向视觉依据；速度PI误差来自目标与本次编码器。它不是发布新视觉时即更新的ControlSnapshot.frame_id。", 13, "muted"),
              text(44, 888, "相机灰条是帧周期/采集传输，不表示曝光必须20 ms；舵机来源追踪命令依据，不表示实测转角或真实脉宽。", 12, "muted"),
              '</g></svg>']
    return "\n".join(parts)


def zoom_svg():
    """放大一个有新帧的10ms窗，硬件时间单列而不分摊CPU预算。"""
    def x(value):
        return PLOT_LEFT + (value - 20) / 10 * (PLOT_RIGHT - PLOT_LEFT)

    parts = begin_svg(784, "20至30毫秒：CPU执行时间与并行硬件",
                      "CSI0.05加PIT0.20加视觉2.00加收发1.00等于3.25毫秒；其余6.75毫秒为未展开的主循环工作与预算，非实测空闲。", "zoom")
    parts += [text(28, 38, "放大20—30 ms：这一有新帧的CPU窗口", 25, weight=700),
              rect(1230, 16, 222, 29, "panel", "line", 6),
              text(1341, 36, "离线示意 · 未实测", 14, "pit", "middle", 650),
              text(28, 67, "独立教学经过时间；所有执行耗时均是假设。所列任务3.25 ms，其余6.75 ms不代表实测空闲。", 14, "muted"),
              rect(28, 86, 1424, 68, "panel", "line"),
              text(44, 106, "代码已配置/换算（未实测）：PIT 10 ms；相机50 FPS / 20 ms；舵机50 Hz / 20 ms；电机17 kHz / 58.8 μs（每10 ms约170周期）。", 12, "camera", weight=600),
              text(44, 127, "教学CPU耗时：CSI 0.05 + PIT 0.20 + 视觉 2.00 + 收发 1.00 = 3.25 ms；按假设推演：其他主循环 / 余量6.75 ms。", 12, "pit"),
              text(44, 147, "初始F-1、舵机相位25 ms/中位脉宽1.5 ms也是教学假设；代码预算：单轮运行收发截止2 ms，实际耗时待实测。", 12, "vision")]
    for value in range(20, 31):
        parts += [line(x(value), 203, x(value), 654, "line", 1,
                       'stroke-dasharray="3 5"'),
                  text(x(value), 195, value, 14, "muted", "middle")]
    parts += lane_header("CPU这一窗", "代码周期10 ms（未实测）", "CPU", 259, "pit")
    blocks = [(20, 20.05, "csi", None), (20.05, 20.25, "pit", None),
              (20.25, 22.25, "vision", ("视觉 F0（假设）", "2.00 ms")),
              (22.25, 23.25, "io", ("一次收发", "1 ms·假设")),
              (23.25, 30, "url(#zoom-other-hatch)", ("其他主循环 / 示例余量（推演）", "6.75 ms"))]
    for start, end, color, label in blocks:
        parts.append(rect(x(start), 257, x(end) - x(start), 75, color, radius=0))
        if label:
            middle = x((start + end) / 2)
            parts += [text(middle, 288, label[0], 15, "ink" if start >= 23.25 else "bg", "middle", 650),
                      text(middle, 313, label[1], 14, "ink" if start >= 23.25 else "bg", "middle")]
    parts += [line(x(20.025), 253, x(20.35), 223, "csi", 1.5),
              text(x(20.35) + 7, 222, "CSI 0.05 ms（假设）", 13, "csi"),
              line(x(20.15), 335, x(20.48), 359, "pit", 1.5),
              text(x(20.48) + 7, 366, "PIT CPU 0.20 ms（假设）", 13, "pit"),
              text(28, 401, "PIT内部：本次编码器 → 保护 → 速度斜坡 → 左右PI → 舵机命令限速 → 写寄存器。整块0.20 ms是假设。", 14, "muted")]
    parts += lane_header("相机采集/传输", "代码50 FPS → 20 ms（换算，未测）", "硬件", 436, "camera")
    parts += [rect(PLOT_LEFT, 438, PLOT_RIGHT - PLOT_LEFT, 48, "#243449", "#4d7087", 5),
              text((PLOT_LEFT + PLOT_RIGHT) / 2, 468, "F1 仍在采集 / 传输：20—40 ms（跨过当前10 ms窗）", 15, "camera", "middle", 600)]
    parts += lane_header("舵机脉冲", "代码50 Hz → 20 ms（换算，未测）", "硬件", 532, "servo")
    parts += [rect(PLOT_LEFT, 533, PLOT_RIGHT - PLOT_LEFT, 65, "panel", radius=4),
              line(PLOT_LEFT, 584, PLOT_RIGHT, 584, "servo", 1.5),
              rect(x(25), 543, x(26.5) - x(25), 41, "#1c4d3a", radius=0),
              f'<path d="M {x(25):.2f} 584 V 543 H {x(26.5):.2f} V 584" fill="none" stroke="#8ae0b2" stroke-width="2"/>',
              text(x(25) + 7, 570, "F-1 · 1.5 ms假设", 13, "servo", weight=600)]
    parts += lane_header("电机PWM", "代码17 kHz（未测）· 硬件连续", "硬件", 627, "motor")
    parts += [rect(PLOT_LEFT, 633, PLOT_RIGHT - PLOT_LEFT, 31, "url(#zoom-motor-pattern)", "motor", 4)]
    parts += cursor_svg(x(26), 211, 674, prefix="zoom")
    parts += [rect(28, 698, 1424, 62, "panel", "line"),
              text(44, 723, "相机20 ms与舵机20 ms是硬件时间，不从CPU10 ms预算里分摊；电机PWM持续自主输出。", 15, weight=600),
              text(44, 746, "任务不是固定槽位，余量不是睡眠；其他10 ms窗可没有新帧。视觉可跨周期被PIT打断，本例假设2 ms未跨。", 13, "muted"),
              '</g></svg>']
    return "\n".join(parts)


HTML_TEMPLATE = r'''<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>控制与硬件的公共时间轴 · 离线教学示例</title>
<style>
:root{color-scheme:dark;--bg:#080f1d;--panel:#131f32;--line:#2c3c54;--ink:#ecf4ff;--muted:#99adc7}
*{box-sizing:border-box}body{margin:0;background:radial-gradient(ellipse at 20% 0,#18253b 0,transparent 46%),var(--bg);color:var(--ink);font-family:"Segoe UI","Microsoft YaHei",sans-serif;line-height:1.55}
main{max-width:1560px;margin:auto;padding:36px 30px 44px}.eyebrow{font-size:12px;letter-spacing:2px;color:#8fa9ce;font-weight:700;margin:0 0 12px}h1{font-size:clamp(25px,3vw,36px);line-height:1.3;margin:0 0 14px}h2{font-size:21px;margin:0 0 14px}.intro{color:var(--muted);max-width:1120px;margin:0 0 20px;font-size:15px}.notice{padding:14px 18px;background:#25251f;border:1px solid #5a4933;border-radius:12px;color:#ffd3a4;font-size:14px;margin-bottom:22px}.notice small{display:block;color:#c6b299;margin-top:5px}.controls{background:var(--panel);border:1px solid var(--line);border-radius:14px;padding:17px 20px;display:flex;gap:18px;align-items:center;flex-wrap:wrap}.time{min-width:153px}.time label{display:block;color:var(--muted);font-size:12px}.time output{font-size:26px;font-variant-numeric:tabular-nums;font-weight:700}.slider{flex:1;min-width:250px}.slider input{width:100%;accent-color:#92b8ff;cursor:pointer}.range-labels{display:flex;justify-content:space-between;font-size:12px;color:var(--muted)}button{background:#21324e;border:1px solid #486488;border-radius:8px;color:var(--ink);padding:10px 15px;font:inherit;cursor:pointer}button:hover{background:#2d466b}button:focus-visible,input:focus-visible{outline:2px solid #aaccff;outline-offset:4px}.buttons{display:flex;gap:9px}.cards{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:14px;margin:17px 0}.card{background:var(--panel);border:1px solid var(--line);border-top:3px solid var(--color);border-radius:12px;padding:14px 17px;min-height:150px}.card h3{font-size:13px;color:var(--muted);font-weight:500;margin:0 0 5px}.frame{font-size:30px;color:var(--color);font-weight:700;line-height:1.35}.card p{font-size:12px;color:var(--muted);margin:7px 0 0;min-height:36px}.activity{background:#101c2e;border:1px solid var(--line);border-radius:10px;margin:0 0 22px;padding:11px 16px;font-size:14px;color:#b9cbe5}.activity strong{color:var(--ink)}.figure{border:1px solid var(--line);border-radius:15px;overflow:auto;background:#0b1220;margin-bottom:22px}.figure svg{display:block;width:100%;height:auto;min-width:1060px}.zoom svg{min-width:1060px}.section-note{font-size:14px;color:var(--muted);margin:-2px 0 16px}.facts{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:14px}.fact{background:var(--panel);border:1px solid var(--line);padding:17px 18px;border-radius:12px;font-size:13px;color:var(--muted)}.fact strong{display:block;color:var(--ink);font-size:15px;margin-bottom:7px}footer{margin-top:23px;color:#8399b7;font-size:12px}noscript{display:block;padding:12px;color:#ffd3a4}@media(max-width:950px){main{padding:24px 16px}.cards{grid-template-columns:repeat(2,minmax(0,1fr))}.facts{grid-template-columns:1fr}.figure svg{min-width:1080px}}@media(max-width:560px){.cards{grid-template-columns:1fr 1fr;gap:8px}.card{padding:12px 10px}.frame{font-size:26px}.controls{gap:10px;padding:14px}.time{min-width:120px}.slider{min-width:180px}.buttons{width:100%}}
</style>
</head>
<body>
<main>
<p class="eyebrow">离线示意 / 未实测 / RUNNING · TRACK</p>
<h1>把采集、计算和脉冲放在同一条时间轴</h1>
<p class="intro">拖动时间，观察最新图像何时完成采集、何时发布视觉结果，以及后续PIT和舵机脉冲何时采用它。窗口前F-1已有效；本页不连接车辆。</p>
<div class="notice"><strong>代码已配置/据其换算（未实测）</strong>　PIT 10 ms / 100 Hz；相机50 FPS / 20 ms；舵机50 Hz / 20 ms；电机17 kHz / 约58.8 μs（每10 ms约170周期）。
<small><strong>教学假设</strong>　初始F-1缓存；CSI 0.05 ms；整轮PIT CPU 0.20 ms；视觉CPU 2 ms；一次运行收发1 ms；舵机相位5 / 25 / 45 ms、1.5 ms中位脉宽。</small>
<small><strong>按假设推演</strong>　视觉22.25 / 42.25 ms发布；余量6.75 ms；26 / 46 ms帧来源状态。　<strong>代码预算</strong>　单轮运行收发截止2 ms，物理最坏耗时待测。</small>
<small>横轴是独立教学经过时间，不是当前10 ms量化的control_now_ms。相机灰条表示20 ms帧周期/采集传输，曝光时长未展开。</small></div>
<section class="controls" aria-label="查看时间">
<div class="time"><label for="time-slider">当前查看时间（教学）</label><output id="time-value" for="time-slider">26.00 ms</output></div>
<div class="slider"><input id="time-slider" type="range" min="0" max="60" step="0.05" value="26" aria-label="查看时间，单位毫秒"><div class="range-labels"><span>0 ms</span><span>20 ms</span><span>40 ms</span><span>60 ms</span></div></div>
<div class="buttons"><button type="button" id="jump-26">看26 ms</button><button type="button" id="jump-46">看46 ms</button></div>
</section>
<section class="cards" aria-live="polite" aria-label="该时刻的帧来源">
<article class="card" style="--color:#68d7e5"><h3>已完成采集 · 按假设推演</h3><div class="frame" id="camera-frame">F0</div><p id="camera-detail"></p></article>
<article class="card" style="--color:#b6a1ff"><h3>已发布视觉 · 按假设推演</h3><div class="frame" id="vision-frame">F0</div><p id="vision-detail"></p></article>
<article class="card" style="--color:#ffb277"><h3>最近完成PIT · 按假设推演</h3><div class="frame" id="pit-frame">F-1</div><p id="pit-detail"></p></article>
<article class="card" style="--color:#8ae0b2"><h3>最近舵机命令来源 · 按假设推演</h3><div class="frame" id="servo-frame">F-1</div><p id="servo-detail"></p></article>
</section>
<div class="activity"><strong>这一刻CPU：</strong><span id="cpu-activity"></span><span id="pending-pit"></span></div>
<section class="figure" id="overview-figure" aria-label="0至60毫秒公共时间轴">__OVERVIEW__</section>
<h2>再看这一有新帧的10 ms窗口</h2>
<p class="section-note">20—30 ms里，CPU列出的工作共3.25 ms。相机和舵机在硬件上并行，不能拿它们的20 ms周期分摊CPU时间。</p>
<section class="figure zoom" aria-label="20至30毫秒CPU执行时间放大">__ZOOM__</section>
<section class="facts" aria-label="读图要点">
<div class="fact"><strong>发布新图，会先改变转向目标</strong>22.25 ms发布F0后，TRACK的P/PD目标已更新；舵机命令限速及寄存器写入要到PIT30。图中的PIT来源是教学追踪标签，不是ControlSnapshot.frame_id。</div>
<div class="fact"><strong>CPU条块来自事件，不是固定槽位</strong>6.75 ms是本窗未展开的主循环工作/预算，不能当实测空闲或睡眠。其他无新帧的10 ms窗不必运行视觉；视觉若跨周期，可以被可服务的PIT中断打断。</div>
<div class="fact"><strong>通信和硬件各自有自己的节奏</strong>1 ms是一次收发服务的教学假设；代码设每轮运行服务截止预算2 ms，物理最坏耗时待实测，并非每10 ms总预算。运行波形最高10 Hz；电机代码配置17 kHz自主输出。舵机F号追踪命令依据，1.5 ms只是中位脉宽假设。</div>
</section>
<footer>完成事件在执行结束时成立；执行/高脉冲区间采用[开始,结束)。60 ms处CSI刚开始，PIT60仍等待，60.25 ms完成在图窗之外。静态两图默认观察26 ms。</footer>
<noscript>浏览器已停用脚本；当前展示26 ms的静态示例。启用本地脚本后可拖动时间。</noscript>
</main>
<script>
const model=__MODEL__;
const slider=document.getElementById('time-slider');
const last=(events,predicate)=>events.filter(predicate).at(-1)||null;
const fmt=value=>Number(value).toFixed(2);
// 完成事件不能提前到到期时刻；60ms边界的PIT仍等待CSI服务。
function getTimelineState(time){
  const t=Math.max(0,Math.min(60,Number(time)));
  return {time:t,camera:last(model.camera,e=>e.end<=t),vision:last(model.vision,e=>e.end<=t),
    pit:last(model.ticks,e=>e.end<=t),servo:last(model.pulses,e=>e.start<=t),
    activity:model.services.find(e=>e.start<=t&&t<e.end)||null,
    pending:model.ticks.find(e=>e.due<=t&&t<e.start)||null};
}
function setText(id,value){document.getElementById(id).textContent=value;}
function update(time){
  const state=getTimelineState(time),t=state.time;
  slider.value=t;setText('time-value',fmt(t)+' ms');
  setText('camera-frame',state.camera?state.camera.source:'F-1');
  const capturing=model.camera.find(e=>e.start<=t&&t<e.end);
  setText('camera-detail',(state.camera?'硬件完成于'+fmt(state.camera.end)+' ms。':'F-1在窗口前完成。')+(capturing?'正在采集'+capturing.source+'。':'后续采集未展开。'));
  setText('vision-frame',state.vision?state.vision.source:'F-1');
  setText('vision-detail',state.vision?'发布于'+fmt(state.vision.end)+' ms；转向目标已更新。':'窗口前已有有效结果。');
  setText('pit-frame',state.pit?state.pit.source:'—');
  setText('pit-detail',state.pit?'到期'+state.pit.due+' ms，执行'+fmt(state.pit.start)+'—'+fmt(state.pit.end)+' ms。速度PI用本次编码器。':'PIT0正在执行；窗口前PIT未展开。');
  setText('servo-frame',state.servo?state.servo.source:'—');
  setText('servo-detail',state.servo?'脉冲'+fmt(state.servo.start)+'—'+fmt(state.servo.end)+' ms；当前'+(t<state.servo.end?'处于高电平。':'为低电平，保留最近脉冲来源。'):'本窗口首个脉冲将在5 ms开始。');
  setText('cpu-activity',state.activity?state.activity.label+'（'+fmt(state.activity.start)+'—'+fmt(state.activity.end)+' ms）':'其他主循环 / 未展开工作与预算；不表示CPU睡眠。');
  setText('pending-pit',state.pending?'；PIT'+state.pending.due+'已到期，等待CSI结束。':'');
  const cursor=document.getElementById('overview-cursor');
  cursor.setAttribute('transform','translate('+(model.plotLeft+t/60*(model.plotRight-model.plotLeft))+' 0)');
  setText('overview-cursor-label',fmt(t)+' ms');
  // 放大图只涵盖20至30ms，其余时刻隐藏放大图游标而不伪造窗口。
  const zoom=document.getElementById('zoom-cursor');
  zoom.style.display=t>=20&&t<=30?'':'none';
  zoom.setAttribute('transform','translate('+(model.plotLeft+(t-20)/10*(model.plotRight-model.plotLeft))+' 0)');
  setText('zoom-cursor-label',fmt(t)+' ms');
  setText('cursor-caption',fmt(t)+' ms（按假设推演）：采集 '+(state.camera?state.camera.source:'F-1')+' / 视觉 '+(state.vision?state.vision.source:'F-1')+'；最近完成PIT '+(state.pit?state.pit.due+' ms · '+state.pit.source:'未展开')+'；最近舵机脉冲 '+(state.servo?state.servo.source:'未到5 ms')+'。');
  window.currentTimelineState=state;
}
window.getTimelineState=getTimelineState;
slider.addEventListener('input',event=>update(event.target.value));
document.getElementById('jump-26').addEventListener('click',()=>update(26));
document.getElementById('jump-46').addEventListener('click',()=>update(46));
update(26);
</script>
</body>
</html>
'''


def render_html(overview, zoom):
    """嵌入图形及事件模型，HTML离线打开即可操作滑块。"""
    return (HTML_TEMPLATE.replace("__OVERVIEW__", overview)
            .replace("__ZOOM__", zoom)
            .replace("__MODEL__", json.dumps(MODEL, ensure_ascii=False)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "docs/visualizations")
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    overview, zoom = overview_svg(), zoom_svg()
    outputs = {"control_timeline_overview.svg": overview,
               "control_timeline_10ms.svg": zoom,
               "control_timeline.html": render_html(overview, zoom)}
    for name, content in outputs.items():
        path = args.output_dir / name
        path.write_text(content + "\n", encoding="utf-8")
        print(path)


if __name__ == "__main__":
    main()
