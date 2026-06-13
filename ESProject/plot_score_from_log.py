# Run: 
# python3 plot_score_from_log.py

import re
from collections import deque
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


SCRIPT_DIR = Path(__file__).resolve().parent
LOG_FILE = SCRIPT_DIR / "fan_log.txt"

MAX_POINTS = 300
WARNING_THRESHOLD = 3.0
ALERT_THRESHOLD = 6.0
FLOAT_RE = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)"

NORMAL_PATTERN = re.compile(
    r"^\[(?P<state>NORMAL|WARNING|ALERT)\]\s+"
    r"frame=(?P<frame>\d+)\s+"
    rf"score=(?P<score>{FLOAT_RE})\s+"
    r"baseline=(?P<baseline>yes|no)"
    r"(?:\s+scoreAxis=(?P<score_axis>[A-Za-z]+))?"
)

BASELINE_PATTERN = re.compile(
    r"^\[BASELINE\]\s+"
    r"frame=(?P<frame>\d+)\s+"
    r"progress=(?P<progress>\d+)/(?P<total>\d+)"
)

frames = deque(maxlen=MAX_POINTS)
scores = deque(maxlen=MAX_POINTS)
states = deque(maxlen=MAX_POINTS)
baselines = deque(maxlen=MAX_POINTS)
score_axes = deque(maxlen=MAX_POINTS)

file_position = 0
last_file_size = 0
last_message = f"Waiting for {LOG_FILE}"
pending_line = ""


def read_new_lines():
    global file_position, last_file_size, last_message, pending_line

    try:
        current_size = LOG_FILE.stat().st_size
        if current_size < file_position:
            file_position = 0
            pending_line = ""
            frames.clear()
            scores.clear()
            states.clear()
            baselines.clear()
            score_axes.clear()
            last_message = "Log file reset, reloading"

        if current_size == file_position:
            last_file_size = current_size
            return

        with LOG_FILE.open("r", encoding="utf-8", errors="replace") as f:
            f.seek(file_position)
            new_text = f.read()
            file_position = f.tell()
            last_file_size = current_size
    except FileNotFoundError:
        last_message = f"Waiting for {LOG_FILE}"
        return

    if not new_text:
        return

    text = pending_line + new_text
    raw_lines = text.splitlines(keepends=True)

    if raw_lines and not raw_lines[-1].endswith(("\n", "\r")):
        pending_line = raw_lines.pop()
    else:
        pending_line = ""

    lines = [line.rstrip("\r\n") for line in raw_lines]

    for line in lines:
        baseline_match = BASELINE_PATTERN.search(line)
        if baseline_match:
            frame = int(baseline_match.group("frame"))
            progress = int(baseline_match.group("progress"))
            total = int(baseline_match.group("total"))
            last_message = f"Baseline frame {frame}: {progress}/{total}"
            continue

        match = NORMAL_PATTERN.search(line)
        if not match:
            continue

        frame = int(match.group("frame"))
        score = float(match.group("score"))
        state = match.group("state")
        baseline = match.group("baseline")
        score_axis = match.group("score_axis") or "unknown"

        frames.append(frame)
        scores.append(score)
        states.append(state)
        baselines.append(baseline)
        score_axes.append(score_axis)
        last_message = f"Loaded frame {frame}"


fig, ax = plt.subplots(figsize=(10, 5))

line_score, = ax.plot([], [], linewidth=2, label="Anomaly Score")

ax.axhline(WARNING_THRESHOLD, linestyle="--", linewidth=1, label="Warning Threshold")
ax.axhline(ALERT_THRESHOLD, linestyle="--", linewidth=1, label="Alert Threshold")

status_text = ax.text(
    0.02,
    0.95,
    "",
    transform=ax.transAxes,
    verticalalignment="top"
)

file_text = ax.text(
    0.02,
    0.80,
    "",
    transform=ax.transAxes,
    verticalalignment="top"
)

ax.set_title("Live Fan Vibration Anomaly Score")
ax.set_xlabel("Frame")
ax.set_ylabel("Score")
ax.grid(True)
ax.legend(loc="upper right")


def update_plot(_):
    read_new_lines()

    if not frames:
        file_text.set_text(last_message)
        return line_score, status_text, file_text

    x = list(frames)
    y = list(scores)

    line_score.set_data(x, y)

    ax.set_xlim(min(x), max(x) + 1)

    y_max = max(max(y) + 1.0, ALERT_THRESHOLD + 1.0)
    ax.set_ylim(0, y_max)

    latest_frame = frames[-1]
    latest_score = scores[-1]
    latest_state = states[-1]
    latest_baseline = baselines[-1]
    latest_score_axis = score_axes[-1]

    status_text.set_text(
        f"Frame: {latest_frame}\n"
        f"Score: {latest_score:.2f}\n"
        f"State: {latest_state}\n"
        f"Baseline: {latest_baseline}\n"
        f"Score Axis: {latest_score_axis}"
    )
    file_text.set_text(f"{LOG_FILE.name}: {last_file_size} bytes")

    return line_score, status_text, file_text


anim = FuncAnimation(
    fig,
    update_plot,
    interval=500,
    blit=False,
    cache_frame_data=False
)

plt.tight_layout()
plt.show(block=True)
