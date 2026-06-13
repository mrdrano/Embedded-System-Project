import subprocess
import re
import queue
import threading
from collections import deque
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


# ====== 設定區 ======
SCRIPT_DIR = Path(__file__).resolve().parent
SOURCE_PATH = SCRIPT_DIR / "main.c"
EXECUTABLE_PATH = SCRIPT_DIR / "fan_vibration_rt"
CMD = ["stdbuf", "-oL", str(EXECUTABLE_PATH)]

MAX_POINTS = 300          # 圖上最多保留最近 300 筆 frame
WARNING_THRESHOLD = 3.0
ALERT_THRESHOLD = 6.0

# 解析這種格式：
# [NORMAL] frame=876 score=0.00 baseline=no ...
LOG_PATTERN = re.compile(
    r"\[(?P<state>[A-Z]+)\]\s+"
    r"frame=(?P<frame>\d+)\s+"
    r"score=(?P<score>[-+]?\d+(?:\.\d+)?)\s+"
    r"baseline=(?P<baseline>yes|no)"
)


frames = deque(maxlen=MAX_POINTS)
scores = deque(maxlen=MAX_POINTS)
states = deque(maxlen=MAX_POINTS)
baselines = deque(maxlen=MAX_POINTS)
line_queue = queue.Queue()


def build_if_needed():
    """
    main.c 更新後，舊的 fan_vibration_rt 不會自動變成新程式。
    這裡在啟動前檢查時間戳，必要時重新編譯。
    """
    if not SOURCE_PATH.exists():
        raise FileNotFoundError(f"找不到 {SOURCE_PATH}")

    needs_build = (
        not EXECUTABLE_PATH.exists()
        or SOURCE_PATH.stat().st_mtime > EXECUTABLE_PATH.stat().st_mtime
    )
    if not needs_build:
        return

    compile_cmd = [
        "gcc",
        "-O2",
        "-Wall",
        "-Wextra",
        "-pthread",
        "main.c",
        "-lm",
        "-o",
        EXECUTABLE_PATH.name,
    ]
    print("main.c 比 fan_vibration_rt 新，重新編譯中...")
    completed = subprocess.run(
        compile_cmd,
        cwd=SCRIPT_DIR,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    if completed.stdout:
        print(completed.stdout, end="")
    if completed.returncode != 0:
        raise RuntimeError("編譯 main.c 失敗，請先修正上方 gcc 訊息")


def start_process():
    """
    啟動 C 程式，並即時讀取 stdout。
    stdbuf -oL 的目的是讓 C 程式 stdout 以 line-buffered 輸出，
    避免 Python 端等很久才收到資料。
    """
    return subprocess.Popen(
        CMD,
        cwd=SCRIPT_DIR,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1
    )


build_if_needed()
process = start_process()


def collect_process_output():
    for line in process.stdout:
        line_queue.put(line)
    line_queue.put(f"[PLOT] fan_vibration_rt exited with code {process.poll()}\n")


reader_thread = threading.Thread(target=collect_process_output, daemon=True)
reader_thread.start()


def read_available_lines():
    """
    每次動畫更新時，讀取目前 queue 裡已經收到的 log。
    真正會 blocking 的 stdout 讀取放在背景 thread，避免圖形介面卡住。
    """
    for _ in range(100):
        try:
            line = line_queue.get_nowait()
        except queue.Empty:
            break

        print(line, end="")  # 同時保留終端機輸出，方便 debug

        match = LOG_PATTERN.search(line)
        if not match:
            continue

        frame = int(match.group("frame"))
        score = float(match.group("score"))
        state = match.group("state")
        baseline = match.group("baseline")

        frames.append(frame)
        scores.append(score)
        states.append(state)
        baselines.append(baseline)


# ====== Matplotlib 初始化 ======
fig, ax = plt.subplots(figsize=(10, 5))

line_score, = ax.plot([], [], linewidth=2, label="Anomaly Score")
line_warning = ax.axhline(
    WARNING_THRESHOLD,
    linestyle="--",
    linewidth=1,
    label="Warning Threshold"
)
line_alert = ax.axhline(
    ALERT_THRESHOLD,
    linestyle="--",
    linewidth=1,
    label="Alert Threshold"
)

status_text = ax.text(
    0.02,
    0.95,
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
    read_available_lines()

    if not frames:
        return line_score, status_text

    x = list(frames)
    y = list(scores)

    line_score.set_data(x, y)

    ax.set_xlim(min(x), max(x) + 1)

    y_max = max(max(y) + 1.0, ALERT_THRESHOLD + 1.0)
    ax.set_ylim(0, y_max)

    latest_state = states[-1]
    latest_baseline = baselines[-1]
    latest_score = scores[-1]
    latest_frame = frames[-1]

    status_text.set_text(
        f"Frame: {latest_frame}\n"
        f"Score: {latest_score:.2f}\n"
        f"State: {latest_state}\n"
        f"Baseline: {latest_baseline}"
    )

    return line_score, status_text


try:
    ani = FuncAnimation(
        fig,
        update_plot,
        interval=200,
        blit=False
    )

    plt.tight_layout()
    plt.show()

except KeyboardInterrupt:
    pass

finally:
    if process.poll() is None:
        process.terminate()
