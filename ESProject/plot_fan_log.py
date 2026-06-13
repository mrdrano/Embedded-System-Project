#!/usr/bin/env python3
"""
plot_fan_log.py

Parse fan vibration RT monitor logs and generate charts after baseline ends,
from the first frame after baseline collection until CONFIG_FRAME.

Usage:
    python3 plot_fan_log.py fan_log_mudium_normal.txt
    python3 plot_fan_log.py fan_log_mudium_normal.txt --config-frame 512 --out-dir charts

Generated charts:
    1. score_line.png
    2. frequency_line.png
    3. rms_vibration_line.png
    4. harmonic_ratio_line.png
    5. harmonic_amplitude_line.png
    6. rt_latency_line.png
    7. rt_event_counts.png
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import matplotlib.pyplot as plt


# If CONFIG_FRAME is None, the program automatically uses the largest parsed frame.
CONFIG_FRAME: Optional[int] = None
FLOAT_RE = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)"

NORMAL_PATTERN = re.compile(
    r"^\[(?P<state>[A-Z]+)\]\s+"
    r"frame=(?P<frame>\d+)\s+"
    rf"score=(?P<score>{FLOAT_RE})\s+"
    r"baseline=(?P<baseline>yes|no)\s+"
    r"(?:scoreAxis=(?P<score_axis>[A-Za-z]+)\s+)?"
    rf"domHz\(x/y/z\)=(?P<dom_x>{FLOAT_RE})/(?P<dom_y>{FLOAT_RE})/(?P<dom_z>{FLOAT_RE})\s+"
    rf"rpm1x=(?P<rpm1x>{FLOAT_RE})\s+"
    rf"rms\(x/y/z/t\)=(?P<rms_x>{FLOAT_RE})/(?P<rms_y>{FLOAT_RE})/(?P<rms_z>{FLOAT_RE})/(?P<rms_t>{FLOAT_RE})"
    rf"(?:\s+Yharm\(A1/A2/A3\)=(?P<y_amp_1x>{FLOAT_RE})/(?P<y_amp_2x>{FLOAT_RE})/(?P<y_amp_3x>{FLOAT_RE})"
    rf"\s+Yratio2=(?P<y_ratio2>{FLOAT_RE})\s+Yratio3=(?P<y_ratio3>{FLOAT_RE}))?"
    rf"(?:\s+Zharm\(A1/A2/A3\)=(?P<z_amp_1x>{FLOAT_RE})/(?P<z_amp_2x>{FLOAT_RE})/(?P<z_amp_3x>{FLOAT_RE})"
    rf"\s+(?:Z)?ratio2=(?P<z_ratio2>{FLOAT_RE})\s+(?:Z)?ratio3=(?P<z_ratio3>{FLOAT_RE}))?"
)

BASELINE_PATTERN = re.compile(
    r"^\[BASELINE\]\s+frame=(?P<frame>\d+)\s+progress=(?P<progress>\d+)/(?:\d+)"
)

RT_PATTERN = re.compile(
    r"^\[RT\]\s+"
    rf"wake_avg/max=(?P<wake_avg>{FLOAT_RE})/(?P<wake_max>{FLOAT_RE})us\s+"
    rf"i2c_avg/max=(?P<i2c_avg>{FLOAT_RE})/(?P<i2c_max>{FLOAT_RE})us\s+"
    rf"handoff_avg/max=(?P<handoff_avg>{FLOAT_RE})/(?P<handoff_max>{FLOAT_RE})us\s+"
    r"j>100us=(?P<j_gt100>\d+)\s+"
    r"i2c>500us=(?P<i2c_gt500>\d+)\s+"
    r"handoff>100us=(?P<handoff_gt100>\d+)\s+"
    r"overrun=(?P<overrun>\d+)\s+"
    r"missed=(?P<missed>\d+)"
)


def to_float_dict(match: re.Match, exclude: Tuple[str, ...] = ()) -> Dict[str, float]:
    out: Dict[str, float] = {}
    for key, value in match.groupdict().items():
        if key in exclude or value is None:
            continue
        out[key] = float(value)
    return out


def parse_log(log_path: Path) -> Tuple[List[Dict[str, float]], Dict[int, Dict[str, float]], int]:
    """Return normal data rows, RT rows indexed by frame, and baseline end frame."""
    rows: List[Dict[str, float]] = []
    rt_by_frame: Dict[int, Dict[str, float]] = {}
    baseline_end_frame = 0
    last_data_frame: Optional[int] = None

    with log_path.open("r", encoding="utf-8", errors="replace") as f:
        for raw_line in f:
            line = raw_line.strip()

            baseline_match = BASELINE_PATTERN.match(line)
            if baseline_match:
                baseline_end_frame = max(baseline_end_frame, int(baseline_match.group("frame")))
                continue

            normal_match = NORMAL_PATTERN.match(line)
            if normal_match:
                row = to_float_dict(normal_match, exclude=("state", "baseline", "score_axis"))
                row["frame"] = int(normal_match.group("frame"))
                row["score"] = float(normal_match.group("score"))
                row["baseline_is_valid"] = 1.0 if normal_match.group("baseline") == "yes" else 0.0
                row["score_axis_y"] = 1.0 if normal_match.group("score_axis") == "Y" else 0.0
                rows.append(row)
                last_data_frame = int(row["frame"])
                continue

            rt_match = RT_PATTERN.match(line)
            if rt_match and last_data_frame is not None:
                rt_row = to_float_dict(rt_match)
                rt_row["frame"] = last_data_frame
                rt_by_frame[last_data_frame] = rt_row

    if not rows:
        raise ValueError("No [NORMAL] data rows were parsed. Please check the log format.")

    if baseline_end_frame == 0:
        # Fallback: if no [BASELINE] line exists, start from the first baseline=yes row.
        baseline_yes_frames = [int(r["frame"]) for r in rows if r.get("baseline_is_valid", 0.0) > 0.0]
        baseline_end_frame = min(baseline_yes_frames) - 1 if baseline_yes_frames else 0

    return rows, rt_by_frame, baseline_end_frame


def filter_after_baseline(
    rows: List[Dict[str, float]],
    rt_by_frame: Dict[int, Dict[str, float]],
    baseline_end_frame: int,
    config_frame: Optional[int],
) -> Tuple[List[Dict[str, float]], List[Dict[str, float]], int]:
    max_frame = max(int(r["frame"]) for r in rows)
    end_frame = config_frame if config_frame is not None else max_frame

    filtered_rows = [
        r for r in rows
        if baseline_end_frame < int(r["frame"]) <= end_frame
    ]
    filtered_rt = [
        rt_by_frame[int(r["frame"])] for r in filtered_rows
        if int(r["frame"]) in rt_by_frame
    ]

    if not filtered_rows:
        raise ValueError(
            f"No data remains after filtering: baseline_end_frame={baseline_end_frame}, "
            f"CONFIG_FRAME={end_frame}."
        )

    return filtered_rows, filtered_rt, end_frame


def get_series(rows: List[Dict[str, float]], key: str) -> List[float]:
    return [float(r[key]) for r in rows]


def save_score_chart(rows: List[Dict[str, float]], out_dir: Path) -> None:
    frames = get_series(rows, "frame")
    plt.figure(figsize=(12, 5))
    plt.plot(frames, get_series(rows, "score"), color="tab:blue", label="score", linewidth=1.8)
    plt.axhline(3.0, color="tab:orange", linestyle="--", linewidth=1.2, label="warning threshold = 3.0")
    plt.axhline(6.0, color="tab:red", linestyle="--", linewidth=1.2, label="alert threshold = 6.0")
    plt.title("Score after baseline")
    plt.xlabel("Frame")
    plt.ylabel("Score")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(out_dir / "score_line.png", dpi=160)
    plt.close()


def save_frequency_chart(rows: List[Dict[str, float]], out_dir: Path) -> None:
    frames = get_series(rows, "frame")
    plt.figure(figsize=(12, 5))
    plt.plot(frames, get_series(rows, "dom_x"), color="tab:blue", label="domHz x", linewidth=1.5)
    plt.plot(frames, get_series(rows, "dom_y"), color="tab:orange", label="domHz y", linewidth=1.5)
    plt.plot(frames, get_series(rows, "dom_z"), color="tab:green", label="domHz z", linewidth=1.5)
    plt.title("Dominant frequency after baseline")
    plt.xlabel("Frame")
    plt.ylabel("Frequency (Hz)")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(out_dir / "frequency_line.png", dpi=160)
    plt.close()


def save_rms_chart(rows: List[Dict[str, float]], out_dir: Path) -> None:
    frames = get_series(rows, "frame")
    plt.figure(figsize=(12, 5))
    plt.plot(frames, get_series(rows, "rms_x"), color="tab:blue", label="rms x", linewidth=1.5)
    plt.plot(frames, get_series(rows, "rms_y"), color="tab:orange", label="rms y", linewidth=1.5)
    plt.plot(frames, get_series(rows, "rms_z"), color="tab:green", label="rms z", linewidth=1.5)
    plt.plot(frames, get_series(rows, "rms_t"), color="tab:red", label="rms total", linewidth=1.8)
    plt.title("RMS vibration intensity after baseline")
    plt.xlabel("Frame")
    plt.ylabel("RMS acceleration (g)")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(out_dir / "rms_vibration_line.png", dpi=160)
    plt.close()


def has_keys(rows: List[Dict[str, float]], keys: List[str]) -> bool:
    return all(all(key in row for key in keys) for row in rows)


def save_harmonic_ratio_chart(rows: List[Dict[str, float]], out_dir: Path) -> None:
    keys = ["y_ratio2", "y_ratio3", "z_ratio2", "z_ratio3"]
    if not has_keys(rows, keys):
        print("No complete Y/Z harmonic ratio fields were parsed; skip harmonic ratio chart.")
        return

    frames = get_series(rows, "frame")
    plt.figure(figsize=(12, 5))
    plt.plot(frames, get_series(rows, "y_ratio2"), color="tab:orange", label="Y ratio 2X/1X", linewidth=1.5)
    plt.plot(frames, get_series(rows, "y_ratio3"), color="tab:red", label="Y ratio 3X/1X", linewidth=1.5)
    plt.plot(frames, get_series(rows, "z_ratio2"), color="tab:green", linestyle="--", label="Z ratio 2X/1X", linewidth=1.3)
    plt.plot(frames, get_series(rows, "z_ratio3"), color="tab:purple", linestyle="--", label="Z ratio 3X/1X", linewidth=1.3)
    plt.title("Harmonic ratios after baseline")
    plt.xlabel("Frame")
    plt.ylabel("Amplitude ratio")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(out_dir / "harmonic_ratio_line.png", dpi=160)
    plt.close()


def save_harmonic_amplitude_chart(rows: List[Dict[str, float]], out_dir: Path) -> None:
    keys = ["y_amp_1x", "y_amp_2x", "y_amp_3x", "z_amp_1x", "z_amp_2x", "z_amp_3x"]
    if not has_keys(rows, keys):
        print("No complete Y/Z harmonic amplitude fields were parsed; skip harmonic amplitude chart.")
        return

    frames = get_series(rows, "frame")
    plt.figure(figsize=(12, 5))
    plt.plot(frames, get_series(rows, "y_amp_1x"), color="tab:orange", label="Y A1", linewidth=1.5)
    plt.plot(frames, get_series(rows, "y_amp_2x"), color="tab:red", label="Y A2", linewidth=1.3)
    plt.plot(frames, get_series(rows, "y_amp_3x"), color="tab:pink", label="Y A3", linewidth=1.3)
    plt.plot(frames, get_series(rows, "z_amp_1x"), color="tab:green", linestyle="--", label="Z A1", linewidth=1.3)
    plt.plot(frames, get_series(rows, "z_amp_2x"), color="tab:purple", linestyle="--", label="Z A2", linewidth=1.2)
    plt.plot(frames, get_series(rows, "z_amp_3x"), color="tab:brown", linestyle="--", label="Z A3", linewidth=1.2)
    plt.title("Harmonic amplitudes after baseline")
    plt.xlabel("Frame")
    plt.ylabel("FFT amplitude")
    plt.grid(True, alpha=0.3)
    plt.legend(ncol=3)
    plt.tight_layout()
    plt.savefig(out_dir / "harmonic_amplitude_line.png", dpi=160)
    plt.close()


def save_rt_latency_chart(rt_rows: List[Dict[str, float]], out_dir: Path) -> None:
    if not rt_rows:
        print("No RT rows were parsed; skip RT latency chart.")
        return

    frames = get_series(rt_rows, "frame")
    plt.figure(figsize=(12, 5))

    # Average latency: solid lines. Maximum latency: dashed lines.
    plt.plot(frames, get_series(rt_rows, "wake_avg"), color="tab:blue", label="wake avg", linewidth=1.4)
    plt.plot(frames, get_series(rt_rows, "wake_max"), color="tab:blue", linestyle="--", label="wake max", linewidth=1.2)
    plt.plot(frames, get_series(rt_rows, "i2c_avg"), color="tab:orange", label="i2c avg", linewidth=1.4)
    plt.plot(frames, get_series(rt_rows, "i2c_max"), color="tab:orange", linestyle="--", label="i2c max", linewidth=1.2)
    plt.plot(frames, get_series(rt_rows, "handoff_avg"), color="tab:green", label="handoff avg", linewidth=1.4)
    plt.plot(frames, get_series(rt_rows, "handoff_max"), color="tab:green", linestyle="--", label="handoff max", linewidth=1.2)

    plt.title("RT latency after baseline")
    plt.xlabel("Frame")
    plt.ylabel("Latency (us)")
    plt.grid(True, alpha=0.3)
    plt.legend(ncol=3)
    plt.tight_layout()
    plt.savefig(out_dir / "rt_latency_line.png", dpi=160)
    plt.close()


def save_rt_event_count_chart(rt_rows: List[Dict[str, float]], out_dir: Path) -> None:
    if not rt_rows:
        print("No RT rows were parsed; skip RT event count chart.")
        return

    labels = ["j>100us", "i2c>500us", "handoff>100us", "overrun", "missed"]
    keys = ["j_gt100", "i2c_gt500", "handoff_gt100", "overrun", "missed"]

    # These counters are cumulative in the log. The last value represents total events in the selected range.
    totals = [int(rt_rows[-1][key]) for key in keys]

    plt.figure(figsize=(10, 5))
    plt.bar(labels, totals, color=["tab:blue", "tab:orange", "tab:green", "tab:red", "tab:purple"])
    plt.title("RT threshold / deadline event counts after baseline")
    plt.xlabel("RT event type")
    plt.ylabel("Cumulative count")
    plt.grid(True, axis="y", alpha=0.3)
    plt.tight_layout()
    plt.savefig(out_dir / "rt_event_counts.png", dpi=160)
    plt.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate charts from fan vibration RT monitor log.")
    parser.add_argument("log_file", type=Path, help="Path to fan log txt file")
    parser.add_argument("--config-frame", type=int, default=CONFIG_FRAME, help="Use data until this frame. Default: max frame in log")
    parser.add_argument("--out-dir", type=Path, default=Path("fan_log_charts"), help="Output directory for PNG charts")
    args = parser.parse_args()

    rows, rt_by_frame, baseline_end_frame = parse_log(args.log_file)
    rows, rt_rows, end_frame = filter_after_baseline(rows, rt_by_frame, baseline_end_frame, args.config_frame)

    args.out_dir.mkdir(parents=True, exist_ok=True)

    save_score_chart(rows, args.out_dir)
    save_frequency_chart(rows, args.out_dir)
    save_rms_chart(rows, args.out_dir)
    save_harmonic_ratio_chart(rows, args.out_dir)
    save_harmonic_amplitude_chart(rows, args.out_dir)
    save_rt_latency_chart(rt_rows, args.out_dir)
    save_rt_event_count_chart(rt_rows, args.out_dir)

    print(f"baseline_end_frame = {baseline_end_frame}")
    print(f"CONFIG_FRAME/end_frame = {end_frame}")
    print(f"selected frame range = {int(rows[0]['frame'])}..{int(rows[-1]['frame'])}")
    print(f"output directory = {args.out_dir.resolve()}")
    print("generated files:")
    for png in sorted(args.out_dir.glob("*.png")):
        print(f"  - {png.name}")


if __name__ == "__main__":
    main()
