#!/usr/bin/env python3
import csv
import math
import sys
from typing import Tuple

def read_first_last_xy(csv_path: str) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    first_xy = None
    last_xy = None

    with open(csv_path, "r", newline="") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or len(row) < 5:
                continue
            # Expected format:
            # [timestamp, seq, "map", x, y, ...]
            try:
                frame = row[2]
                if frame != "map":
                    continue
                x = float(row[3])
                y = float(row[4])
            except (ValueError, IndexError):
                continue

            if first_xy is None:
                first_xy = (x, y)
            last_xy = (x, y)

    if first_xy is None or last_xy is None:
        raise ValueError("No valid map,x,y rows found in the CSV.")

    return first_xy, last_xy

def euclidean_distance(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    return math.hypot(dx, dy)

def main():
    csv_path = sys.argv[1] if len(sys.argv) > 1 else "/home/nvidia/kiro_limit/go2_ros2_sdk/path_tracker/path/path_4_1_5.csv"
    first_xy, last_xy = read_first_last_xy(csv_path)
    dist = euclidean_distance(first_xy, last_xy)

    print(f"Start (x, y): {first_xy[0]:.12f}, {first_xy[1]:.12f}")
    print(f"End   (x, y): {last_xy[0]:.12f}, {last_xy[1]:.12f}")
    print(f"Euclidean distance (start ↔ end): {dist:.6f}")

if __name__ == "__main__":
    main()