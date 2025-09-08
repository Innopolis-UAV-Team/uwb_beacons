#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import re
import sys
import matplotlib.pyplot as plt

# Регулярка для поиска x, y, z (z можно игнорировать)
POS_PATTERN = re.compile(
    r"Position:\s*x=(-?\d+(?:\.\d+)?)\s*y=(-?\d+(?:\.\d+)?)\s*z=(-?\d+(?:\.\d+)?)"
)

def parse_positions(file_path):
    xs, ys, zs = [], [], []
    try:
        with open(file_path, "r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                m = POS_PATTERN.search(line)
                if m:
                    xs.append(float(m.group(1)))
                    ys.append(float(m.group(2)))
                    zs.append(float(m.group(3)))
    except Exception as e:
        print(f"[ERROR] Не удалось прочитать '{file_path}': {e}", file=sys.stderr)
        sys.exit(1)

    if len(xs) == 0:
        print(f"[WARN] В файле '{file_path}' не найдено координат!", file=sys.stderr)
    return xs, ys, zs


def plot_xy(xs, ys, title):
    plt.figure(figsize=(8, 6))
    plt.plot(xs, ys, "o-", markersize=3, linewidth=1, alpha=0.7)
    plt.xlabel("X, мм")
    plt.ylabel("Y, мм")
    plt.title(title)
    plt.axis("equal")
    plt.grid(True)
    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(description="Построение графика XY из логов трилатерации.")
    parser.add_argument("--file", required=True, help="Путь к файлу логов, например data/trilateration/log3.txt")
    args = parser.parse_args()

    xs, ys, _ = parse_positions(args.file)
    if xs and ys:
        plot_xy(xs, ys, f"Траектория: {args.file}")
    else:
        print("[ERROR] Нет данных для отображения.", file=sys.stderr)


if __name__ == "__main__":
    main()
