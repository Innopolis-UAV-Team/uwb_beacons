#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import os
import re
import sys
from statistics import mean, median
import math

DATA_PATTERN = re.compile(r"data:\s*(-?\d+)", re.IGNORECASE)

def is_number(s: str) -> bool:
    try:
        float(s.replace(",", "."))
        return True
    except Exception:
        return False

def parse_number(s: str) -> float:
    # поддержка папок вида "80" или "80.0" или "80,0"
    return float(s.replace(",", "."))

def read_mm_values_from_txt(path: str):
    values_mm = []
    #print(path)
    try:
        with open(path, "r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                m = DATA_PATTERN.search(line)
                #print(m)
                if m:
                    values_mm.append(int(m.group(1)))
    except Exception as e:
        print(f"[WARN] Не удалось прочитать '{path}': {e}", file=sys.stderr)
    return values_mm

def compute_stats(values_m, ref_m):
    n = len(values_m)
    if n == 0:
        return None
    mu = mean(values_m)
    med = median(values_m)
    # выборочное СКО (ddof=1)
    if n > 1:
        var = sum((x - mu) ** 2 for x in values_m) / (n - 1)
        std = math.sqrt(var)
    else:
        std = float("nan")
    bias = mu - ref_m
    return mu, med, std, bias, n

def main():
    ap = argparse.ArgumentParser(description="Анализ показаний датчика по папкам.")
    ap.add_argument("--root", default="data", help="Корневая папка с измерениями (по умолчанию: ./data)")
    ap.add_argument("--out", default="results.txt", help="Файл для вывода результатов (по умолчанию: results.txt)")
    args = ap.parse_args()

    root = args.root
    if not os.path.isdir(root):
        print(f"[ERROR] Папка '{root}' не найдена.", file=sys.stderr)
        sys.exit(1)

    rows = []
    for name in sorted(os.listdir(root), key=lambda s: (not is_number(s), s)):
        subdir = os.path.join(root, name)
        if not os.path.isdir(subdir):
            continue
        if not is_number(name):
            print(f"[INFO] Пропускаю подпапку '{name}' (непохоже на число см).", file=sys.stderr)
            continue

        ref_cm = parse_number(name)
        ref_m = ref_cm / 100.0

        all_mm = []
        for fname in os.listdir(subdir):
            if not fname.lower().endswith(".txt"):
                continue
            path = os.path.join(subdir, fname)
            vals = read_mm_values_from_txt(path)
            all_mm.extend(vals)

        values_m = [v / 1000.0 for v in all_mm]  # мм -> м
        stats = compute_stats(values_m, ref_m)
        if stats is None:
            print(f"[WARN] В папке '{name}' нет валидных данных.", file=sys.stderr)
            continue

        mu, med, std, bias, n = stats
        print(ref_m, mu)
        rows.append((ref_m, mu, med, std, bias, n))

    # Сохраняем результаты (TSV)
    header = "ref_m\tmean_m\tmedian_m\tstd_m\tmean_error_m\tcount"
    try:
        with open(args.out, "w", encoding="utf-8") as f:
            f.write(header + "\n")
            for ref_m, mu, med, std, bias, n in sorted(rows, key=lambda r: r[0]):
                f.write(f"{ref_m:.6f}\t{mu:.6f}\t{med:.6f}\t{(std if math.isfinite(std) else float('nan')):.6f}\t{bias:.6f}\t{n}\n")
    except Exception as e:
        print(f"[ERROR] Не удалось записать '{args.out}': {e}", file=sys.stderr)
        sys.exit(1)

    print(f"Готово. Результаты в '{args.out}'. Найдено {len(rows)} наборов измерений.")

if __name__ == "__main__":
    main()
