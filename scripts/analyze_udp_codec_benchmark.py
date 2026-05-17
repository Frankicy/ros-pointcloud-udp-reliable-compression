#!/usr/bin/env python3
import csv
import glob
import os
import statistics

def is_number(x):
    try:
        float(x)
        return True
    except Exception:
        return False

def analyze_file(path):
    print("")
    print("File:", path)
    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
        fields = reader.fieldnames or []
    print("Rows:", len(rows))
    for field in fields:
        values = []
        for row in rows:
            value = str(row.get(field, "")).strip()
            if value and is_number(value):
                values.append(float(value))
        if values:
            print(field + ": count=" + str(len(values)) +
                  ", avg=" + format(statistics.mean(values), ".6f") +
                  ", min=" + format(min(values), ".6f") +
                  ", max=" + format(max(values), ".6f"))

def main():
    paths = []
    for folder in ["benchmark_results", "logs", "results", "."]:
        if os.path.isdir(folder):
            paths += glob.glob(os.path.join(folder, "*.csv"))
    paths = sorted(set(paths))
    if not paths:
        print("No CSV files found.")
        print("Checked: benchmark_results, logs, results, current directory")
        return
    for path in paths:
        analyze_file(path)

if __name__ == "__main__":
    main()
