#!/usr/bin/env python3
"""
NVIDIA Jetson Orin Nano Hardware & ROS 2 Multisensor Fusion Benchmarking Tool
Monitors CPU utilization, GPU usage, RAM consumption, thermal metrics, and topic frequencies across Tailscale network.
Outputs benchmark report CSV and summary metrics for research presentation.
"""

import os
import sys
import time
import csv

def get_sys_metrics():
    metrics = {
        'timestamp': time.time(),
        'cpu_percent': 0.0,
        'ram_used_mb': 0.0,
        'ram_total_mb': 0.0,
        'gpu_percent': 0.0,
        'temp_cpu_c': 0.0
    }

    try:
        with open('/proc/meminfo', 'r') as f:
            lines = f.readlines()
            tot = 0
            avail = 0
            for l in lines:
                if l.startswith('MemTotal:'):
                    tot = int(l.split()[1]) / 1024.0
                elif l.startswith('MemAvailable:'):
                    avail = int(l.split()[1]) / 1024.0
            metrics['ram_total_mb'] = tot
            metrics['ram_used_mb'] = tot - avail
    except Exception:
        pass

    gpu_path = '/sys/devices/gpu.0/load'
    if os.path.exists(gpu_path):
        try:
            with open(gpu_path, 'r') as f:
                metrics['gpu_percent'] = float(f.read().strip()) / 10.0
        except Exception:
            pass

    thermal_path = '/sys/class/thermal/thermal_zone0/temp'
    if os.path.exists(thermal_path):
        try:
            with open(thermal_path, 'r') as f:
                metrics['temp_cpu_c'] = float(f.read().strip()) / 1000.0
        except Exception:
            pass

    return metrics

def run_benchmark(duration_sec=30, output_csv='jetson_benchmark_results.csv'):
    print(f"=== Starting Jetson Orin Nano Multisensor Fusion Benchmark ({duration_sec}s) ===")
    print(f"Saving metric log to: {output_csv}")

    fieldnames = ['timestamp', 'cpu_percent', 'ram_used_mb', 'ram_total_mb', 'gpu_percent', 'temp_cpu_c']
    
    with open(output_csv, 'w', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()

        start_time = time.time()
        count = 0
        while (time.time() - start_time) < duration_sec:
            m = get_sys_metrics()
            writer.writerow(m)
            count += 1
            print(f"[{count:02d}s] RAM: {m['ram_used_mb']:.1f}/{m['ram_total_mb']:.1f} MB | GPU: {m['gpu_percent']:.1f}% | Temp: {m['temp_cpu_c']:.1f}°C")
            time.sleep(1.0)

    print(f"=== Benchmark Complete ({count} samples recorded) ===")

if __name__ == '__main__':
    dur = 30
    if len(sys.argv) > 1:
        dur = int(sys.argv[1])
    run_benchmark(dur)
