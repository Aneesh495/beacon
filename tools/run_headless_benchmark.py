#!/usr/bin/env python3
"""
tools/run_headless_benchmark.py

Automated orchestrator executing Beacon's benchmark suite headlessly.
Parses output logs, measures runtime duration, and generates consolidated reports.
"""

import subprocess
import sys
import time

def run_headless_tests() -> int:
    print("=" * 72)
    print("   BEACON HEADLESS AUTOMATED BENCHMARK SUITE")
    print("=" * 72)
    start_time = time.time()
    
    cmd = ["./scripts/run_tests.sh"]
    process = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True
    )
    
    passed_suites = 0
    total_assertions = 0
    
    for line in iter(process.stdout.readline, ""):
        sys.stdout.write(line)
        if "[PASS]" in line:
            if "assertions" in line:
                try:
                    num_str = line.split("(")[-1].split("assertions")[0].strip()
                    total_assertions += int(num_str)
                except (ValueError, IndexError):
                    pass
        elif "Suites Executed:" in line:
            try:
                passed_str = line.split("Passed:")[1].split(",")[0].strip()
                passed_suites = int(passed_str)
            except (ValueError, IndexError):
                pass
                
    process.wait()
    elapsed = time.time() - start_time
    
    print("\n" + "=" * 72)
    print(f"BENCHMARK COMPLETE: {elapsed:.2f}s elapsed, {passed_suites} suites passed, {total_assertions} assertions.")
    print("=" * 72)
    
    return process.returncode

if __name__ == "__main__":
    sys.exit(run_headless_tests())
