#!/usr/bin/env python3
"""
Pre-commit validation script: Enforces zero em/en dashes across all repository files.

This script scans source files, documentation, scripts, and configuration files
for Unicode em dashes (U+2014), en dashes (U+2013), horizontal bars (U+2015),
and ASCII double-dashes used as typographic em dashes.

Exit code 0: All files conform to the strict zero em dash policy.
Exit code 1: Violations detected with file paths and line numbers.
"""

import os
import sys

# Allowed file extensions to inspect
INSPECTED_EXTENSIONS = {
    ".gd", ".py", ".md", ".sh", ".json", ".txt", ".yml", ".yaml",
    ".tscn", ".godot", ".cfg", ".toml", ".c", ".h", ".cpp", ".hpp"
}

# Directories to skip
EXCLUDED_DIRS = {
    ".git", ".godot", "__pycache__", "build", "dist", ".import"
}

# Forbidden characters and sequences
FORBIDDEN_PATTERNS = [
    ("\u2014", "Unicode Em Dash (U+2014)"),
    ("\u2013", "Unicode En Dash (U+2013)"),
    ("\u2015", "Unicode Horizontal Bar (U+2015)"),
]

def scan_file(filepath: str) -> list:
    """Scans a single file for forbidden dash characters and sequences."""
    violations = []
    try:
        with open(filepath, "r", encoding="utf-8", errors="replace") as f:
            for line_idx, line in enumerate(f, start=1):
                for pattern, name in FORBIDDEN_PATTERNS:
                    if pattern in line:
                        col = line.find(pattern) + 1
                        violations.append((filepath, line_idx, col, name, line.strip()))
                # Check for double dash surrounded by spaces as em dash substitute
                double_dash_sub = " " + "-" * 2 + " "
                if double_dash_sub in line and filepath != os.path.abspath(__file__):
                    col = line.find(double_dash_sub) + 1
                    violations.append((filepath, line_idx, col, "ASCII Double-Dash as Em Dash", line.strip()))
    except Exception as err:
        violations.append((filepath, 0, 0, f"Read error: {err}", ""))
    return violations

def main() -> int:
    root_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    total_violations = []

    for dirpath, dirnames, filenames in os.walk(root_dir):
        # Exclude directories in-place
        dirnames[:] = [d for d in dirnames if d not in EXCLUDED_DIRS]
        for fname in filenames:
            ext = os.path.splitext(fname)[1].lower()
            if ext in INSPECTED_EXTENSIONS:
                full_path = os.path.join(dirpath, fname)
                v = scan_file(full_path)
                if v:
                    total_violations.extend(v)

    if total_violations:
        print("=" * 72)
        print("EM DASH POLICY VIOLATIONS DETECTED")
        print("=" * 72)
        for filepath, line_idx, col, rule, snippet in total_violations:
            rel_path = os.path.relpath(filepath, root_dir)
            print(f"{rel_path}:{line_idx}:{col} -> {rule}")
            print(f"  Snippet: {snippet}")
        print("=" * 72)
        print(f"Total violations found: {len(total_violations)}")
        return 1

    print("Em Dash Scanner: Clean pass. Zero violations found across all files.")
    return 0

if __name__ == "__main__":
    sys.exit(main())
