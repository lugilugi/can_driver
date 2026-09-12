#!/usr/bin/env python3
"""Reject legacy V4 names outside explicitly historical release notes."""

from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
HISTORICAL_FILES = {ROOT / "CHANGELOG.md"}

LEGACY_TOKENS = (
    "DASH_" + "S" + "TAT",
    "AUX_" + "C" + "TRL",
    "PWR_" + "MON" + "ITOR",
    "PWR_" + "ENER" + "GY",
    "network_" + "pedal_t",
    "network_" + "aux_" + "ctrl_t",
    "network_" + "pwr_" + "monitor_780_t",
    "network_" + "pwr_" + "monitor_740_t",
    "network_" + "pwr_" + "energy_t",
    "network_" + "dash_" + "stat_t",
    "NETWORK_" + "PEDAL_" + "FRAME_ID",
    "NETWORK_" + "AUX_" + "CTRL_",
    "NETWORK_" + "PWR_" + "MONITOR_",
    "NETWORK_" + "PWR_" + "ENERGY_",
    "NETWORK_" + "DASH_" + "STAT_",
    "brake_" + "light",
    "kill_" + "flag",
    "raw_" + "throttle_adc",
)


def should_skip(path: Path) -> bool:
    relative_parts = path.relative_to(ROOT).parts
    if any(part in {".git", "build", "__pycache__", ".venv"} for part in relative_parts):
        return True
    if path.suffix.lower() in {".o", ".a", ".elf", ".map", ".pyc"}:
        return True
    return False


def main() -> int:
    violations: list[tuple[Path, int, str, str]] = []

    for path in ROOT.rglob("*"):
        if not path.is_file() or should_skip(path) or path in HISTORICAL_FILES:
            continue

        try:
            lines = path.read_text(encoding="utf-8").splitlines()
        except (UnicodeDecodeError, OSError):
            continue

        for line_number, line in enumerate(lines, start=1):
            for token in LEGACY_TOKENS:
                if token in line:
                    violations.append((path, line_number, token, line.strip()))

    if violations:
        for path, line_number, token, line in violations:
            print(f"{path}:{line_number}: legacy token {token!r}: {line}")
        return 1

    print("no legacy V4 references found outside CHANGELOG.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
