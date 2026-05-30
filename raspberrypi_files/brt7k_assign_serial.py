#!/usr/bin/env python3
"""Assign stable /dev links for BRT7K ESP32 boards from their boot banner."""

from __future__ import annotations

import argparse
import glob
import os
import select
import sys
import termios
import time
from pathlib import Path


BAUD_RATES = {
    115200: termios.B115200,
}


ROLE_LINKS = {
    "drive": "/dev/esp_drive",
    "gripper": "/dev/esp_gripper",
}


def configure_serial(fd: int, baud: int) -> None:
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
    attrs[3] = 0
    attrs[4] = BAUD_RATES[baud]
    attrs[5] = BAUD_RATES[baud]
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 1
    termios.tcsetattr(fd, termios.TCSANOW, attrs)


def read_banner(device: str, baud: int, timeout: float) -> str:
    fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        configure_serial(fd, baud)
        deadline = time.monotonic() + timeout
        chunks: list[bytes] = []

        while time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            readable, _, _ = select.select([fd], [], [], min(0.2, remaining))
            if not readable:
                continue

            try:
                data = os.read(fd, 512)
            except BlockingIOError:
                continue

            if data:
                chunks.append(data)
                text = b"".join(chunks).decode("utf-8", errors="ignore")
                if "BRT7K_ROLE=" in text:
                    return text

        return b"".join(chunks).decode("utf-8", errors="ignore")
    finally:
        os.close(fd)


def parse_role(text: str) -> str | None:
    for line in text.splitlines():
        line = line.strip()
        if not line.startswith("BRT7K_ROLE="):
            continue

        role = line.split("=", 1)[1].strip().lower()
        if role in ROLE_LINKS:
            return role

    return None


def replace_symlink(link: str, target: str, dry_run: bool) -> None:
    link_path = Path(link)
    if dry_run:
        print(f"{link} -> {target}")
        return

    if link_path.exists() or link_path.is_symlink():
        link_path.unlink()

    os.symlink(target, link)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Scan /dev/ttyUSB* and create /dev/esp_* links from ESP32 boot banners."
    )
    parser.add_argument("--baud", type=int, default=115200, choices=BAUD_RATES.keys())
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument(
        "--require",
        default="",
        help="Comma-separated roles that must be found, for example: drive,gripper",
    )
    parser.add_argument("devices", nargs="*", default=glob.glob("/dev/ttyUSB*"))
    args = parser.parse_args()

    found: dict[str, str] = {}

    for device in sorted(args.devices):
        try:
            banner = read_banner(device, args.baud, args.timeout)
        except OSError as exc:
            print(f"skip {device}: {exc}", file=sys.stderr)
            continue

        role = parse_role(banner)
        if role is None:
            continue

        found[role] = device
        replace_symlink(ROLE_LINKS[role], device, args.dry_run)

    required = {role.strip() for role in args.require.split(",") if role.strip()}
    unknown_required = sorted(required - set(ROLE_LINKS))
    if unknown_required:
        print(f"unknown required roles: {', '.join(unknown_required)}", file=sys.stderr)
        return 2

    missing = sorted(required - set(found))
    if missing:
        print(f"missing roles: {', '.join(missing)}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
