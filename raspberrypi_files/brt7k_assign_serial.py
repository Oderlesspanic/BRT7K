#!/usr/bin/env python3
"""Assign stable /dev links for BRT7K ESP32 boards from their boot banner."""

from __future__ import annotations

import argparse
import fcntl
import glob
import os
import select
import subprocess
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
    "lidar": "/dev/lidar",
}


USB_ID_ROLES = {
    ("10c4", "ea60"): "lidar",
}


ESP_USB_IDS = {
    ("1a86", "7523"),
}

TIOCMBIS = 0x5416
TIOCMBIC = 0x5417
TIOCM_DTR = 0x002
TIOCM_RTS = 0x004


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


def modem_control(fd: int, request: int, bits: int) -> None:
    fcntl.ioctl(fd, request, bits.to_bytes(4, sys.byteorder))


def reset_esp32(fd: int) -> None:
    # Common ESP32 auto-reset wiring: RTS controls EN, DTR controls GPIO0.
    # Keep GPIO0 high, pulse EN low, then release reset into normal boot.
    modem_control(fd, TIOCMBIC, TIOCM_DTR)
    modem_control(fd, TIOCMBIS, TIOCM_RTS)
    time.sleep(0.1)
    modem_control(fd, TIOCMBIC, TIOCM_RTS)
    time.sleep(0.2)


def read_banner(device: str, baud: int, timeout: float, reset: bool) -> str:
    fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        configure_serial(fd, baud)
        if reset:
            reset_esp32(fd)

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


def usb_ids(device: str) -> tuple[str, str] | None:
    tty_name = Path(device).name
    sys_path = (Path("/sys/class/tty") / tty_name / "device").resolve()

    for path in [sys_path, *sys_path.parents]:
        vendor_file = path / "idVendor"
        product_file = path / "idProduct"
        if not vendor_file.exists() or not product_file.exists():
            continue

        try:
            vendor = vendor_file.read_text(encoding="utf-8").strip().lower()
            product = product_file.read_text(encoding="utf-8").strip().lower()
        except OSError:
            continue

        return vendor, product

    try:
        result = subprocess.run(
            ["udevadm", "info", "-q", "property", "-n", device],
            check=False,
            capture_output=True,
            text=True,
        )
    except OSError:
        return None

    properties: dict[str, str] = {}
    for line in result.stdout.splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        properties[key] = value.strip().lower()

    vendor = properties.get("ID_VENDOR_ID")
    product = properties.get("ID_MODEL_ID")
    if vendor and product:
        return vendor, product

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
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--no-reset", action="store_true")
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument(
        "--require",
        default="",
        help="Comma-separated roles that must be found, for example: drive,gripper",
    )
    parser.add_argument("devices", nargs="*", default=glob.glob("/dev/ttyUSB*"))
    args = parser.parse_args()

    found: dict[str, str] = {}

    for device in sorted(args.devices):
        ids = usb_ids(device)
        if args.verbose:
            print(f"scan {device}: usb_id={ids}", file=sys.stderr)

        if ids in USB_ID_ROLES:
            role = USB_ID_ROLES[ids]
            found[role] = device
            replace_symlink(ROLE_LINKS[role], device, args.dry_run)
            continue

        try:
            banner = read_banner(
                device,
                args.baud,
                args.timeout,
                reset=(ids in ESP_USB_IDS and not args.no_reset),
            )
        except OSError as exc:
            print(f"skip {device}: {exc}", file=sys.stderr)
            continue

        role = parse_role(banner)
        if args.verbose:
            preview = banner[-200:].replace("\n", "\\n")
            print(f"scan {device}: role={role}, banner_tail={preview}", file=sys.stderr)

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
