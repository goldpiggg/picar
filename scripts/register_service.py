#!/usr/bin/env python3
"""Point the existing Adeept service at this checkout; no package installation."""
import argparse
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parent.parent
SERVICE = 'Adeept_Robot.service'


def unit_text(root):
    root = str(root).replace('\\', '\\\\').replace('"', '\\"').replace('%', '%%')
    return f'''[Unit]
Description=Adeept PiCar controller with integrated QR recognition
After=network.target wifi-hotspot-manager.service

[Service]
Type=simple
User=root
WorkingDirectory="{root}/Server"
ExecStart=/usr/bin/python3 "{root}/Server/WebServer.py"
Restart=no
Environment=PYTHONUNBUFFERED=1

[Install]
WantedBy=multi-user.target
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--start', action='store_true', help='Start now; official startup initializes servos')
    args = parser.parse_args()
    if sys.platform != 'linux' or not hasattr(os, 'geteuid') or os.geteuid() != 0:
        parser.error('Run on Raspberry Pi using sudo python3 scripts/register_service.py')
    entry = ROOT / 'Server' / 'WebServer.py'
    if not entry.is_file() or not (ROOT / 'mission_config.json').is_file():
        parser.error('Incomplete checkout')
    target = Path('/etc/systemd/system') / SERVICE
    backup = target.with_suffix(target.suffix + '.before-picar')
    if target.exists() and not backup.exists():
        backup.write_bytes(target.read_bytes())
    subprocess.run(['systemctl', 'stop', SERVICE], check=True)
    target.write_text(unit_text(ROOT), encoding='utf-8')
    target.chmod(0o644)
    subprocess.run(['systemctl', 'daemon-reload'], check=True)
    subprocess.run(['systemctl', 'enable', SERVICE], check=True)
    if args.start:
        subprocess.run(['systemctl', 'start', SERVICE], check=True)
        subprocess.run(['systemctl', '--no-pager', 'status', SERVICE], check=True)
    else:
        print('Registered. Controller is stopped. Start with: sudo systemctl start ' + SERVICE)


if __name__ == '__main__':
    main()
