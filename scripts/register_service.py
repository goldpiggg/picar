#!/usr/bin/env python3
"""Point the existing Adeept service at this checkout; no package installation."""
import argparse
import os
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parent.parent
SERVICE = 'Adeept_Robot.service'


def unit_text(root):
    root = str(root)
    if any(char in root for char in ('\n','\r','\0')):
        raise ValueError('Service path must not contain control characters')
    # WorkingDirectory is a scalar path, not an ExecStart argument. Its parser
    # does not remove surrounding quotes: a leading quote makes it relative.
    directory = (root+'/Server').replace('%','%%')
    command_path = (root+'/Server/WebServer.py').replace('\\','\\\\').replace('"','\\"').replace('%','%%').replace('$','$$')
    return f'''[Unit]
Description=Adeept PiCar controller with integrated QR recognition
After=network.target wifi-hotspot-manager.service

[Service]
Type=simple
User=root
WorkingDirectory={directory}
ExecStart=/usr/bin/python3 "{command_path}"
Restart=no
Environment=PYTHONUNBUFFERED=1

[Install]
WantedBy=multi-user.target
'''


def validate_unit(text):
    """Use systemd's real parser without starting or stopping any service."""
    with tempfile.TemporaryDirectory(prefix='picar-unit-') as directory:
        candidate = Path(directory)/SERVICE
        candidate.write_text(text,encoding='utf-8')
        subprocess.run(['systemd-analyze','verify',str(candidate)],check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--start', action='store_true', help='Start now; official startup initializes servos')
    args = parser.parse_args()
    if sys.platform != 'linux' or not hasattr(os, 'geteuid') or os.geteuid() != 0:
        parser.error('Run on Raspberry Pi using sudo python3 scripts/register_service.py')
    entry = ROOT / 'Server' / 'WebServer.py'
    if not entry.is_file() or not (ROOT / 'mission_config.json').is_file():
        parser.error('Incomplete checkout')
    text = unit_text(ROOT)
    validate_unit(text)  # Reject malformed settings before replacing the unit.
    target = Path('/etc/systemd/system') / SERVICE
    backup = target.with_suffix(target.suffix + '.before-picar')
    if target.exists() and not backup.exists():
        backup.write_bytes(target.read_bytes())
    target.write_text(text, encoding='utf-8')
    target.chmod(0o644)
    subprocess.run(['systemctl', 'daemon-reload'], check=True)
    # Repair the manager's bad-unit setting before asking it to stop/start.
    # Replacing and reloading a unit does not itself start its controller.
    subprocess.run(['systemctl', 'stop', SERVICE], check=True)
    subprocess.run(['systemctl', 'enable', SERVICE], check=True)
    if args.start:
        subprocess.run(['systemctl', 'start', SERVICE], check=True)
        subprocess.run(['systemctl', '--no-pager', 'status', SERVICE], check=True)
    else:
        print('Registered. Controller is stopped. Start with: sudo systemctl start ' + SERVICE)


if __name__ == '__main__':
    main()
