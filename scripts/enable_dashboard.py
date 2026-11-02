#!/usr/bin/env python3
"""One-time setup for button-controlled missions; keep all taught arm angles."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parent.parent


def enable_dashboard(path):
    path = Path(path)
    original = path.read_bytes()
    values = json.loads(original.decode('utf-8'))
    if not isinstance(values,dict):
        raise ValueError('mission_config.json must contain an object')
    values.update(web_start_required=True,auto_approach_enabled=True,qr_enabled=True)
    backup = path.with_name(path.name+'.before-dashboard')
    if not backup.exists(): backup.write_bytes(original)
    stat = path.stat()
    temp = path.with_name(path.name+'.dashboard.tmp')
    temp.write_text(json.dumps(values,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    os.chmod(temp,stat.st_mode & 0o7777)
    if hasattr(os,'chown'):
        try: os.chown(temp,stat.st_uid,stat.st_gid)
        except PermissionError: pass
    temp.replace(path)
    return backup


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--install-service',action='store_true',help='Register this checkout and start its web server at boot')
    args = parser.parse_args()
    if args.install_service and (sys.platform!='linux' or not hasattr(os,'geteuid') or os.geteuid()!=0):
        parser.error('On Raspberry Pi, run sudo python3 scripts/enable_dashboard.py --install-service')
    backup = enable_dashboard(ROOT/'mission_config.json')
    print('Button start enabled. Original settings backed up to '+str(backup))
    print('Taught arm_poses.json and arm_enabled are preserved.')
    if args.install_service:
        subprocess.run([sys.executable,str(ROOT/'scripts/register_service.py'),'--start'],check=True)
    print('Open http://ROBOT_IP:5000/dashboard and press the mission start button.')


if __name__ == '__main__': main()
