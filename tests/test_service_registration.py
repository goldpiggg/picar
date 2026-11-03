"""Check service paths using systemd's actual Linux parser when available."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from unittest.mock import patch
import scripts.register_service as registration

from scripts.register_service import SERVICE, unit_text, validate_unit


class ServiceUnitTests(unittest.TestCase):
    def test_registration_repairs_and_reloads_bad_unit_before_stop(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)/'picar'
            (root/'Server').mkdir(parents=True)
            (root/'Server/WebServer.py').write_text('# not executed',encoding='utf-8')
            (root/'mission_config.json').write_text('{}',encoding='utf-8')
            etc = Path(directory)/'systemd'
            etc.mkdir()
            target = etc/SERVICE
            target.write_text('WorkingDirectory="/broken"',encoding='utf-8')
            backup = target.with_suffix(target.suffix+'.before-picar')
            backup.write_text('original backup',encoding='utf-8')
            calls = []
            def run(args,check):
                calls.append(args)
                self.assertTrue(check)
                if args[1]=='stop':
                    self.assertEqual(calls[-2],['systemctl','daemon-reload'])
                    self.assertIn('WorkingDirectory='+str(root)+'/Server\n',target.read_text(encoding='utf-8'))
                return subprocess.CompletedProcess(args,0)
            def mapped_path(value):
                return etc if str(value)=='/etc/systemd/system' else Path(value)
            with patch.object(registration,'ROOT',root),patch.object(registration,'Path',mapped_path),\
                 patch.object(registration,'validate_unit') as verify,\
                 patch.object(registration.subprocess,'run',side_effect=run),\
                 patch.object(registration.sys,'platform','linux'),\
                 patch.object(registration.os,'geteuid',return_value=0,create=True),\
                 patch.object(registration.sys,'argv',['register_service.py','--start']):
                registration.main()
            verify.assert_called_once()
            self.assertEqual(backup.read_text(encoding='utf-8'),'original backup')
            self.assertEqual(calls,[['systemctl','daemon-reload'],['systemctl','stop',SERVICE],
                ['systemctl','enable',SERVICE],['systemctl','start',SERVICE],
                ['systemctl','--no-pager','status',SERVICE]])

    def test_scalar_path_has_no_argument_quotes_and_exec_path_stays_quoted(self):
        text = unit_text('/home/pi/picar kit')
        self.assertIn('\nWorkingDirectory=/home/pi/picar kit/Server\n',text)
        self.assertIn('\nExecStart=/usr/bin/python3 "/home/pi/picar kit/Server/WebServer.py"\n',text)

    def test_percent_specifiers_and_exec_environment_expansion_are_escaped(self):
        text = unit_text('/home/pi/robot% $kit')
        self.assertIn('WorkingDirectory=/home/pi/robot%% $kit/Server\n',text)
        self.assertIn('ExecStart=/usr/bin/python3 "/home/pi/robot%% $$kit/Server/WebServer.py"\n',text)
        for path in ['/home/pi/a\nb','/home/pi/a\rb','/home/pi/a\0b']:
            with self.assertRaises(ValueError): unit_text(path)

    def test_validation_uses_a_temporary_unit_and_propagates_failure(self):
        def reject(args,check):
            candidate = Path(args[-1])
            self.assertEqual(args[:2],['systemd-analyze','verify'])
            self.assertEqual(candidate.name,SERVICE)
            self.assertEqual(candidate.read_text(encoding='utf-8'),unit_text('/home/pi/picar'))
            self.assertTrue(check)
            raise subprocess.CalledProcessError(1,args)
        with patch('scripts.register_service.subprocess.run',side_effect=reject):
            with self.assertRaises(subprocess.CalledProcessError):
                validate_unit(unit_text('/home/pi/picar'))

    @unittest.skipUnless(os.name=='posix' and shutil.which('systemd-analyze'),'systemd parser only available on Linux')
    def test_real_systemd_parser_accepts_paths_and_rejects_old_quoted_directory(self):
        with tempfile.TemporaryDirectory(prefix='picar unit ') as directory:
            root = Path(directory)/'robot %kit $test'
            (root/'Server').mkdir(parents=True)
            text = unit_text(root)
            unit = Path(directory)/SERVICE
            unit.write_text(text,encoding='utf-8')
            good = subprocess.run(['systemd-analyze','verify',str(unit)],capture_output=True,text=True,timeout=30)
            self.assertEqual(good.returncode,0,good.stdout+good.stderr)
            directory_line = next(line for line in text.splitlines() if line.startswith('WorkingDirectory='))
            broken = text.replace(directory_line,'WorkingDirectory="'+directory_line.split('=',1)[1]+'"')
            unit.write_text(broken,encoding='utf-8')
            bad = subprocess.run(['systemd-analyze','verify',str(unit)],capture_output=True,text=True,timeout=30)
            self.assertNotEqual(bad.returncode,0)
            self.assertIn('WorkingDirectory',bad.stdout+bad.stderr)


if __name__ == '__main__': unittest.main()
