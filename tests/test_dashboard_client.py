"""Run the actual dashboard client against camera loss/reopen scenarios."""
from pathlib import Path
import shutil
import subprocess
import unittest


class DashboardCameraClientTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which('node'), 'Node is needed for the camera client check')
    def test_offline_retry_and_camera_generation_resubscribe_without_motion(self):
        result = subprocess.run([shutil.which('node'),str(Path(__file__).with_name('dashboard_camera_retry.cjs'))],
            capture_output=True,text=True,encoding='utf-8',timeout=15)
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)

    @unittest.skipUnless(shutil.which('node'), 'Node is needed for the selection client check')
    def test_four_sequences_restart_clearing_stale_polls_and_no_motion_requests(self):
        result = subprocess.run([shutil.which('node'),str(Path(__file__).with_name('dashboard_sequence_selection.cjs'))],
            capture_output=True,text=True,encoding='utf-8',timeout=15)
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)


if __name__ == '__main__': unittest.main()
