"""Run the real client script against an out-of-order request/stop scenario."""
from pathlib import Path
import shutil
import subprocess
import unittest


class PositionClientTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which('node'), 'Node is needed for the pose recall flow check')
    def test_recall_adjust_and_resave_flow(self):
        result = subprocess.run([shutil.which('node'), str(Path(__file__).with_name('positions_recall.cjs'))],
            capture_output=True,text=True,encoding='utf-8',timeout=15)
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)

    @unittest.skipUnless(shutil.which('node'), 'Node is needed for the browser request race check')
    def test_stop_cancels_a_recall_still_waiting_for_preview_angles(self):
        result = subprocess.run([shutil.which('node'), str(Path(__file__).with_name('positions_client_stop.cjs'))],
            capture_output=True, text=True, timeout=15)
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)

    @unittest.skipUnless(shutil.which('node'), 'Node is needed for the data error display check')
    def test_bad_pose_data_is_distinguished_from_a_server_disconnect(self):
        result = subprocess.run([shutil.which('node'), str(Path(__file__).with_name('positions_data_errors.cjs'))],
            capture_output=True,text=True,encoding='utf-8',timeout=15)
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)


if __name__ == '__main__': unittest.main()
