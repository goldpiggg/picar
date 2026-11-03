"""Native side decoder, complementary results and real-image regressions."""
import unittest
from types import SimpleNamespace
from unittest.mock import patch
from pathlib import Path
try:
    import cv2
    import numpy as np
    import zxingcpp
except ImportError:
    zxingcpp=None
from picar_mission.qr_scan import QRReader
from picar_mission.sequence_selection import classify_side_start


@unittest.skipIf(zxingcpp is None,'ZXing/OpenCV unavailable')
class NativeSideTests(unittest.TestCase):
    def test_clear_pair_decides_without_opencv_decoding(self):
        from test_side_qr_decode import side_frame
        reader=QRReader(cv2)
        with patch.object(reader,'detector',SimpleNamespace()),patch.object(reader,'aruco',SimpleNamespace()):
            found=reader.detect_side(side_frame(150))
        self.assertEqual({d['label'] for d in found},{'PICK','PLACE'})
        self.assertEqual(classify_side_start(found,'LEFT'),'LEFT-B')

    def test_complementary_decoders_are_combined(self):
        reader=QRReader(cv2)
        quad=np.array([[[400,100],[500,100],[500,200],[400,200]]],np.float32)
        reader.aruco=SimpleNamespace(detectAndDecodeMulti=lambda frame:(True,['PLACE'],quad,None))
        with patch.object(reader,'_read_zxing',return_value=[dict(text='PICK',label='PICK',center_normalized=[.2,.3])]):
            found=reader.detect_side(np.zeros((480,640),np.uint8))
        self.assertEqual({d['label'] for d in found},{'PICK','PLACE'})
        self.assertEqual(classify_side_start(found,'LEFT'),'LEFT-B')

    def test_native_failure_falls_back_to_opencv(self):
        from test_side_qr_decode import side_frame
        reader=QRReader(cv2)
        def fail(*args,**kwargs):raise RuntimeError('native decoder test fault')
        reader.zxing=SimpleNamespace(read_barcodes=fail,BarcodeFormat=zxingcpp.BarcodeFormat)
        found=reader.detect_side(side_frame(150))
        self.assertEqual({d['label'] for d in found},{'PICK','PLACE'})
        self.assertEqual(reader.backend,'OPENCV')

    def test_real_frames_select_and_lone_code_does_not_guess(self):
        fixtures=Path(__file__).parent/'fixtures'
        for name,want in [('side_crumpled.png','LEFT-B'),('side_place_unread.png','LEFT-B'),('side_pick_only.png',None)]:
            with self.subTest(frame=name):
                frame=cv2.imdecode(np.frombuffer((fixtures/name).read_bytes(),np.uint8),1)
                found=QRReader(cv2).detect_side(frame,one_is_enough=True)
                self.assertEqual(classify_side_start(found,'LEFT',settled=True),want)
