import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'vision'))
try:
    import numpy as np
    import official_garbage as O
except ImportError:  # pragma: no cover
    O = None


@unittest.skipIf(O is None, 'OpenCV/numpy not installed')
class DecodeTest(unittest.TestCase):
    def test_one_box_back_in_image_pixels(self):
        image = np.zeros((480, 640, 3), np.uint8)
        _, ratio, pad = O.letterbox(image)
        self.assertEqual((ratio, pad), (1.0, (0, 80)))
        out = np.zeros((25200, 8), np.float32)
        out[0] = [320, 320, 100, 50, 0.9, 0.1, 0.1, 0.95]                # Syringe, centred in the letterbox
        out[1] = [322, 321, 98, 52, 0.8, 0.1, 0.1, 0.9]                 # duplicate, removed by NMS
        out[2] = [100, 150, 40, 40, 0.9, 0.9, 0.05, 0.05]               # Fish_bone
        dets = O.decode(out, ratio, pad, image.shape[:2])
        self.assertEqual([d['label'] for d in dets], ['Syringe', 'Fish_bone'])
        self.assertEqual(dets[0]['bbox'], [270, 215, 370, 265])
        self.assertEqual(dets[0]['category'], 'hazardous')
