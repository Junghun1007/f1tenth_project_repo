"""camera_driver C++ 터널 프레임 통계 = tunnel_tcn_test Python frame_stats (같은 특징 16개가 나오는지).

C++ 컴파일러·OpenCV(pkg-config opencv4)가 없으면 건너뛴다.
    python3 -m pytest -q src/camera_driver/test/test_tunnel_frame_stats.py
"""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import types
import unittest

import numpy as np

SRC = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SRC / 'tunnel_tcn_test'))

PROGRAM = r'''
#include "camera_driver/tunnel_frame_stats.hpp"
#include <cstdio>
#include <fstream>
#include <vector>
int main(int argc, char ** argv) {
  // argv: <파일> <폭> <높이> <stride> <luma=1|gray=0>
  std::ifstream f(argv[1], std::ios::binary);
  std::vector<unsigned char> d((std::istreambuf_iterator<char>(f)), {});
  const int w = std::atoi(argv[2]), h = std::atoi(argv[3]), stride = std::atoi(argv[4]);
  cv::Mat gray = std::atoi(argv[5]) ? camera_driver::tunnel_gray_from_luma(d.data(), w, h, stride)
                                    : cv::Mat(h, w, CV_8UC1, d.data(), stride).clone();
  const auto s = camera_driver::tunnel_frame_stats(gray);
  std::printf("%.9f %.9f %.9f %.9f %.9f %.9f %.9f\n", s.mean, s.std, s.center_mean, s.surround_mean, s.p10, s.p90,
              s.center_dark_fraction);
  if (argc > 6) { std::ofstream o(argv[6], std::ios::binary); o.write(reinterpret_cast<char *>(gray.data), gray.total()); }
}
'''


def _opencv_flags():
    if shutil.which('c++') is None or shutil.which('pkg-config') is None:
        return None
    cflags = subprocess.run(['pkg-config', '--cflags', 'opencv4'], capture_output=True, text=True)
    libdir = subprocess.run(['pkg-config', '--variable=libdir', 'opencv4'], capture_output=True, text=True)
    if cflags.returncode or libdir.returncode:
        return None
    lib = libdir.stdout.strip()   # 쓰는 모듈만 링크 (전체 목록은 환경에 따라 빠진 라이브러리가 있음)
    return cflags.stdout.split() + [f'-L{lib}', f'-Wl,-rpath,{lib}', '-lopencv_imgproc', '-lopencv_core']


class TunnelFrameStatsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        flags = _opencv_flags()
        if flags is None:
            raise unittest.SkipTest('C++ compiler or OpenCV (pkg-config opencv4) unavailable')
        try:
            import cv2  # noqa: F401  (Python 쪽 기준 계산에 필요)
        except ImportError:
            raise unittest.SkipTest('python3-opencv unavailable')
        cls.tmp = tempfile.TemporaryDirectory()
        tmp = Path(cls.tmp.name)
        (tmp / 'main.cpp').write_text(PROGRAM)
        cls.exe = tmp / 'stats'
        subprocess.run(['c++', '-std=c++17', '-O1', '-I', str(SRC / 'camera_driver/include'), str(tmp / 'main.cpp'),
                        str(SRC / 'camera_driver/src/tunnel_frame_stats.cpp'), '-o', str(cls.exe), *flags], check=True)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def cpp(self, data, w, h, stride, luma, out=None):
        path = Path(self.tmp.name) / 'in.raw'
        path.write_bytes(np.ascontiguousarray(data).tobytes())
        args = [str(self.exe), str(path), str(w), str(h), str(stride), '1' if luma else '0'] + ([str(out)] if out else [])
        values = [float(v) for v in subprocess.run(args, check=True, capture_output=True, text=True).stdout.split()]
        from tunnel_tcn_test.features import FRAME_STATS
        return dict(zip(FRAME_STATS, values))

    def test_golden_replay_features_match(self):
        """학습 때 특징(golden_replay.npz)을 C++ 통계 → push_stats로 그대로 재현."""
        from tunnel_tcn_test.features import FeatureExtractor, preprocess
        with np.load(SRC / 'tunnel_tcn_test/models/golden_replay.npz') as data:
            images, expected = data['synthetic_images'], data['synthetic_features']
        extractor = FeatureExtractor()
        for i, image in enumerate(images):
            gray = preprocess(image)
            stats = self.cpp(gray, gray.shape[1], gray.shape[0], gray.shape[1], luma=False)
            stats = {k: float(np.float32(v)) for k, v in stats.items()}   # 메시지는 float32
            np.testing.assert_allclose(extractor.push_stats(stats, i / 30.), expected[i], atol=2e-5)

    def test_nv12_luma_path_matches_python_preview_decoder(self):
        """NV12 원본(패딩 있는 stride) → C++ 축소·LUT = display.mono_image, 통계도 같음."""
        from tunnel_tcn_test.display import mono_image
        from tunnel_tcn_test.features import frame_stats
        rng = np.random.default_rng(3)
        w, h, stride = 1280, 800, 1344
        y = rng.integers(0, 256, (h, stride), np.uint8)
        y[200:600, 400:900] //= 4   # 가운데 어두운 영역 (터널 입구처럼)
        nv12 = np.concatenate([y, rng.integers(0, 256, (h // 2, stride), np.uint8)])
        message = types.SimpleNamespace(height=h, width=w, step=stride, encoding='nv12', data=nv12.tobytes())
        expected_gray = mono_image(message)
        out = Path(self.tmp.name) / 'gray.raw'
        stats = self.cpp(nv12, w, h, stride, luma=True, out=out)
        gray = np.frombuffer(out.read_bytes(), np.uint8).reshape(400, 640)
        np.testing.assert_array_equal(gray, expected_gray)
        reference = frame_stats(expected_gray)
        for key, value in reference.items():
            self.assertAlmostEqual(stats[key], value, delta=1e-3 * max(1.0, abs(value)), msg=key)


if __name__ == '__main__':
    unittest.main()
