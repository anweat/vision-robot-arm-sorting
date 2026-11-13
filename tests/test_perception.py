"""语音/手势识别的纯函数映射与模型降级。

映射（文本→cmd、类别→cmd）不依赖模型，开发机可测；模型推理在缺目录/缺依赖时
必须抛 ModelUnavailable，不能因为开发机没有 ais_bench 就崩——检查文件发生在
import ais_bench 之前，所以这里能直接验证降级路径。
"""
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'web'))
import perception as P  # noqa: E402


class VoiceMapTest(unittest.TestCase):
    def test_start_keywords(self):
        for text in ("启动", "开始", "开始分拣", "请开始干活", "START"):
            self.assertEqual(P.voice_command(text), {"cmd": "start"}, text)

    def test_stop_keywords_win_over_start(self):
        # 顺序从安全到动作：含"停止"的句子不能被判成启动
        self.assertEqual(P.voice_command("先停止再开始"), {"cmd": "stop"})
        for text in ("停止", "停下", "停一下", "别动", "stop now"):
            self.assertEqual(P.voice_command(text), {"cmd": "stop"}, text)

    def test_dance_keyword(self):
        self.assertEqual(P.voice_command("跳个舞"), {"cmd": "run_preset", "preset": "dance"})
        self.assertEqual(P.voice_command("来个舞蹈"), {"cmd": "run_preset", "preset": "dance"})

    def test_unmatched_returns_none(self):
        self.assertIsNone(P.voice_command("今天天气不错"))
        self.assertIsNone(P.voice_command(""))
        self.assertIsNone(P.voice_command(None))           # type: ignore[arg-type]

    def test_unknown_preset_is_ignored(self):
        # 复位/归位暂无执行侧映射，不能误触发未接入的动作
        self.assertIsNone(P.voice_command("复位"))
        self.assertIsNone(P.voice_command("归位"))

    def test_returns_a_copy_not_the_shared_constant(self):
        a = P.voice_command("启动")
        a["cmd"] = "hacked"
        self.assertEqual(P.voice_command("启动"), {"cmd": "start"})


class GestureMapTest(unittest.TestCase):
    def test_four_labels_map_to_safe_actions(self):
        self.assertEqual(P.gesture_command("fist"), {"cmd": "gripper", "action": "grip"})
        self.assertEqual(P.gesture_command("palm"), {"cmd": "gripper", "action": "open"})
        self.assertEqual(P.gesture_command("like"), {"cmd": "start"})
        self.assertEqual(P.gesture_command("ok"), {"cmd": "stop"})

    def test_unknown_label_returns_none(self):
        self.assertIsNone(P.gesture_command("peace"))
        self.assertIsNone(P.gesture_command(""))

    def test_returns_a_copy(self):
        a = P.gesture_command("fist")
        a["action"] = "wide"
        self.assertEqual(P.gesture_command("fist"), {"cmd": "gripper", "action": "grip"})


class ModelFallbackTest(unittest.TestCase):
    """缺模型文件 / 缺依赖时必须抛 ModelUnavailable，且在 import ais_bench 之前就抛，
    这样开发机（没有 ais_bench）也能跑到这条断言。"""

    def test_missing_model_dir_raises_before_importing_ais_bench(self):
        with self.assertRaises(P.ModelUnavailable):
            P.transcribe(b"fake-wav-bytes", "/no/such/asr/dir")
        with self.assertRaises(P.ModelUnavailable):
            P.classify(b"fake-image-bytes", "/no/such/gesture/dir")

    def test_empty_input_raises_value_error(self):
        # 空数据是参数问题，不是模型问题：用 ValueError，调用方按 400 处理
        with self.assertRaises(ValueError):
            P.transcribe(b"", "/no/such/dir")
        with self.assertRaises(ValueError):
            P.classify(b"", "/no/such/dir")

    def test_bad_base64_raises_value_error(self):
        with self.assertRaises(ValueError):
            P.transcribe("not!base64!", "/no/such/dir")

    def test_grab_snapshot_failure_is_model_unavailable(self):
        # 相机地址不可达：降级成 ModelUnavailable，不让端点 500
        with self.assertRaises(P.ModelUnavailable):
            P.grab_snapshot("http://127.0.0.1:9/snapshot.jpg", timeout=0.2)


if __name__ == "__main__":
    unittest.main()
