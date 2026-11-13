"""W4 定时启动（src/web/serve.py 的 Timers）。标准库 unittest。

覆盖需求规格 W4 的口径：默认关闭、可查看、可取消、到点前 lead 秒进入"待发"、
到点自动启动，以及"启动时刻偏差 ≤ 5 s"。

`tick()` / `status()` 的 now 可注入，所以这里的钟是**假的**：整套不到 0.1 秒，
不依赖线程调度也不真的是按顺序大约等待。只有最后一个用例真的把后台线程跑起来，
验证它能启能停。
"""
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'web'))
import serve as S  # noqa: E402

DAY = 86400


class TimersTest(unittest.TestCase):
    def setUp(self):
        self.fired = []
        self.base = 1_800_000_000.0        # 固定基准时刻，测试里的"现在"
        self.now = self.base               # 最后一个用例走真时钟，这里先给个值兜底
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)

    def make(self, **overrides):
        timers = S.Timers(Path(self.tmp.name) / 'timers.json', self._fire)
        for key, value in overrides.items():
            setattr(timers, key, value)
        return timers

    def _fire(self, timer):
        self.fired.append((timer, self.now))
        return dict(ok=True, started=True)

    def tick(self, timers, offset, lead=None):
        """把钟拨到 base+offset 并推进一次；返回那一刻的 status。"""
        self.now = self.base + offset
        timers.LEAD = timers.LEAD if lead is None else lead
        timers.tick(self.now)
        return timers.status(self.now)

    # ---- 默认关闭：不启用就没有这个功能 ----------------------------------
    def test_no_timers_file_means_the_feature_is_off(self):
        state = S.State(Path(self.tmp.name) / 'log.jsonl')
        self.assertIsNone(state.timers)
        self.assertFalse(state.status()['timer'].get('enabled', True))

    def test_the_real_lead_and_tick_are_the_documented_ones(self):
        """其余用例把这两个值调小了；这里确认生产取值没被改。"""
        self.assertEqual(S.Timers.LEAD, 5.0)    # W4：到点先提示 5 s，可在页面取消
        self.assertLess(S.Timers.TICK, 5.0)     # 检查周期必须远小于允许偏差 5 s

    # ---- 状态与可见性 -----------------------------------------------------
    def test_status_reports_countdown_before_it_dues(self):
        timers = self.make()
        self.assertEqual(timers.status(self.base)['count'], 0)
        timer = timers.add(self.base + 300)
        status = timers.status(self.base)
        self.assertEqual(status['count'], 1)
        self.assertAlmostEqual(status['at'], timer['at'], places=2)
        self.assertEqual(status['countdown_s'], 300)
        self.assertFalse(status['pending'])

    def test_garbage_in_the_task_file_does_not_break_the_service(self):
        path = Path(self.tmp.name) / 'broken.json'
        path.write_text('{ 这不是 JSON', encoding='utf-8')
        self.assertEqual(S.Timers(path, self._fire).list, [])

    # ---- 到点前进入"待发"，且此时还能取消 ---------------------------------
    def test_enters_pending_lead_seconds_before_due_and_can_still_be_cancelled(self):
        timers = self.make(LEAD=5.0)
        timer = timers.add(self.base + 10)
        self.tick(timers, 3)                       # 还没进 lead 窗口
        self.assertFalse(timers.status(self.base + 3)['pending'])
        status = self.tick(timers, 6)              # 距到点 4 s < lead 5 s
        self.assertTrue(status['pending'], '到点前应进入待发')
        self.assertTrue(timers.remove(timer['id']))
        self.tick(timers, 20)                      # 过了原定时刻也不该触发
        self.assertEqual(self.fired, [])

    # ---- 到点触发 ---------------------------------------------------------
    def test_fires_at_the_configured_time_within_the_5s_acceptance_limit(self):
        timers = self.make(LEAD=5.0, TICK=0.5)
        due = self.base + 10
        timers.add(due)
        self.tick(timers, 5)
        self.assertEqual(self.fired, [])
        self.now = due + 0.5                       # 一个 TICK 之内必然被检出
        timers.tick(self.now)
        self.assertTrue(self.fired, '定时器没有在到点后触发')
        fired_timer, fired_at = self.fired[0]
        self.assertAlmostEqual(fired_timer['at'], due, places=2)   # add 量化到毫秒
        self.assertLessEqual(abs(fired_at - due), 5.0)             # W4：偏差 ≤ 5 s
        self.assertEqual(timers.status(self.now)['count'], 0)      # 一次性任务跑完移除

    def test_once_timer_is_removed_but_daily_is_rescheduled_a_day_later(self):
        timers = self.make(LEAD=5.0)
        due = self.base + 10
        timers.add(due, repeat='daily')
        self.now = due + 1
        timers.tick(self.now)
        self.assertTrue(self.fired)
        self.assertEqual(timers.status(self.now)['count'], 1)          # 还在表里
        self.assertAlmostEqual(timers.next()['at'], due + DAY, delta=1.0)

    def test_the_fire_result_is_remembered_for_the_page(self):
        timers = self.make(LEAD=5.0)
        timers.add(self.base + 10)
        self.now = self.base + 11
        timers.tick(self.now)
        result = timers.status(self.now)['last_result']
        self.assertTrue(result['started'])               # 页面要能看到"到点确实发了"

    def test_a_failing_start_is_reported_not_swallowed(self):
        timers = S.Timers(Path(self.tmp.name) / 't.json',
                          lambda t: dict(ok=False, error='控制台没有任务启动入口'))
        timers.add(self.base + 10)
        timers.tick(self.base + 11)
        last = timers.status(self.base + 11)['last_result']
        self.assertFalse(last['ok'])
        self.assertIn('控制台', last['error'])

    # ---- 错过不补跑 -------------------------------------------------------
    def test_a_once_timer_missed_by_more_than_grace_is_dropped_not_fired(self):
        """停机一天后重启：一次性任务到点早过了，不能补跑机械臂。"""
        timers = self.make(LEAD=5.0)
        timers.add(self.base + 10)
        self.tick(timers, 10 + S.Timers.GRACE + 1)      # 已经过了宽限窗口
        self.assertEqual(self.fired, [])
        self.assertEqual(timers.status(self.now)['count'], 0)   # 直接收尾，不留僵尸任务
        self.assertFalse(timers.status(self.now)['last_result']['ok'])

    def test_a_missed_daily_timer_is_rescheduled_not_caught_up(self):
        """错过 3 天的 daily：重排到下一个未来时刻，而不是连续补触发 3 次。"""
        timers = self.make(LEAD=5.0)
        due = self.base + 10
        timers.add(due, repeat='daily')
        self.tick(timers, 10 + 3 * DAY)                  # 三天后才第一次 tick
        self.assertEqual(self.fired, [], '错过的 daily 不能补触发')
        nxt = timers.next()['at']
        self.assertGreater(nxt, self.now)                # 已排到未来
        self.assertLessEqual(nxt - self.now, DAY)
        self.tick(timers, nxt - self.base + 0.5)         # 到新时刻正常触发
        self.assertEqual(len(self.fired), 1)

    def test_a_timer_inside_the_grace_window_still_fires(self):
        timers = self.make(LEAD=5.0)
        timers.add(self.base + 10)
        self.tick(timers, 10 + S.Timers.GRACE - 0.5)     # 宽限内：正常触发
        self.assertTrue(self.fired)

    def test_a_crashing_fire_still_settles_the_task(self):
        """fire 抛异常：错误要留在 last_result 里，任务收尾，不能下个 tick 反复补触发。"""
        def boom(timer):
            raise RuntimeError('控制台掉线')
        timers = S.Timers(Path(self.tmp.name) / 't.json', boom)
        timers.add(self.base + 10)
        timers.tick(self.base + 11)
        last = timers.status(self.base + 11)['last_result']
        self.assertFalse(last['ok'])
        self.assertIn('控制台掉线', last['error'])
        self.assertEqual(timers.status(self.base + 11)['count'], 0)
        timers.tick(self.base + 12)                      # 不会再次触发
        self.assertIn('控制台掉线', timers.status(self.base + 12)['last_result']['error'])

    # ---- 取消 -------------------------------------------------------------
    def test_cancel_removes_it_and_it_never_fires(self):
        timers = self.make(LEAD=5.0)
        timer = timers.add(self.base + 10)
        self.assertTrue(timers.remove(timer['id']))
        self.assertFalse(timers.remove('不存在的 id'))
        timers.tick(self.base + 20)
        self.assertEqual(self.fired, [])
        self.assertEqual(timers.status(self.base + 20)['count'], 0)

    # ---- 持久化 -----------------------------------------------------------
    def test_tasks_survive_a_restart(self):
        timers = self.make()
        at = self.base + 600
        timers.add(at, repeat='daily')
        reloaded = S.Timers(timers.path, self._fire)
        self.assertEqual(len(reloaded.list), 1)
        self.assertAlmostEqual(reloaded.next()['at'], at, places=2)

    # ---- 启动路径：定时器与网页走同一条 -------------------------------------
    def test_starting_without_a_console_says_so_instead_of_pretending(self):
        state = S.State(Path(self.tmp.name) / 'log.jsonl')
        reply = state.start_task(source='timer')
        self.assertFalse(reply['ok'])
        self.assertIn('控制台', reply['error'])

    # ---- 后台线程能启能停 --------------------------------------------------
    def test_the_background_thread_runs_and_can_be_stopped(self):
        timers = self.make(TICK=0.05)
        timers.add(time.time() + 0.2)
        thread = threading.Thread(target=timers.run, daemon=True)
        thread.start()
        deadline = time.time() + 5
        while time.time() < deadline and not self.fired:
            time.sleep(0.02)
        self.assertTrue(self.fired, '后台线程没有自动触发')
        timers.stop()
        thread.join(timeout=5)
        self.assertFalse(thread.is_alive(), 'stop() 之后线程应当退出')


if __name__ == '__main__':
    unittest.main()
