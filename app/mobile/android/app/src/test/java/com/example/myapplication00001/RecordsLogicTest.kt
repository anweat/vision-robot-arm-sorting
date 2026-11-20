package com.example.myapplication00001

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/** 记录页的判定：定时任务三态、成功率口径、网关错误提示。 */
class RecordsLogicTest {
    @Test
    fun timerDisabledWhenGatewayHasNoTimers() {
        // 网关不加 --timers 时 /api/status 的 timer 是 {"enabled": false}
        assertEquals(TimerState.Disabled, timerState(enabled = false, pending = true, id = "t1", countdownS = 3.0))
    }

    @Test
    fun timerPendingNeedsId() {
        assertEquals(TimerState.Pending("t1", 4.2), timerState(true, true, "t1", 4.2))
        assertEquals(TimerState.Idle, timerState(true, true, null, 4.2))
        assertEquals(TimerState.Idle, timerState(true, false, "t1", 30.0))
    }

    @Test
    fun timerCountdownNeverNegative() {
        assertEquals(TimerState.Pending("t1", 0.0), timerState(true, true, "t1", -0.4))
        assertEquals(TimerState.Pending("t1", 0.0), timerState(true, true, "t1", null))
    }

    @Test
    fun successRateExcludesCleared() {
        // 32 条样例：成功 ÷（成功 + 夹空 + 跳过 + 失败），挪开障碍不进分母
        assertEquals(75, successRatePercent(sorted = 24, missed = 4, skipped = 2, failed = 2))
        assertNull(successRatePercent(0, 0, 0, 0))
    }

    @Test
    fun errorTextForAuthAndMissingEndpoint() {
        assertTrue(recordsErrorText(401, "token 无效").contains("口令"))
        assertTrue(recordsErrorText(404, "没有这个接口").contains("网关地址"))
        assertEquals("跨源写操作被拒绝", recordsErrorText(403, "跨源写操作被拒绝"))
        assertEquals("暂时无法读取记录", recordsErrorText(500, ""))
    }
}
