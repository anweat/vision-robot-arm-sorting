package com.example.myapplication00001

import org.junit.Assert.*
import org.junit.Test

class ScriptProgressTest {
    @Test fun latestStateReplacesTheSamePath() {
        val a = ScriptStepState("a", "steps[1].do[2]", "place", "started")
        val state = ScriptProgress().accept(a).accept(a.copy(state = "done", result = "done"))
        assertEquals(1, state.lines.size)
        assertEquals("done", state.lines[a.path]?.state)
    }
    @Test fun newSessionAndRestartClearPreviousHighlights() {
        val a = ScriptStepState("a", "steps[0]", "scan", "done")
        assertTrue(ScriptProgress().accept(a).accept(a.copy(session = "b", path = "", step = "flow", state = "started")).lines.isEmpty())
        assertTrue(ScriptProgress().accept(a).accept(a.copy(path = "", step = "flow", state = "started")).lines.isEmpty())
    }
}
