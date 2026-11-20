package com.example.myapplication00001

import com.example.myapplication00001.relay.GatewayPort
import com.example.myapplication00001.relay.PortMap
import com.example.myapplication00001.relay.RelayConfig
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Test

/** 跳板转发到板卡的网关端口：留空跟随板卡广播，手填固定（文档 36 §4）。 */
class GatewayPortTest {
    @Test
    fun manualWins() {
        assertEquals(8090 to "手填", GatewayPort.choose(8090, 8081, "10.0.0.5", "10.0.0.5"))
    }

    @Test
    fun autoFollowsBeaconOfCurrentBoard() {
        assertEquals(8081 to "广播", GatewayPort.choose(null, 8081, "10.0.0.5", "10.0.0.5"))
    }

    @Test
    fun beaconOfAnotherHostIsIgnored() {
        // 手填了 127.0.0.1（adb reverse），广播来自共享链路上的板卡：不能拿那台的端口
        assertEquals(8090 to "默认", GatewayPort.choose(null, 8081, "10.0.0.5", "127.0.0.1"))
    }

    @Test
    fun noBeaconUsesDefault() {
        assertEquals(RelayConfig.DEFAULT_GATEWAY to "默认", GatewayPort.choose(null, null, null, null))
        assertEquals(8090 to "默认", GatewayPort.choose(null, null, "10.0.0.5", "10.0.0.5"))
    }

    @Test
    fun validPort() {
        assertEquals(8081, RelayConfig.validPort(8081))
        assertNull(RelayConfig.validPort(0))
        assertNull(RelayConfig.validPort(70000))
        assertNull(RelayConfig.validPort(null))
    }

    @Test
    fun portMapReadsRemoteAtConnectTime() {
        var port = 8090
        val map = RelayConfig().maps { port }.first()
        assertEquals(8090, map.remote)
        port = 8081
        assertEquals(8081, map.remote)
        assertEquals(22, RelayConfig().maps { port }[1].remote)
        assertEquals(22, PortMap("SSH", 2222, 22).remote)
    }
}
