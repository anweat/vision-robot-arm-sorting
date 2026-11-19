package com.example.myapplication00001.relay

import android.content.Context
import java.io.IOException
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.Inet4Address
import java.net.Inet6Address
import java.net.InetAddress
import java.net.InetSocketAddress
import java.net.NetworkInterface
import java.net.ServerSocket
import java.net.Socket
import java.util.concurrent.ExecutorService
import java.util.concurrent.atomic.AtomicInteger
import java.util.concurrent.atomic.AtomicLong
import org.json.JSONObject

/**
 * 跳板（文档 31 §2.1）：手机经 USB 网络共享连着板卡，把板卡上的网关与 SSH 转发给 Tailscale/ZeroTier 上的其他设备。
 * 只做 TCP 字节转发，不解析内容；安全检查全部在板卡上的控制台与网关。
 */
data class PortMap(val name: String, val local: Int, val remote: Int)

data class RelayConfig(
    /** 空 = 等板卡广播（[BEACON_PORT]）自动填；开发时可填 127.0.0.1 配合 adb reverse。 */
    val boardHost: String = "",
    val gatewayLocal: Int = 8090,
    val gatewayRemote: Int = 8090,
    val sshLocal: Int = 2222,
    val sshRemote: Int = 22,
    /** 默认只放行本机与 Tailscale/ZeroTier 来的连接；打开后热点/同一 Wi-Fi 里的设备也能连。 */
    val allowLan: Boolean = false,
) {
    fun maps() = listOf(PortMap("网关", gatewayLocal, gatewayRemote), PortMap("SSH", sshLocal, sshRemote))

    fun save(context: Context) {
        context.getSharedPreferences(PREFS, Context.MODE_PRIVATE).edit()
            .putString("board_host", boardHost.trim())
            .putInt("gateway_local", gatewayLocal).putInt("gateway_remote", gatewayRemote)
            .putInt("ssh_local", sshLocal).putInt("ssh_remote", sshRemote)
            .putBoolean("allow_lan", allowLan)
            .apply()
    }

    companion object {
        private const val PREFS = "relay"
        const val BEACON_PORT = 37020

        fun load(context: Context): RelayConfig {
            val p = context.getSharedPreferences(PREFS, Context.MODE_PRIVATE)
            val d = RelayConfig()
            return RelayConfig(
                boardHost = p.getString("board_host", d.boardHost) ?: "",
                gatewayLocal = p.getInt("gateway_local", d.gatewayLocal),
                gatewayRemote = p.getInt("gateway_remote", d.gatewayRemote),
                sshLocal = p.getInt("ssh_local", d.sshLocal),
                sshRemote = p.getInt("ssh_remote", d.sshRemote),
                allowLan = p.getBoolean("allow_lan", d.allowLan),
            )
        }
    }
}

/** 最近的转发日志，给界面看（不落盘）。 */
object RelayLog {
    private val lines = ArrayDeque<String>()
    private val fmt = java.text.SimpleDateFormat("HH:mm:ss", java.util.Locale.CHINA)

    @Synchronized fun add(level: String, text: String) {
        lines.addLast("${fmt.format(java.util.Date())} $level $text")
        while (lines.size > 200) lines.removeFirst()
        android.util.Log.i("ArmRelay", "$level $text")
    }

    fun i(text: String) = add("·", text)
    fun w(text: String) = add("!", text)

    @Synchronized fun snapshot(n: Int = 40): List<String> = lines.toList().takeLast(n)
}

/** 一个本地端口 → 板卡端口。每条连接两个线程对拷，任一端断开就双向关闭。 */
class Forwarder(
    val map: PortMap,
    private val target: () -> String?,
    private val allow: (InetAddress) -> Boolean,
    private val pool: ExecutorService,
) {
    val active = AtomicInteger()
    val total = AtomicLong()
    @Volatile var lastError = ""
    private var server: ServerSocket? = null

    fun start() {
        val s = ServerSocket()
        s.reuseAddress = true
        s.bind(InetSocketAddress(map.local))
        server = s
        pool.execute {
            while (!s.isClosed) {
                val client = try { s.accept() } catch (_: IOException) { break }
                pool.execute { handle(client) }
            }
        }
    }

    fun stop() {
        runCatching { server?.close() }
    }

    private fun handle(client: Socket) {
        val peer = client.inetAddress
        if (!allow(peer)) {
            RelayLog.w("拒绝 ${peer.hostAddress} → ${map.name}（不在 Tailscale/ZeroTier 网段；可在跳板页放行局域网）")
            closeQuietly(client)
            return
        }
        val host = target()
        if (host.isNullOrBlank()) {
            lastError = "还没找到板卡"
            closeQuietly(client)
            return
        }
        val upstream = Socket()
        try {
            upstream.connect(InetSocketAddress(host, map.remote), 3000)
            upstream.tcpNoDelay = true
            client.tcpNoDelay = true
        } catch (e: IOException) {
            lastError = "连不上 $host:${map.remote}：${e.message}"
            RelayLog.w("${map.name}：$lastError")
            closeQuietly(client)
            closeQuietly(upstream)
            return
        }
        lastError = ""
        active.incrementAndGet()
        total.incrementAndGet()
        val halves = AtomicInteger()
        val finish = {
            if (halves.incrementAndGet() == 2) {
                closeQuietly(client)
                closeQuietly(upstream)
                active.decrementAndGet()
            }
        }
        pool.execute { pipe(client, upstream, finish) }
        pipe(upstream, client, finish)
    }

    private fun pipe(from: Socket, to: Socket, finish: () -> Unit) {
        try {
            val buf = ByteArray(16 * 1024)
            val input = from.getInputStream()
            val output = to.getOutputStream()
            while (true) {
                val n = input.read(buf)
                if (n < 0) break
                output.write(buf, 0, n)
            }
        } catch (_: IOException) {
        } finally {
            runCatching { to.shutdownOutput() }
            finish()
        }
    }

    companion object {
        fun closeQuietly(s: Socket) {
            runCatching { s.close() }
        }
    }
}

/** 板卡在共享链路上每 2 s 广播一条 `{"kind":"arm-board",...}`（scripts/board_beacon.py），收到就记下来源地址。 */
class BeaconListener(private val onBoard: (String, JSONObject) -> Unit) {
    @Volatile private var socket: DatagramSocket? = null

    fun start(pool: ExecutorService) {
        val s = try {
            DatagramSocket(null).apply {
                reuseAddress = true
                broadcast = true
                bind(InetSocketAddress(RelayConfig.BEACON_PORT))
            }
        } catch (e: IOException) {
            RelayLog.w("广播端口 ${RelayConfig.BEACON_PORT} 打不开：${e.message}；请在跳板页手填板卡地址")
            return
        }
        socket = s
        pool.execute {
            val buf = ByteArray(1024)
            while (!s.isClosed) {
                val p = DatagramPacket(buf, buf.size)
                try { s.receive(p) } catch (_: IOException) { break }
                val body = runCatching { JSONObject(String(p.data, 0, p.length, Charsets.UTF_8)) }.getOrNull()
                if (body?.optString("kind") == "arm-board") onBoard(p.address.hostAddress ?: continue, body)
            }
        }
    }

    fun stop() {
        runCatching { socket?.close() }
    }
}

object Net {
    /** Tailscale 的 CGNAT 段 100.64.0.0/10 与 IPv6 段 fd7a:115c:a1e0::/48。 */
    fun isTailnet(a: InetAddress): Boolean {
        val b = a.address
        return when (a) {
            is Inet4Address -> (b[0].toInt() and 0xff) == 100 && (b[1].toInt() and 0xc0) == 64
            is Inet6Address -> b[0] == 0xfd.toByte() && b[1] == 0x7a.toByte() && b[2] == 0x11.toByte() &&
                b[3] == 0x5c.toByte() && b[4] == 0xa1.toByte() && b[5] == 0xe0.toByte()
            else -> false
        }
    }

    /** 与本机某个 tun 网卡同网段（ZeroTier 给的是 /24 之类的段；Tailscale 是 /32，另由 [isTailnet] 判）。 */
    fun onVpnSubnet(a: InetAddress): Boolean = vpnAddresses().any { (addr, prefix) ->
        sameSubnet(addr, a, prefix)
    }

    fun vpnAddresses(): List<Pair<InetAddress, Int>> = try {
        NetworkInterface.getNetworkInterfaces()?.toList().orEmpty()
            .filter { it.isUp && it.name.startsWith("tun") }
            .flatMap { nif -> nif.interfaceAddresses.map { it.address to it.networkPrefixLength.toInt() } }
    } catch (_: Exception) {
        emptyList()
    }

    /** 本机在虚拟网上的 IPv4 地址（显示给用户，别的设备用它连跳板）。 */
    fun vpnIpv4(): List<String> = vpnAddresses().map { it.first }.filterIsInstance<Inet4Address>()
        .mapNotNull { it.hostAddress }

    /** 共享/热点之类的本地网卡地址，排查时显示。 */
    fun localInterfaces(): List<String> = try {
        NetworkInterface.getNetworkInterfaces()?.toList().orEmpty()
            .filter { it.isUp && !it.isLoopback && !it.name.startsWith("tun") && !it.name.startsWith("dummy") }
            .flatMap { nif ->
                nif.interfaceAddresses.filter { it.address is Inet4Address }
                    .map { "${nif.name} ${it.address.hostAddress}/${it.networkPrefixLength}" }
            }
    } catch (_: Exception) {
        emptyList()
    }

    private fun sameSubnet(a: InetAddress, b: InetAddress, prefix: Int): Boolean {
        val x = a.address
        val y = b.address
        if (x.size != y.size || prefix <= 0) return false
        var bits = prefix
        for (i in x.indices) {
            if (bits <= 0) return true
            val mask = if (bits >= 8) 0xff else (0xff shl (8 - bits)) and 0xff
            if ((x[i].toInt() and mask) != (y[i].toInt() and mask)) return false
            bits -= 8
        }
        return true
    }

    /** TCP 建连耗时（ms），不通返回 -1。 */
    fun probe(host: String, port: Int, timeoutMs: Int = 2000): Long {
        val t0 = System.nanoTime()
        return try {
            Socket().use { it.connect(InetSocketAddress(host, port), timeoutMs) }
            (System.nanoTime() - t0) / 1_000_000
        } catch (_: IOException) {
            -1
        }
    }
}
