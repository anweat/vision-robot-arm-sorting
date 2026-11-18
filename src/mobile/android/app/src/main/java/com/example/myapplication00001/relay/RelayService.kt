package com.example.myapplication00001.relay

import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.PendingIntent
import android.app.Service
import android.content.Context
import android.content.Intent
import android.content.pm.ServiceInfo
import android.os.Build
import android.os.IBinder
import android.os.PowerManager
import com.example.myapplication00001.MainActivity
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors
import java.util.concurrent.ScheduledExecutorService
import java.util.concurrent.TimeUnit

/** 跳板前台服务：常驻转发，每 3 s 巡检板卡是否可达，状态写在常驻通知里。 */
class RelayService : Service() {
    data class Snapshot(
        val board: String?,
        val boardSource: String,
        val boardMs: Long,
        val beaconAt: Long,
        val vpn: List<String>,
        val local: List<String>,
        val ports: List<String>,
        val active: Int,
        val errors: List<String>,
    )

    companion object {
        private const val ACTION_STOP = "com.example.myapplication00001.relay.STOP"
        private const val CHANNEL = "relay"
        private const val NOTIFICATION_ID = 7

        @Volatile var instance: RelayService? = null
            private set

        fun start(context: Context) {
            val intent = Intent(context, RelayService::class.java)
            if (Build.VERSION.SDK_INT >= 26) context.startForegroundService(intent) else context.startService(intent)
        }

        fun stop(context: Context) {
            context.startService(Intent(context, RelayService::class.java).setAction(ACTION_STOP))
        }
    }

    private lateinit var config: RelayConfig
    private val forwarders = mutableListOf<Forwarder>()
    private var pool: ExecutorService? = null
    private var tick: ScheduledExecutorService? = null
    private var beacon: BeaconListener? = null
    private var wake: PowerManager.WakeLock? = null

    @Volatile private var beaconHost: String? = null
    @Volatile private var beaconAt = 0L
    @Volatile private var boardMs = -2L
    @Volatile private var lastUp: Boolean? = null

    fun board(): String? = config.boardHost.ifBlank { null } ?: beaconHost

    fun snapshot() = Snapshot(
        board = board(),
        boardSource = if (config.boardHost.isNotBlank()) "手填" else if (beaconHost != null) "广播" else "—",
        boardMs = boardMs,
        beaconAt = beaconAt,
        vpn = Net.vpnIpv4(),
        local = Net.localInterfaces(),
        ports = forwarders.map { "${it.map.name} :${it.map.local} → :${it.map.remote}（累计 ${it.total.get()}）" },
        active = forwarders.sumOf { it.active.get() },
        errors = forwarders.mapNotNull { f -> f.lastError.takeIf { it.isNotBlank() }?.let { "${f.map.name}：$it" } },
    )

    override fun onBind(intent: Intent?): IBinder? = null

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        if (intent?.action == ACTION_STOP) {
            stopSelf()
            return START_NOT_STICKY
        }
        showNotification("启动中…")
        if (instance == null) startRelay()
        return START_STICKY
    }

    private fun startRelay() {
        config = RelayConfig.load(this)
        val executor = Executors.newCachedThreadPool()
        pool = executor
        for (map in config.maps()) {
            val f = Forwarder(map, ::board, ::allow, executor)
            try {
                f.start()
                forwarders.add(f)
            } catch (e: Exception) {
                RelayLog.w("端口 ${map.local}（${map.name}）监听失败：${e.message}")
            }
        }
        beacon = BeaconListener { host, body ->
            // 板卡只会出现在 USB 共享链路上；虚拟网上的任何设备都能发这种包，不能让它改走转发目标
            val addr = runCatching { java.net.InetAddress.getByName(host) }.getOrNull() ?: return@BeaconListener
            if (Net.isTailnet(addr) || Net.onVpnSubnet(addr)) {
                RelayLog.w("忽略来自虚拟网的板卡广播：$host")
                return@BeaconListener
            }
            if (host != beaconHost) RelayLog.i("收到板卡广播：$host（${body.optString("host")}）")
            beaconHost = host
            beaconAt = System.currentTimeMillis()
        }.also { it.start(executor) }
        val pm = getSystemService(POWER_SERVICE) as PowerManager
        wake = pm.newWakeLock(PowerManager.PARTIAL_WAKE_LOCK, "arm-relay:forward").also { it.acquire() }
        instance = this
        tick = Executors.newSingleThreadScheduledExecutor().also {
            it.scheduleWithFixedDelay(::check, 0, 3, TimeUnit.SECONDS)
        }
        RelayLog.i("跳板已启动：${forwarders.size} 个端口，" + if (config.allowLan) "放行局域网" else "只放行 Tailscale/ZeroTier")
    }

    private fun allow(peer: java.net.InetAddress): Boolean =
        peer.isLoopbackAddress || Net.isTailnet(peer) || Net.onVpnSubnet(peer) ||
            (config.allowLan && (peer.isSiteLocalAddress || peer.isLinkLocalAddress))

    private fun check() {
        try {
            val host = board()
            boardMs = if (host == null) -2 else Net.probe(host, config.gatewayRemote)
            val up = boardMs >= 0
            if (up != lastUp) RelayLog.add(if (up) "·" else "!", if (up) "板卡网关可达（$host）" else "板卡网关不可达（${host ?: "未找到"}）")
            lastUp = up
            val vpn = Net.vpnIpv4().firstOrNull()
            val text = when {
                vpn == null -> "没有 Tailscale/ZeroTier 地址 — 先打开 VPN"
                host == null -> "$vpn · 等板卡广播（或在跳板页手填地址）"
                !up -> "$vpn · 板卡 $host 网关不可达"
                else -> "网关 $vpn:${config.gatewayLocal} · ssh -p ${config.sshLocal} root@$vpn · 活动 ${forwarders.sumOf { it.active.get() }}"
            }
            showNotification(text)
        } catch (e: Exception) {
            RelayLog.w("巡检异常：$e")
        }
    }

    private fun showNotification(text: String) {
        val nm = getSystemService(NOTIFICATION_SERVICE) as NotificationManager
        if (Build.VERSION.SDK_INT >= 26 && nm.getNotificationChannel(CHANNEL) == null) {
            nm.createNotificationChannel(NotificationChannel(CHANNEL, "跳板状态", NotificationManager.IMPORTANCE_LOW))
        }
        val flags = PendingIntent.FLAG_IMMUTABLE or PendingIntent.FLAG_UPDATE_CURRENT
        val open = PendingIntent.getActivity(this, 0, Intent(this, MainActivity::class.java), flags)
        val stop = PendingIntent.getService(this, 1, Intent(this, RelayService::class.java).setAction(ACTION_STOP), flags)
        @Suppress("DEPRECATION")
        val builder = if (Build.VERSION.SDK_INT >= 26) Notification.Builder(this, CHANNEL) else Notification.Builder(this)
        val notification = builder
            .setSmallIcon(android.R.drawable.stat_notify_sync)
            .setContentTitle("机械臂跳板 · 转发中")
            .setContentText(text)
            .setStyle(Notification.BigTextStyle().bigText(text))
            .setOngoing(true)
            .setContentIntent(open)
            .addAction(Notification.Action.Builder(null, "停止跳板", stop).build())
            .build()
        if (Build.VERSION.SDK_INT >= 34) {
            startForeground(NOTIFICATION_ID, notification, ServiceInfo.FOREGROUND_SERVICE_TYPE_SPECIAL_USE)
        } else {
            startForeground(NOTIFICATION_ID, notification)
        }
    }

    override fun onDestroy() {
        instance = null
        tick?.shutdownNow()
        beacon?.stop()
        forwarders.forEach { it.stop() }
        forwarders.clear()
        pool?.shutdownNow()
        wake?.takeIf { it.isHeld }?.release()
        RelayLog.i("跳板已停止")
        super.onDestroy()
    }
}
