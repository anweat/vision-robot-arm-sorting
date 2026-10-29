// 数据层：只走日志契约 §5 的四个路径，外加 /api/status* 与 /api/cmd。
// 仿真后端（src/sim/sim_server.py）与本服务（src/web/serve.py）提供同名路径，切换不用改代码。

import { EVENTS } from "./contract.js";

// --token 打开后：页面从 ?token= 进来，记住口令，之后所有请求都带上。
// 静态页面本身不校验（不然白屏），API 才校验；EventSource 不能加请求头，走 query。
const TOKEN = new URLSearchParams(location.search).get("token")
  ?? sessionStorage.getItem("armToken") ?? "";
if (TOKEN) sessionStorage.setItem("armToken", TOKEN);
export const authUrl = (url) =>
  TOKEN ? url + (url.includes("?") ? "&" : "?") + "token=" + encodeURIComponent(TOKEN) : url;
const AUTH_HEADERS = TOKEN ? { "X-Token": TOKEN } : {};

const json = async (url, options) => {
  const res = await fetch(authUrl(url), { ...options, headers: { ...AUTH_HEADERS, ...(options?.headers ?? {}) } });
  if (!res.ok) throw new Error(`${res.status} ${url}`);
  return res.json();
};

/** 一次拉取：GET /api/log?after&limit */
export const fetchLog = (after = 0, limit = 500) =>
  json(`/api/log?after=${after}&limit=${limit}`);

/** 逐块记录：GET /api/log/records（六项 + 轮次） */
export const fetchRecords = () => json("/api/log/records");

/** 统计：GET /api/log/summary */
export const fetchSummary = () => json("/api/log/summary");

/** 当前状态：GET /api/status */
export const fetchStatus = () => json("/api/status");

/** W4 定时任务列表；未启用 --timers 时服务端 404，调用方要能接受并提示 */
export const fetchTimers = () => json("/api/timers");

/** E6 未知物体告警列表与存档目录 */
export const fetchArchive = () => json("/api/archive");

/**
 * 桌上物块的基座坐标与禁入区尺寸，给三维视图画方块用。
 * 执行侧没有这一层时返回 404，调用方降级成"不画物块"，不假造位置。
 */
export const fetchBlocks = () => json("/api/blocks");

/** 存档图片。<name> 来自 unknown_object 事件的 image 字段 */
export const archiveUrl = (name) => authUrl(`/api/archive/${encodeURIComponent(name ?? "")}`);

/** 下发指令：POST /api/cmd，需求规格 ④ */
export const sendCmd = async (cmd, extra = {}) => {
  const res = await fetch(authUrl("/api/cmd"), {
    method: "POST",
    headers: { "Content-Type": "application/json", ...AUTH_HEADERS },
    body: JSON.stringify({ cmd, source: "web", ...extra }),
  });
  const body = await res.json().catch(() => ({}));
  if (!res.ok) {
    const err = new Error(body.error || `指令失败 ${res.status}`);
    err.status = res.status;                       // 409 = 冲突/忙，调用方刷新状态而不是重试
    throw err;
  }
  return body;
};

/** 页面关闭/切后台时释放操作权（sendBeacon 在 unload 期间也能发出去） */
export const releaseControl = () => {
  const blob = new Blob([JSON.stringify({ cmd: "stop", source: "web" })],
                        { type: "application/json" });
  navigator.sendBeacon(authUrl("/api/cmd"), blob);
};

/**
 * W4 定时任务：新增与取消。
 * at 传 epoch 秒（页面自己按浏览器时区算好，避免 ISO 字符串在时区上的歧义）。
 */
export const sendTimer = async (body) => {
  const res = await fetch(authUrl("/api/timers"), {
    method: "POST",
    headers: { "Content-Type": "application/json", ...AUTH_HEADERS },
    body: JSON.stringify(body),
  });
  const payload = await res.json().catch(() => ({}));
  if (!res.ok) throw new Error(payload.error || `失败 ${res.status}`);
  return payload;
};

/**
 * 事件流订阅。SSE 优先，断线由 EventSource 自带重连并按 Last-Event-ID 续传；
 * SSE 不可用（或被代理挡住）时退化为按 seq 轮询，行为一致。
 *
 * onEvent(event) 收到的 event 已带 seq；契约外的事件已在这里过滤掉。
 * onState(state) 报告连接状态：connecting / live / polling / error
 */
export class LogFeed {
  constructor({ onEvent, onState } = {}) {
    this.onEvent = onEvent ?? (() => {});
    this.onState = onState ?? (() => {});
    this.seq = 0; this.timer = null; this.source = null; this.stopped = false;
    // 延迟：日志自己的时间轴 vs 页面墙钟的推进之差，契约允许同一份日志内相减
    this._t0 = null; this._wall0 = null;
  }

  /** 延迟（秒）：非负；仅在收到过事件后有值 */
  get latency() {
    if (this._t0 === null || this._wall0 === null) return null;
    return Math.max(0, (performance.now() - this._wall0) / 1000 - (this._lastT - this._t0));
  }

  /**
   * since 为空时只收新事件；传入 0 会把已有日志回放一遍。
   * seed = {t, wall}：延迟估计的基线——最近一条回填事件的 t 加上那一刻的墙钟。
   * 不带基线时第一条实时事件自成基线；历史回填不参与估计，免得回放出"0.00 s"冒充实时。
   */
  async start(since = null, seed = null) {
    this.stopped = false;
    if (seed && typeof seed.t === "number") {
      this._t0 = seed.t; this._wall0 = seed.wall; this._lastT = seed.t;
    }
    this.onState("connecting");
    try {
      const first = await fetchLog(0, 1);
      this.seq = since ?? first.total ?? 0;
      const probe = await fetch(authUrl("/api/log/stream"),
                                { method: "GET", headers: { Accept: "text/event-stream" } });
      probe.body?.cancel();
      if (!probe.ok || !probe.headers.get("content-type")?.includes("event-stream")) throw new Error("no sse");
      this._openSse();
    } catch {
      this._poll();                                // 退化为轮询
    }
  }

  _openSse() {
    this.source = new EventSource(authUrl(`/api/log/stream?after=${this.seq}`));
    this.onState("connecting");
    this.source.onopen = () => this.onState("live");
    this.source.onmessage = (m) => {
      this.onState("live");
      this._accept(JSON.parse(m.data));
    };
    this.source.onerror = () => {
      this.onState("error");                       // EventSource 会自己重连
    };
  }

  _poll() {
    this.onState("polling");
    const tick = async () => {
      if (this.stopped) return;
      try {
        const { events, total } = await fetchLog(this.seq, 500);
        events.forEach((e) => this._accept(e));
        if (total !== undefined) this.total = total;
      } catch { this.onState("error"); }
      this.timer = setTimeout(tick, 800);
    };
    tick();
  }

  _accept(event) {
    if (!event || typeof event.event !== "string") return;
    this.seq = Math.max(this.seq, event.seq ?? 0);
    if (this._t0 === null && typeof event.t === "number") {
      this._t0 = event.t; this._wall0 = performance.now();
    }
    if (typeof event.t === "number") this._lastT = event.t;
    if (!EVENTS[event.event]) return;              // 契约外的事件跳过
    this.onEvent(event);
  }

  stop() {
    this.stopped = true;
    this.source?.close();
    clearTimeout(this.timer);
  }
}

/** 状态订阅：≥ 2 Hz 的模式与六轴角度推送 */
export class StatusFeed {
  constructor({ onStatus, onState } = {}) {
    this.onStatus = onStatus ?? (() => {});
    this.onState = onState ?? (() => {});
    this.source = null; this.timer = null; this.stopped = false;
  }

  start() {
    this.stopped = false;
    try {
      this.source = new EventSource(authUrl("/api/status/stream"));
      this.source.onmessage = (m) => { this.onState("live"); this.onStatus(JSON.parse(m.data)); };
      this.source.onerror = () => this.onState("connecting");
    } catch { this._poll(); }
    this._poll();                                   // SSE 慢一拍时也有数据
  }

  _poll() {
    const tick = async () => {
      if (this.stopped) return;
      try { this.onStatus(await fetchStatus()); } catch { /* 保持上一次 */ }
      this.timer = setTimeout(tick, 500);
    };
    tick();
  }

  stop() { this.stopped = true; this.source?.close(); clearTimeout(this.timer); }
}
