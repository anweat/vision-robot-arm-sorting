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
 * 语音/手势走控制台那一套（/api/interact/*），本服务只是代理。
 * 返回 {status: accepted|ignored|rejected, command, preset, matched, reason, id, t, ...}。
 * 三态与 reason 原样显示，页面不自己判断词表、阈值和连续帧。
 */
export const fetchInteract = () => json("/api/interact");

/** 武装/解除：除“停止”外的指令都要先武装（POST /api/interact/arm） */
export const armInteract = async (armed) => {
  const res = await fetch(authUrl("/api/interact/arm"), {
    method: "POST",
    headers: { "Content-Type": "application/json", ...AUTH_HEADERS },
    body: JSON.stringify({ armed: !!armed }),
  });
  const payload = await res.json().catch(() => ({}));
  if (!res.ok) throw new Error(payload.error || `武装失败 ${res.status}`);
  return payload;
};

/** 一句识别文本（POST /api/interact/voice）；识别与执行都在控制台 */
export const sendVoice = async (text) => {
  const res = await fetch(authUrl("/api/interact/voice"), {
    method: "POST",
    headers: { "Content-Type": "application/json", ...AUTH_HEADERS },
    body: JSON.stringify({ text }),
  });
  const payload = await res.json().catch(() => ({}));
  if (!res.ok) throw new Error(payload.error || `语音识别失败 ${res.status}`);
  return payload;
};

/** 一帧手势分类结果（POST /api/interact/gesture）；服务端自己累计连续帧 */
export const sendGestureFrame = async (label, confidence = 0.9) => {
  const res = await fetch(authUrl("/api/interact/gesture"), {
    method: "POST",
    headers: { "Content-Type": "application/json", ...AUTH_HEADERS },
    body: JSON.stringify({ label, confidence }),
  });
  const payload = await res.json().catch(() => ({}));
  if (!res.ok) throw new Error(payload.error || `手势识别失败 ${res.status}`);
  return payload;
};

/**
 * 连发 frames 帧，返回最后一次结果（真正可能触发的那一次）。
 * 真实识别器是逐帧上报的，这里按同样的帧数发，才走得到控制台那条“连续 3 帧”的门槛。
 */
export const sendGesture = async (label, frames = 3, confidence = 0.9) => {
  let last = null;
  for (let i = 0; i < frames; i += 1) last = await sendGestureFrame(label, confidence);
  return last;
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
    } catch {
      this._poll();                                // 取不到总数就直接轮询
      return;
    }
    this._openSse();
  }

  _openSse() {
    let opened = false;
    this.source = new EventSource(authUrl(`/api/log/stream?after=${this.seq}`));
    this.source.onopen = () => { opened = true; this.onState("live"); };
    this.source.onmessage = (m) => {
      this.onState("live");
      this._accept(JSON.parse(m.data));
    };
    this.source.onerror = () => {
      // 连不上又没打开过：服务端没有 SSE，降级轮询，别让 console 一直报错
      // （之前用 fetch 探测再 cancel，浏览器会记一条 ERR_ABORTED）
      if (!opened) { this.source.close(); this._poll(); return; }
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
    this.source = null;
    clearTimeout(this.timer);
    this.timer = null;
  }

  /**
   * 页面切到后台就断开长连接：每个页面占 2 条 SSE，浏览器同域只给 6 条，
   * 开三个标签页就会占满，新页面连 HTML 都拿不到（一直转圈）。
   * 回来时 resume() 从上次的 seq 续上，不重复收事件。
   */
  pause() {
    this._paused = true;
    this.stop();
  }

  async resume() {
    if (!this._paused) return;
    this._paused = false;
    const seed = this._lastT === null ? null : { t: this._lastT, wall: performance.now() };
    await this.start(this.seq, seed);
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
    this._openSse();
  }

  // SSE 优先，连上就不再轮询（之前 catch 外无条件 _poll() 导致 SSE 与轮询并存、
  // 请求量翻倍；catch 分支还会调两次 _poll，两个循环写同一个 timer，stop() 只能停一个，另一个泄漏）。
  // SSE 构造失败或从未打开时，才降级成一条轮询，写法与 LogFeed 对齐。
  _openSse() {
    let opened = false;
    try {
      this.source = new EventSource(authUrl("/api/status/stream"));
    } catch { this._poll(); return; }
    this.source.onopen = () => { opened = true; this.onState("live"); };
    this.source.onmessage = (m) => { this.onState("live"); this.onStatus(JSON.parse(m.data)); };
    this.source.onerror = () => {
      if (!opened) { this.source.close(); this._poll(); return; }   // 服务端没有 SSE：降级轮询
      this.onState("error");                                         // 打开过又断：EventSource 自己重连
    };
  }

  _poll() {
    const tick = async () => {
      if (this.stopped) return;
      try { this.onStatus(await fetchStatus()); } catch { /* 保持上一次 */ }
      this.timer = setTimeout(tick, 500);
    };
    tick();
  }

  stop() { this.stopped = true; this.source?.close(); this.source = null; clearTimeout(this.timer); }

  pause() { this._paused = true; this.stop(); }

  resume() {
    if (!this._paused) return;
    this._paused = false;
    this.start();
  }
}
