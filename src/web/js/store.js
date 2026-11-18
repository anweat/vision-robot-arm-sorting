// 页面共享的派生状态：从事件流推出当前轮次、计数、模式与角度。
// 轮次口径：一次 start（网页/定时/按钮，即 interaction command=start）= 一个任务 = 一轮；
// pipeline_start/end 只是这一轮内的明细。没有任务的 pipeline_* 属于日志历史，只读不计入。

import { fetchRecords, fetchSummary } from "./api.js";
import { EVENTS } from "./contract.js";

const EMPTY_COUNTS = () => ({ sorted: 0, missed: 0, skipped: 0, failed: 0, cleared: 0 });

export class Store {
  constructor() {
    this.events = [];            // 契约内事件，按 seq 升序
    this.total = 0;              // 日志总行数（/api/log 的 total / 见过的最大 seq）
    this.records = [];           // 逐块记录（六项）
    this.summary = null;
    this.status = { mode: "manual", joints_deg: null, t: null };
    this.rounds = [];            // 已结束的轮（只含任务轮，不含历史回放）
    this.current = null;         // 进行中的轮（可能一直没有结束，见契约 §7）
    this.historyRounds = 0;      // 无任务的 pipeline_end 计数：日志历史，只读
    this.connection = "connecting";
    this.latency = null;
    this._roundNo = 0;
    this._lastSeq = 0;
    this._listeners = new Set();
  }

  subscribe(fn) { this._listeners.add(fn); return () => this._listeners.delete(fn); }
  notify() { for (const fn of this._listeners) fn(this); }

  push(event) {
    const seq = event?.seq;
    if (seq != null) {
      // 首屏回填与实时流可能交叠，断线续传也会重发：一律按 seq 去重，条数才对得上 total
      if (seq <= this._lastSeq) return;
      this._lastSeq = seq;
      this.total = Math.max(this.total, seq);
    }
    this.events.push(event);
    this._track(event);
    this.notify();
  }

  _track(e) {
    const name = e.event;
    if (name === "interaction" && e.command === "start") {
      // 新的一轮开始；上一轮没等到 pipeline_end 的不算完成（中断的轮不能计入已完成）
      this._roundNo += 1;
      this.current = { no: this._roundNo, t: e.t, run: null, cubes: 0,
                       seconds: null, ended: false, ...EMPTY_COUNTS() };
    } else if (name === "pipeline_start") {
      if (this.current) {
        this.current.run = e.run;
        this.current.cubes = e.cubes?.length ?? 0;
      }
    } else if (name === "pipeline_end") {
      if (this.current) {
        Object.assign(this.current, { seconds: e.seconds, sorted: e.sorted, missed: e.missed,
                                      skipped: e.skipped, cleared: e.cleared,
                                      ended: true, endedAt: e.t });
        this.rounds = [...this.rounds, this.current];
        this.current = null;
      } else {
        this.historyRounds += 1;                 // 历史与回放：只读，不算"完成了 N 轮"
      }
    } else if (name === "stopped") {
      if (this.current && !this.current.ended) this.current.unfinished = true;
    } else if (name === "mode_changed") {
      this.status = { ...this.status, mode: e.mode };
    }
    const result = EVENTS[name]?.record;
    if (result && this.current) this.current[result] += 1;
  }

  setStatus(status) {
    this.status = { ...this.status, ...status };
    this.notify();
  }

  /** 记录与统计不随每次事件刷新，按需拉（1 000 条以内很快） */
  async refresh() {
    try {
      const [records, summary] = await Promise.all([fetchRecords(), fetchSummary()]);
      this.records = records.records ?? [];
      this.summary = summary;
    } catch { /* 保持上一次 */ }
    this.notify();
  }

  /** 最近一个带 pose 的事件，用来显示角度与“在做什么” */
  get lastPose() {
    for (let i = this.events.length - 1; i >= 0; i--) {
      const pose = this.events[i].pose;
      if (Array.isArray(pose) && pose.length === 6) return pose;
    }
    return null;
  }

  /** 六轴姿态只认一个源：有控制台时只读控制台回读，否则才用日志事件里的 pose */
  get pose() {
    const arm = this.status.arm;
    if (arm?.attached) return arm.joints_deg ?? null;
    return this.status.joints_deg ?? this.lastPose;
  }

  get lastEvent() { return this.events[this.events.length - 1] ?? null; }

  /** 已处理块数（本轮） */
  get currentHandled() {
    if (!this.current) return 0;
    const c = this.current;
    return c.sorted + c.missed + c.skipped + c.failed + c.cleared;
  }
}
