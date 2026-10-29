// F1-5 定时启动（W4）：配置、查看、取消，以及到点前 5 s 的"待发"提示。
//
// 需求规格给的风险是"无人值守时启动有碰撞风险"，兜底是"到点先提示 5 s，可在页面取消，
// 默认关闭"。所以这一页的重点不是排任务的便捷，而是**让人看得见、来得及取消**：
// 待发窗口里横幅常驻并带倒计时，取消按钮同时撤掉任务本身。
//
// 注意 here 的时间与日志里的 t 不是一回事：t 是板卡时钟（比开发机快 8 小时，只能同份日志内
// 相减），而定时任务的 at 是服务端墙钟，可以照常显示成时间。

import { fetchTimers, sendTimer } from "./api.js";
import { toast } from "./format.js";

const REPEAT_NAMES = { once: "一次", daily: "每天" };

const wall = (at) =>
  at == null ? "—" : new Date(at * 1000).toLocaleString("zh-CN", { hour12: false });

export function createTimersView(root) {
  root.innerHTML = `
    <section class="panel">
      <h2>已排任务 <small>可查看、可取消</small></h2>
      <table class="table" id="timerTable">
        <thead><tr><th>到点</th><th>重复</th><th>状态</th><th></th></tr></thead>
        <tbody></tbody>
      </table>
      <p class="hint" id="timerHint">读取中…</p>
    </section>
    <section class="panel">
      <h2>排一个任务 <small>到点前 5 s 会提示，那时还能取消</small></h2>
      <div class="row">
        <label>时刻 <input id="atInput" type="datetime-local"></label>
        <label>重复
          <select id="repeatInput">
            <option value="once">一次</option>
            <option value="daily">每天</option>
          </select>
        </label>
        <button id="addBtn" class="primary">添加</button>
      </div>
      <p class="hint" id="addHint">到点会自动启动一整轮分拣。无人值守启动有碰撞风险，确认现场没人再排。</p>
    </section>`;

  const $ = (id) => root.querySelector("#" + id);
  const tbody = $("timerTable").querySelector("tbody");
  const hint = $("timerHint");
  const atInput = $("atInput");

  // 默认值：一刻钟之后。datetime-local 要的是不含时区的本地串
  const localInput = (d) => {
    const pad = (n) => String(n).padStart(2, "0");
    return `${d.getFullYear()}-${pad(d.getMonth() + 1)}-${pad(d.getDate())}` +
           `T${pad(d.getHours())}:${pad(d.getMinutes())}`;
  };
  atInput.value = localInput(new Date(Date.now() + 15 * 60 * 1000));

  let disabled = false;
  let busy = false;

  async function reload() {
    if (disabled || busy) return;
    try {
      const data = await fetchTimers();
      draw(data);
    } catch (err) {
      disabled = err.message.includes("404");
      hint.textContent = disabled
        ? "定时启动未启用（默认关闭）。启动时加 --timers data/web/timers.json 打开。"
        : `读不到任务表：${err.message}`;
      if (disabled) {
        $("addBtn").disabled = true;
        tbody.innerHTML = "";
      }
    }
  }

  function draw(data) {
    const rows = data.timers ?? [];
    hint.textContent = rows.length
      ? `共 ${rows.length} 个任务；到点前 ${data.lead_s ?? 5} s 会提示，那时还能取消`
      : "还没有任务。定时启动默认关闭，不排任务就永远不会自己启动机械臂。";
    tbody.innerHTML = rows.map((t) => `
      <tr data-id="${t.id}">
        <td>${wall(t.at)}</td>
        <td>${REPEAT_NAMES[t.repeat] ?? t.repeat}</td>
        <td><span class="chip ${t.at <= Date.now() / 1000 ? "warn" : ""}">
          ${t.at <= Date.now() / 1000 ? "待触发" : "等待中"}</span></td>
        <td><button data-cancel="${t.id}" class="danger">取消</button></td>
      </tr>`).join("");
  }

  tbody.addEventListener("click", async (ev) => {
    const btn = ev.target.closest("button[data-cancel]");
    if (!btn || busy) return;
    busy = true;
    try {
      await sendTimer({ cancel: btn.dataset.cancel });
      toast("任务已取消", "good");
      await reload();
    } catch (err) {
      toast(err.message, "bad");
    } finally {
      busy = false;
    }
  });

  $("addBtn").addEventListener("click", async () => {
    if (busy) return;
    const ms = new Date(atInput.value).getTime();
    if (!ms) return toast("先选一个时刻", "bad");
    if (ms <= Date.now()) return toast("时刻必须是将来的", "bad");
    busy = true;
    try {
      await sendTimer({ at: Math.floor(ms / 1000), repeat: $("repeatInput").value });
      toast("已排上", "good");
      await reload();
    } catch (err) {
      toast(err.message, "bad");
    } finally {
      busy = false;
    }
  });

  setInterval(reload, 5000);
  reload();

  // 待发倒计时横幅是全局的（任何页签都可见可取消），在 app.js 里统一渲染
  return {};
}
