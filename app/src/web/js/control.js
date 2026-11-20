// F1-4 模式切换（W1）与 F1-3 控制面板（W2）。
// 指令一律经 POST /api/cmd 下发，字段按需求规格 ④；运动由控制台执行，本页不直接碰硬件。

import { MODES, MODE_NAMES, JOINT_LIMITS, NAMED_POSES, categoryName } from "./contract.js";
import { sendCmd, sendVoice, sendGesture, fetchInteract, armInteract, fetchBlocks } from "./api.js";
import { toast } from "./format.js";
import { createCamera } from "./media.js";

const PRESETS = [["dance", "跳舞"], ["stack", "叠积木"], ["carry", "搬运"]];

async function cmd(payload, okMessage) {
  const started = performance.now();
  try {
    await sendCmd(payload.cmd, payload);
    toast(`${okMessage}（${Math.round(performance.now() - started)} ms）`, "good");
    return true;
  } catch (err) {
    // 409 = 冲突或忙：状态推送会很快刷新，提示等一等，不假装成功
    toast(err.message, "bad");
    return false;
  }
}

export function createControlView(root) {
  root.innerHTML = `
    <section class="panel">
      <h2>速度与夹爪 <small>不用使能：第一条运动指令即取得操作权，本页可见期间自动续期；关页即停</small></h2>
      <div class="row">
        <span class="dim">速度</span>
        <input type="range" id="speedRange" min="3" max="40" step="1" value="20" aria-label="速度">
        <strong id="speedValue">20°/s</strong>
        <span class="chip" id="armChip">—</span>
        <button id="takeoverBtn" hidden>接管</button>
        <span class="sep"></span>
        <button id="stopBtn" class="danger">停止</button>
      </div>
      <div class="row">
        <span class="dim">夹爪</span>
        <button data-grip="grip">合拢</button>
        <button data-grip="open">张开</button>
        <button data-grip="wide">张到最大</button>
        <span class="dim" id="gripState">—</span>
      </div>
      <div class="row">
        <span class="dim">声光</span>
        <button data-led="green">绿</button><button data-led="yellow">黄</button><button data-led="red">红</button>
        <button data-led="blue">蓝</button><button data-led="off">灭</button>
        <button data-beep="120">短鸣</button><button data-beep="400">长鸣</button>
        <span class="dim" id="ledState">—</span>
      </div>
      <p class="hint" id="opHint"></p>
    </section>
    <section class="panel">
      <h2>作业模式 <small>手动 / 预设动作 / 视觉自主</small></h2>
      <div class="row" id="modeRow"></div>
      <p class="hint" id="modeHint">当前模式由服务端持有，切换会写入运行记录。预设动作、视觉自主可以切，但执行侧和视觉侧还没接进来，切过去不会有实际动作。</p>
    </section>
    <section class="panel cam-mini">
      <h2>腕部相机 <small>拖滑块时对照着看；三维视图在作业台</small></h2>
      <img id="ctrlCam" class="cam" alt="腕部相机画面">
      <p class="hint" id="ctrlCamHint">等待画面…</p>
    </section>
    <section class="panel">
      <h2>关节控制 <small>拖动滑块后点“下发”，统一走六舵机批量写入</small></h2>
      <div id="jointBars" class="joints"></div>
      <p class="hint" id="jointHint"></p>
      <div class="row">
        <button id="sendJoints" class="primary">下发角度</button>
        <button id="poseUpright">直立</button>
        <button id="poseObserve">观察位</button>
      </div>
    </section>
    <!-- 桌面物块：拉不到或为空就整块收起，不硬编造位置；实机口径由工作区重构（issue #4）统一 -->
    <section class="panel" id="blocksPanel" hidden>
      <h2>桌面物块 <small>id、类别、基座坐标（cm）；夹爪里的也会列出来</small></h2>
      <table class="table">
        <thead><tr><th>id</th><th>类别</th><th>位置 cm</th><th>状态</th></tr></thead>
        <tbody id="blockRows"></tbody>
      </table>
      <p class="hint" id="blocksHint"></p>
    </section>
    <section class="panel">
      <h2>坐标移动 <small>基座坐标系，厘米</small></h2>
      <div class="row">
        <label>X <input id="tx" type="number" step="0.5" value="0"></label>
        <label>Y <input id="ty" type="number" step="0.5" value="16"></label>
        <label>Z <input id="tz" type="number" step="0.5" value="2"></label>
        <button id="moveTo">移动到</button>
      </div>
      <div class="needs-debug">
        <h3>预设动作（扩展）</h3>
        <div class="row" id="presetRow"></div>
        <p class="hint" id="presetHint">暂不可用：动作本身由执行侧提供，尚未接入；接入后这里会放开。</p>
      </div>
    </section>
    <section class="panel needs-debug">
      <h2>语音与手势（扩展） <small>网关的 /api/interact：武装、连续帧、阈值、冷却与三态结果都由网关的规则判定</small></h2>
      <div class="row">
        <label class="dim"><input id="listenOn" type="checkbox"> 识别端监听（板上腕部相机逐帧手势；要 hrnet.om 与 ais_bench）</label>
        <label class="dim"><input id="listenVoice" type="checkbox"> 同时录音识别（板上音频输入未确认）</label>
        <span class="dim" id="listenState">—</span>
      </div>
      <div class="row">
        <label class="dim"><input id="interactArmed" type="checkbox"> 武装（除“停止”外都要先武装）</label>
        <span class="dim" id="interactState">—</span>
      </div>
      <div class="row">
        <input id="voiceText" type="text" placeholder="输入指令文本，如：启动 / 停止 / 跳个舞 / 合爪" style="flex:1;min-width:0">
        <button id="voiceSend">识别并下发</button>
      </div>
      <p class="hint" id="voiceHint"></p>
      <div class="row" id="gestureRow">
        <span class="dim">手势（连发 3 帧，与识别器逐帧上报等价；脚本里的「等手势」也认这里）</span>
        <button data-gesture="ok">OK→开始</button>
        <button data-gesture="palm">手掌→停止</button>
        <button data-gesture="fist">握拳→合爪</button>
        <button data-gesture="like">点赞→松爪</button>
      </div>
      <p class="hint" id="gestureHint"></p>
      <ul class="hint" id="interactRecent" style="max-height:120px;overflow:auto;padding-left:18px"></ul>
    </section>`;

  const $ = (id) => root.querySelector("#" + id);
  const modeRow = $("modeRow");
  // 预设动作和视觉自主现在点得动、也会真的切换模式并写入运行记录，
  // 但执行侧/视觉侧还没接进来，所以切过去不会有实际动作——点完补一句实话，不让人误会。
  const MODE_NOTE = {
    preset: "预设动作还没接入，这次只切换了模式，不会真的跑动作",
    auto: "视觉侧还没接入，这次只切换了模式，不会开始分拣",
  };
  modeRow.innerHTML = MODES.map((m) =>
    `<button data-mode="${m}" class="mode">${MODE_NAMES[m]}</button>`).join("");
  modeRow.addEventListener("click", (ev) => {
    const btn = ev.target.closest("button[data-mode]");
    if (!btn) return;
    const mode = btn.dataset.mode;
    cmd({ cmd: "set_mode", mode }, `已切到${MODE_NAMES[mode]}`);
    if (MODE_NOTE[mode]) toast(MODE_NOTE[mode]);
  });

  // 预设动作尚未接入执行侧：禁用而不是点了报错，原因写在扩展区 hint 里
  const presetRow = $("presetRow");
  presetRow.innerHTML = PRESETS.map(([p, label]) =>
    `<button data-preset="${p}" disabled title="动作由执行侧提供，尚未接入">${label}</button>`).join("");
  presetRow.addEventListener("click", (ev) => {
    const btn = ev.target.closest("button[data-preset]");
    if (btn && !btn.disabled) cmd({ cmd: "run_preset", preset: btn.dataset.preset }, `已下发“${btn.textContent}”`);
  });

  const bars = $("jointBars");
  bars.innerHTML = JOINT_LIMITS.map((j, i) => `
    <div class="joint" data-i="${i}">
      <span>${j.name}</span>
      <strong>—</strong>
      <input type="range" min="${j.min}" max="${j.max}" step="1" value="${Math.round((j.min + j.max) / 2)}">
      <em class="dim">回读 —</em>
    </div>`).join("");

  // 滑块脏标记：用户动过的轴不再被回读覆盖；没动过的轴跟随回读，
  // 否则只拖一个轴点下发，会把其余轴送回初始中位（滑块初始值不是回读值）
  const dirty = new Set();

  const readJoint = (i) => +bars.querySelector(`.joint[data-i="${i}"] input`).value;
  const setJoint = (i, v) => {
    const input = bars.querySelector(`.joint[data-i="${i}"] input`);
    if (v == null) return;
    input.value = Math.round(v);
    input.closest(".joint").querySelector("strong").textContent = `${Math.round(v)}°`;
    dirty.add(i);
  };

  // 拖动时立刻把目标角度显示出来，不等 0.4 s 的状态轮询——之前只能等机械臂动起来才有数
  bars.addEventListener("input", (ev) => {
    const el = ev.target.closest(".joint");
    if (el) {
      dirty.add(+el.dataset.i);
      el.querySelector("strong").textContent = `${Math.round(+ev.target.value)}°`;
    }
  });

  $("sendJoints").addEventListener("click", async () => {
    // 下发成功就视为"滑块即目标"：清脏标记，让回读追上后重新接管滑块
    if (await cmd({ cmd: "set_joints", joints_deg: JOINT_LIMITS.map((_, i) => readJoint(i)) }, "角度已下发"))
      dirty.clear();
  });
  $("poseUpright").addEventListener("click", () => applyPose(NAMED_POSES.upright));
  $("poseObserve").addEventListener("click", () => applyPose(NAMED_POSES.observe));
  $("stopBtn").addEventListener("click", () => cmd({ cmd: "stop" }, "已停止"));
  $("takeoverBtn").addEventListener("click", () => cmd({ cmd: "takeover" }, "已接管操作权"));
  // 速度：拖动时只改显示，松手才下发；运动中改也行，从下一段起生效（文档 32 §1.3）
  const speedRange = $("speedRange");
  let speedDragging = false;
  speedRange.addEventListener("input", () => {
    speedDragging = true;
    $("speedValue").textContent = `${speedRange.value}°/s`;
  });
  speedRange.addEventListener("change", async () => {
    await cmd({ cmd: "speed", speed: +speedRange.value }, `速度 ${speedRange.value}°/s`);
    speedDragging = false;
  });
  root.querySelectorAll("button[data-grip]").forEach((btn) =>
    btn.addEventListener("click", () =>
      cmd({ cmd: "gripper", action: btn.dataset.grip }, `夹爪“${btn.textContent}”已下发`)));
  $("moveTo").addEventListener("click", () => cmd({
    cmd: "move_to",
    target_cm: [+$("tx").value, +$("ty").value, +$("tz").value],
  }, "移动指令已下发"));

  // 语音/手势（扩展）：整条链在控制台（src/interact/intents.py）。本页只把输入发过去，
  // 把 accepted/ignored/rejected 三态与 reason 原样显示，不在这里判断词表、阈值和连续帧。
  const armedBox = $("interactArmed");
  const interactState = $("interactState");
  const voiceHint = $("voiceHint");
  const voiceText = $("voiceText");
  const gestureHint = $("gestureHint");
  const recentList = $("interactRecent");

  const HEAD = { accepted: "已执行", ignored: "忽略", rejected: "拒绝" };
  const describe = (r) => {
    const head = HEAD[r.status] ?? r.status ?? "—";
    const cmd = r.command ? r.command + (r.preset ? ":" + r.preset : "") : "无指令";
    return `${head} · ${cmd}${r.reason ? " · " + r.reason : ""}`;
  };

  const paintState = (state) => {
    const l = state.listen ?? {};
    $("listenOn").checked = !!l.running;
    $("listenState").textContent = l.running ? "监听中" : l.exit_code ? `已退出（${l.exit_code}）：${(l.output ?? []).at(-1) ?? ""}` : "未开";
    armedBox.checked = !!state.armed;
    interactState.textContent = state.armed ? "已武装" : "未武装（只有“停止”能直接执行）";
    // 用 textContent 逐条建 li：识别文本是外部输入，不进 innerHTML
    recentList.replaceChildren(...(state.recent || []).slice(-8).reverse().map((r) => {
      const li = document.createElement("li");
      li.textContent = `${r.source === "gesture" ? "手势" : "语音"} ${r.input ?? r.label ?? ""} — ${describe(r)}`;
      return li;
    }));
  };

  const refreshInteract = async () => {
    try { paintState(await fetchInteract()); }
    catch { interactState.textContent = "语音/手势不可用（未接控制台）"; }
  };

  armedBox.addEventListener("change", async () => {
    try { paintState(await armInteract(armedBox.checked)); }
    catch (e) { toast(e.message, "bad"); armedBox.checked = !armedBox.checked; }
  });

  $("voiceSend").addEventListener("click", async () => {
    const text = voiceText.value.trim();
    if (!text) { toast("先输入要识别的文本", "bad"); return; }
    try {
      const r = await sendVoice(text);
      voiceHint.textContent = `“${r.input ?? text}” → ${describe(r)}`;
      refreshInteract();
    } catch (e) { toast(e.message, "bad"); }
  });

  $("gestureRow").addEventListener("click", async (ev) => {
    const btn = ev.target.closest("button[data-gesture]");
    if (!btn) return;
    try {
      const r = await sendGesture(btn.dataset.gesture, 3, 0.9);
      gestureHint.textContent = `${btn.textContent} → ${describe(r)}`;
      refreshInteract();
    } catch (e) { toast(e.message, "bad"); }
  });

  refreshInteract();
  setInterval(refreshInteract, 3000);

  // 识别端子进程：开 / 关；状态与最后几行输出来自 /api/interact 的 listen 段
  const listenOn = $("listenOn"), listenVoice = $("listenVoice");
  listenOn.addEventListener("change", async () => {
    try {
      await sendCmd("listen", { on: listenOn.checked, voice: listenVoice.checked });
      toast(listenOn.checked ? "识别端已启动" : "识别端已停止", "good");
    } catch (e) { toast(e.message, "bad"); listenOn.checked = !listenOn.checked; }
  });

  // 声光：经网关转控制台（I2C 由控制台独占）
  root.querySelectorAll("button[data-led]").forEach((b) => b.addEventListener("click", () =>
    cmd({ cmd: "led", colour: b.dataset.led }, `灯：${b.textContent}`)));
  root.querySelectorAll("button[data-beep]").forEach((b) => b.addEventListener("click", () =>
    cmd({ cmd: "beep", ms: +b.dataset.beep }, b.textContent)));

  function applyPose(pose) {
    pose.forEach((v, i) => { if (v != null) setJoint(i, v); });
    toast("已填入预设姿态，点“下发角度”执行");
  }

  // 桌面物块：拉不到或为空就整块收起，不编造位置；实机口径由工作区重构（issue #4）统一。
  // 2 s 一次，只在这一页可见时拉；内容没变就不重绘，免得表格每两秒闪一下
  const blocksPanel = $("blocksPanel");
  const blockRows = $("blockRows");
  const blocksHint = $("blocksHint");
  let blocksHtml = "";
  const pullBlocks = async () => {
    if (root.hidden) return;
    let data = null;
    try { data = await fetchBlocks(); } catch { data = null; }
    const rows = (data && data.blocks) || [];
    if (!rows.length) { blocksPanel.hidden = true; return; }
    blocksPanel.hidden = false;
    const html = rows.map((b) => {
      const pos = (b.centre_cm || []).map((v) => Math.round(v * 10) / 10).join(", ");
      const held = b.held ? "夹持中" : "在桌上";
      return `<tr><td>${b.id ?? "—"}</td><td><span class="tag c-${b.category}">${categoryName(b.category)}</span></td>`
        + `<td>${pos}</td><td>${held}</td></tr>`;
    }).join("");
    if (html !== blocksHtml) { blockRows.innerHTML = html; blocksHtml = html; }
    blocksHint.textContent = `共 ${rows.length} 块${data.block_side_cm ? `，边长 ${data.block_side_cm} cm` : ""}；`
      + "位置来自执行侧的世界接口；当前为仿真场景给出的值。";
  };
  setInterval(pullBlocks, 2000);
  pullBlocks();

  createCamera($("ctrlCam"), $("ctrlCamHint"), () => !root.hidden && !document.hidden);

  return {
    update(store) {
      const mode = store.status.mode ?? "manual";
      modeRow.querySelectorAll("button").forEach((b) =>
        b.classList.toggle("on", b.dataset.mode === mode));

      const arm = store.status.arm ?? {};
      const chip = $("armChip");
      const others = arm.armed && arm.owner && arm.owner !== "web";
      chip.textContent = !arm.attached ? "未接控制台" : others ? `${arm.owner} 持有操作权` : arm.busy ? "运动中" : "空闲";
      chip.className = "chip " + (others ? "warn" : arm.busy ? "on" : "");
      $("takeoverBtn").hidden = !others;
      const sp = arm.speed;
      if (sp && !speedDragging) {
        speedRange.min = sp.min; speedRange.max = sp.max; speedRange.value = sp.value;
        $("speedValue").textContent = `${Math.round(sp.value)}°/s`;
      }
      const led = arm.signal?.led;
      $("ledState").textContent = !arm.attached ? "—" : led ? `当前 ${led.colour ?? led.rgb.join(",")}` : "未设置";
      $("gripState").textContent = !arm.attached ? "—"
        : `${arm.grip_action ?? "—"} · 回读 ${arm.grip_readback ?? "—"}° · ` +
          `${arm.holding ? "夹住" : arm.grip_result === "empty" ? "夹空" : arm.grip_result ?? "—"}`;
      $("opHint").textContent = !arm.attached
        ? "未接控制台，运动类指令会返回 501。启动时加 --console http://127.0.0.1:8770 接仿真，或指向实机控制台。"
        : arm.error ? `控制台错误：${arm.error}`
        : arm.busy ? "正在运动中，等这一步结束；“停止”随时有效。"
        : "直接拖滑块下发、点夹爪或输坐标即可，不需要使能。";

      // 控制台给的是它自己的软限位（core.limits），收到就以它为准：滑块不再给出送不出去的角度
      const limitOf = (i) => {
        const l = arm.limits?.[String(i + 1)];
        return l ? { min: l[0], max: l[1] } : JOINT_LIMITS[i];
      };

      // 姿态只认一个源：接控制台时用 arm.joints_deg（store.pose 已按此取舍）
      const pose = store.pose;
      bars.querySelectorAll(".joint").forEach((el) => {
        const i = +el.dataset.i, lim = limitOf(i);
        const input = el.querySelector("input");
        if (+input.min !== lim.min) input.min = lim.min;
        if (+input.max !== lim.max) input.max = lim.max;
        const back = pose?.[i];
        // 没动过的轴跟随回读（含首次填充：滑块初始值不再是写死的中位）；
        // 动过的轴等回读追上来（差 ≤1°）就清脏，重新交给回读
        if (back != null && !dirty.has(i)) input.value = Math.round(back);
        if (back != null && Math.abs(back - +input.value) <= 1) dirty.delete(i);
        // 限位变了之后滑块值可能落在界外，先夹回来再显示
        const clamped = Math.min(lim.max, Math.max(lim.min, +input.value));
        if (+input.value !== clamped) input.value = clamped;
        // strong = 滑块目标（拖动实时变），em = 执行侧回读（0.4 s 一拍）
        el.querySelector("strong").textContent = `${Math.round(clamped)}°`;
        el.querySelector("em").textContent = `回读 ${back == null ? "—" : Math.round(back)}°`;
        el.querySelector("em").className = back != null && (back < lim.min || back > lim.max) ? "bad" : "dim";
      });

      // 指令不排队：上一段没走完时下发会被拒，先把按钮按住并说明原因
      $("sendJoints").disabled = !!arm.busy;
      $("sendJoints").textContent = arm.busy ? "运动中…" : "下发角度";

      // 限位档与两个容易误会的行为，写明白省得当故障报
      $("jointHint").textContent = !arm.attached
        ? "未接控制台，滑块用默认限位；接上后以控制台的限位为准。"
        : "限位只有一套：滑块范围即舵机允许范围（J2–J4 可到 −10°，J5 90–270°），平台与桌面禁入区只提示不拦。注意实机上 J5 超过 180° 会挡住腕部相机。"
          + " J6 是夹爪不是普通关节：空载完全闭合的读数约 132°，指令再大也会触发阻力保护停在原地（实机相同），不是故障。"
          + ` 运动按每段 ≤${Math.round(arm.speed?.step_deg ?? 8)}° 分段执行（随速度变）、六轴同段同动，段间回读校验、遇阻力立即停；一次只执行一条，整条路径先检查再动。`;
    },

  };
}
