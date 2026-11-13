// F1-6 未知物体告警页（E6）：把不属于四类的物体连同存档图片列出来。
//
// 判定与图片都由 H1-6 的视觉侧给出：它往 --archive 目录存图，并写 unknown_object 事件
// （必填 cube 与 image）。这里只做呈现，不自己识别任何东西。所以在 H1-6 接进来之前，
// 这一页应当老老实实显示"还没有"，而不是拿别的事件凑数。

import { fetchArchive, archiveUrl } from "./api.js";
import { fmtXY, fmtRel } from "./format.js";

export function createUnknownView(root) {
  root.innerHTML = `
    <section class="panel">
      <h2>未知物体 <small>发现不属于四类的物体时不抓取，只记录并告警</small></h2>
      <div class="cards">
        <div class="card"><span>累计告警</span><b id="uCount">—</b></div>
        <div class="card"><span>存档目录</span><b id="uDir">—</b></div>
        <div class="card"><span>最近一次</span><b id="uLast">—</b></div>
      </div>
      <p class="hint" id="uHint">读取中…</p>
    </section>
    <section class="panel">
      <h2>存档图片 <small>点开看大图；图片名来自事件里的 image 字段</small></h2>
      <div class="gallery" id="uGallery"></div>
    </section>`;

  const $ = (id) => root.querySelector("#" + id);
  const gallery = $("uGallery");
  let shown = 0;

  // 目录信息只取一次：它是启动参数，不会变
  fetchArchive().then((a) => {
    $("uDir").textContent = a.archive ? String(a.archive).split(/[\\/]/).slice(-2).join("/") : "未启用";
  }).catch(() => { $("uDir").textContent = "未启用"; });

  function draw(store) {
    const events = store.events.filter((e) => e.event === "unknown_object").slice().reverse();
    $("uCount").textContent = events.length;
    const base = store.events[0]?.t ?? null;
    $("uLast").textContent = events.length ? fmtRel(events[0].t, base) : "—";

    if (!events.length) {
      $("uHint").textContent = "还没有未知物体告警。判定与图片由视觉侧提供：它存图并写 "
        + "unknown_object 事件后，这里会自动列出。";
      gallery.innerHTML = "";
      shown = 0;
      return;
    }
    $("uHint").textContent = `不抓取、只记录。共 ${events.length} 条，按发生时间倒序。`;
    if (shown === events.length) return;          // 没新增就别重画，免得图片闪
    shown = events.length;

    gallery.innerHTML = events.map((e) => `
      <figure class="shot">
        <a href="${archiveUrl(e.image)}" target="_blank" rel="noopener">
          <img src="${archiveUrl(e.image)}" alt="未知物体 ${e.image ?? ""}" loading="lazy">
        </a>
        <figcaption>
          <b>位置 ${fmtXY(e.cube)}</b>
          <span class="dim">${fmtRel(e.t, base)} · seq ${e.seq ?? "—"}</span>
          ${e.votes ? `<span class="dim">判定票型 ${JSON.stringify(e.votes)}</span>` : ""}
          <span class="dim">${e.image ?? "没有图片"}</span>
        </figcaption>
      </figure>`).join("");
  }

  return { update: draw };
}
