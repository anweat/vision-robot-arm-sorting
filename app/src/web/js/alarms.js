// 告警页：上面是待确认的报警与告警（和作业台「提示」栏同一份，确认写日志、各端同步），
// 下面是异常事件明细（按类型筛选）与未知物体存档图。

import { renderNotices, bindAcks, ack } from "./notices.js";
import { createFaultsView } from "./faults.js";
import { createUnknownView } from "./unknown.js";

export function createAlarmsView(root) {
  root.innerHTML = `
    <section class="panel">
      <div class="row spread"><h2 style="margin:0">待确认 <small>报警（流程脚本发的，要确认）· 告警（抓空、夹爪、放置、堵转、桶满、未知物体…）· 状态</small></h2>
        <span><button id="aAckNotices">告警知道了</button> <button id="aAckAll">全部知道了</button></span></div>
      <ul class="notices wide" id="aNotices"></ul>
      <p class="hint">“知道了”写进日志（alarm_ack），网页、手机等各端同时消失。有未确认的报警时，语音、手势、定时都不能启动分拣（页面按钮仍可）。</p>
    </section>
    <div id="aFaults"></div>
    <div id="aUnknown"></div>`;
  const $ = (id) => root.querySelector("#" + id);
  bindAcks(root);
  $("aAckNotices").addEventListener("click", () => ack("notices"));
  $("aAckAll").addEventListener("click", () => ack("*"));
  const faults = createFaultsView($("aFaults"));
  const unknown = createUnknownView($("aUnknown"));
  return {
    update(store) {
      renderNotices($("aNotices"), store, null, { limit: 30 });
      faults.update(store);
      unknown.update?.(store);
    },
    show() { unknown.show?.(); },
  };
}
