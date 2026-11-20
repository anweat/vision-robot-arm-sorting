import {Scene} from './scene.js';
const $=id=>document.getElementById(id), owner=crypto.randomUUID();
let homePlan=null,posePlan=null;
let token='',state=null,scene=null,localBusy=false,preview=false,lastOk=0,frameId=null,points=[],measurePixels=[],frozenImage=null,frameBusy=false;
function notify(text,error=false){$('notice').textContent=text;$('notice').classList.toggle('error',error);}
async function api(path,data){
  const response=await fetch(path,data===undefined?{signal:AbortSignal.timeout(4000)}:{method:'POST',headers:{'Content-Type':'application/json','X-Console-Token':token},body:JSON.stringify({owner,...data}),signal:AbortSignal.timeout(['/api/home/execute','/api/pose/execute'].includes(path)?120000:15000)});
  const result=await response.json();if(!response.ok)throw Error(result.error||response.statusText);return result;
}
function action(fn){return async()=>{try{await fn();}catch(e){notify(e.message,true);}};}
const names=['底座旋转','肩部俯仰','肘部俯仰','腕部俯仰','末端旋腕','夹爪开合'];
function buildJoints(limits){
  names.forEach((name,i)=>{
    const row=document.createElement('div');row.className='joint';
    row.innerHTML=`<div class="joint-row"><div>J${i+1} · ${name}<br><small>${limits[i+1].join(' – ')}° <span id="target${i}"></span></small></div><strong id="q${i}">—</strong><button data-joint="${i+1}" data-delta="-1" aria-label="${name}减小">−</button><button data-joint="${i+1}" data-delta="1" aria-label="${name}增大">+</button></div><label class="preview-slider">预演角度<input id="slider${i}" type="range" min="${limits[i+1][0]}" max="${limits[i+1][1]}" value="${scene.q[i]}" step="1"></label>`;
    $('joints').append(row);
    $(`slider${i}`).oninput=()=>{if(!preview)return;scene.q[i]=Number($(`slider${i}`).value);posePlan=null;$('posePlanText').textContent='目标已改动，请重新预览路径。';renderGeometry();renderState();};
    if(i===5)$(`slider${i}`).disabled=true;  // gripper: use the gripper buttons
  });
  document.querySelectorAll('[data-joint]').forEach(button=>button.onclick=action(async()=>{
    if(preview||localBusy)return;localBusy=true;renderState();
    try{state=await api('/api/jog',{joint:Number(button.dataset.joint),delta:Number(button.dataset.delta)*Number($('step').value)});lastOk=Date.now();notify('单步完成；回读已更新。');}
    finally{localBusy=false;renderState();}
  }));
}
function renderGeometry(){
  const g=scene.draw(),fmt=p=>p.map(v=>(v*100).toFixed(1)).join(' / ');
  $('tip').textContent=fmt(g.tip);$('cameraPosition').textContent=fmt(g.cam);
  scene.q.forEach((q,i)=>$(`q${i}`).textContent=`${q}°`);
  $('sceneLabel').textContent=preview?'姿态模式 · 青色目标 / 灰色当前':state?.mode==='hardware'?(state.age_s==null||state.age_s>2?'回读过期 · 旧姿态，仅供参考':'实机回读 · 几何估计'):'模拟姿态 · 未连接实机';
  $('margin').textContent=preview?'预演未做路径检查':state?.clearance_cm!=null?`${state.clearance_cm.toFixed(1)} cm${state.clearance_cm<0?(state.runtime?.enforce_platform?' · 已进入禁入区':' · 已进入禁入区（只提示）'):''}`:'未知';
  $('margin').classList.toggle('warn',!preview&&state?.clearance_cm!=null&&state.clearance_cm<0);
}
function chip(id,text,kind){$(id).textContent=text;$(id).className='chip'+(kind?' '+kind:'');}
function renderState(){
  if(!state)return;
  const fresh=Date.now()-lastOk<2500;
  chip('mode',state.mode==='hardware'?'实机模式':'离线模拟',state.mode==='hardware'?'warn':'');
  chip('connection',fresh?`回读 ${state.age_s==null?'未知':state.age_s.toFixed(1)+' s 前'}`:'连接中断 · 操作锁定',fresh?'ok':'bad');
  state.pose.forEach((_,i)=>{const c=state.command?.[i];$(`target${i}`).textContent=preview?`· 当前 ${state.pose[i]}°`:c==null?'':`· 指令 ${c}°${c!==state.pose[i]?`（读回 ${state.pose[i]}°）`:''}`;});
  if(state.error)notify(`控制已锁定：${state.error}`,true);
  chip('armState',state.busy?'正在执行一步':others?'另一端持有操作权':state.armed&&fresh?'持有操作权':state.error?'有错误':'空闲',state.busy?'warn':others?'warn':state.armed&&fresh?'ok':state.error?'bad':'');
  const others=state.armed&&state.owner&&state.owner!==owner;
  chip('profileChip',state.speed?`${Math.round(state.speed.value)}°/s · 每段 ≤${Math.round(state.speed.step_deg)}°`:'—','');
  if(state.speed&&document.activeElement!==$('speed')){$('speed').min=state.speed.min;$('speed').max=state.speed.max;$('speed').value=state.speed.value;$('speedVal').textContent=`${Math.round(state.speed.value)}°/s`;}
  $('posePlan').disabled=!preview||localBusy||state.busy||!fresh;
  $('poseGo').disabled=!preview||localBusy||state.busy||!fresh||others||!posePlan;
  document.querySelectorAll('[data-joint]').forEach(b=>b.disabled=preview||localBusy||state.busy||others||!fresh);
  $('sample').disabled=preview||localBusy||state.busy||!fresh;
  $('freeze').disabled=preview||localBusy||state.busy||!fresh||!state.camera.seq;
  $('sampleCount').textContent=`${state.samples??0} 条采样`;
  $('planHome').disabled=preview||localBusy||state.busy||!fresh;
  $('home').disabled=preview||localBusy||state.busy||!fresh||others||!homePlan;
  $('profileInfo').textContent=`一套限位：关节为课程限位（J2–J4 可到 −10°，J5 90–270°）；平台与桌面禁入区${state.runtime?.enforce_platform?'拦截':'只提示不拦截'}。整条路径先检查再动，逐段读回，遇阻停在读回位置。速度只影响段长和节奏。`;
  document.querySelectorAll('[data-verify]').forEach(b=>b.disabled=preview||!fresh);
  const canMove=!preview&&!localBusy&&!state.busy&&!others&&fresh;
  document.querySelectorAll('[data-grip],[data-cart]').forEach(b=>b.disabled=!canMove);
  const g=state.grip,q6=state.pose[5];
  $('gripState').textContent=g?`${{wide:'全开',open:'张开',grip:'夹取'}[g.action]} ${g.command}° → 读回 ${g.readback}°${g.action==='grip'?({holding:' · 已夹住',empty:' · 空夹',not_closed:' · 夹爪没收拢'}[g.result]||(g.holding?' · 已夹住':' · 未夹住')):''}`:(Number.isFinite(q6)?`当前 ${q6}°`:'—');
  $('gripState').className=g&&g.action==='grip'?(g.holding?'holding':'empty'):'';
  $('tipPitch').textContent=state.fk?`${state.fk.tip_pitch_deg[0]}°`:'—';
  const rec=state.recording;
  $('recState').textContent=rec?`录制中 · ${rec.name} · ${rec.steps} 步`:'未录制';$('recState').className=rec?'on':'';
  $('recChip').hidden=!rec;$('recChip').textContent=rec?`● 录制 ${rec.steps} 步`:'';$('recChip').className='chip bad';
  $('recStart').disabled=!!rec;$('recStop').disabled=!rec;
  if(state.keep_out)scene.cfg.keep_out=state.keep_out;
  renderLocalization();
  $('session').textContent=`会话 ${state.session}`;
  if(!preview&&state.pose.every(Number.isFinite)){scene.ghost=null;scene.q=[...state.pose];renderGeometry();}
  else if(preview){scene.ghost=[...state.pose];renderGeometry();}
  $('cameraState').textContent={none:'未接入',replay:'历史静态图',live:'实时源'}[state.camera.kind];
  $('frameAge').textContent=state.camera.error|| (state.camera.kind==='replay'?'历史画面，不对应当前模拟姿态':state.camera.age_s!=null?`${state.camera.clock==='capture'?'采集时间':'接收时间（非采集时间）'}：${state.camera.age_s.toFixed(1)} s 前`:'无画面');
}
async function updateImage(){
  if(frameBusy||!state?.camera.seq||!$('drawerVision').open)return;  // camera panel collapsed: don't pull frames
  frameBusy=true;
  try{const r=await fetch('/api/frame',{signal:AbortSignal.timeout(3000)});if(!r.ok)throw Error('画面不可用');const url=URL.createObjectURL(await r.blob());const img=$('live');const old=img.src;img.onload=()=>URL.revokeObjectURL(url);img.src=url;img.alt=state.camera.kind==='replay'?'历史照片，仅供回放与标定练习':'相机画面，时间状态见下方';img.style.display='block';$('imageEmpty').hidden=true;if(old.startsWith('blob:'))URL.revokeObjectURL(old);}
  catch{ $('frameAge').textContent='画面读取失败；当前显示可能是旧帧'; }
  finally{frameBusy=false;}
}
async function poll(){
  try{state=await api('/api/status');lastOk=Date.now();renderState();await updateImage();}
  catch(e){renderState();notify('连接中断，请检查本地服务。恢复后请重新使能。',true);}
  setTimeout(poll,500);
}
async function stop(){try{state=await api('/api/stop',{});renderState();notify('已禁止后续指令。已发送的一步仍可能完成；此按钮不是硬件急停。');}catch{notify('停止请求未送达。实机操作权将在心跳超时后失效；请现场检查。',true);}}
$('stop').onclick=stop;
document.addEventListener('keydown',e=>{if(e.key==='Escape'){e.preventDefault();stop();}});
document.addEventListener('visibilitychange',()=>{if(document.hidden)stop();});
window.addEventListener('blur',()=>{if(state?.armed)stop();});
setInterval(()=>{if(state?.armed&&state.owner===owner&&!document.hidden&&document.hasFocus()&&Date.now()-lastOk<2500)api('/api/heartbeat',{}).catch(()=>{});},600);
function resetPose(){if(state?.pose.every(Number.isFinite))scene.q=[...state.pose];scene.q.forEach((q,i)=>$(`slider${i}`).value=q);posePlan=null;}
$('preview').onchange=()=>{preview=$('preview').checked;$('joints').classList.toggle('previewing',preview);$('poseTools').hidden=!preview;resetPose();
  // the 3D view is the point of this mode: move it next to the sliders, and back into its drawer after
  const wrap=document.querySelector('.scene-wrap');
  if(preview)$('poseStage').append(wrap);else $('drawerScene').querySelector('.drawer-grid>div').prepend(wrap);
  $('poseStage').hidden=!preview;document.querySelector('.console').classList.toggle('posing',preview);
  renderState();renderGeometry();};
$('poseReset').onclick=()=>{resetPose();$('posePlanText').textContent='已重置为当前姿态。';renderGeometry();renderState();};
$('posePlan').onclick=action(async()=>{posePlan=null;const target=scene.q.slice(0,5).map(v=>Math.round(v));const p=await api('/api/pose/plan',{target});posePlan=p;
  $('posePlanText').textContent=`目标 ${p.target.slice(0,5).join(' / ')}°：${p.segments} 段，约 ${p.seconds} 秒，最大转动 ${p.largest_deg}°${p.margin_cm!=null?`，当前档模型最小余量 ${p.margin_cm} cm`:''}。30 秒内有效。`;renderState();});
$('poseGo').onclick=action(async()=>{if(!posePlan)return;localBusy=true;renderState();
  try{state=await api('/api/pose/execute',{plan_id:posePlan.id});lastOk=Date.now();notify('已执行到目标姿态；读回已更新。');}
  finally{localBusy=false;posePlan=null;$('posePlanText').textContent='执行结束。可继续拖动滑块设置下一个目标。';renderState();}});
$('resetView').onclick=()=>{scene.yaw=-.65;scene.pitch=.55;scene.scale=850;scene.draw();};
$('frustum').onchange=()=>{scene.showFrustum=$('frustum').checked;scene.draw();};
$('keepout').onchange=()=>{scene.showKeepout=$('keepout').checked;scene.draw();};
$('addObject').onclick=action(async()=>{
  const x=Number($('objectX').value)/100,y=Number($('objectY').value)/100,size=Number($('objectSize').value)/100,yaw=Number($('objectYaw').value);
  if(![x,y,size,yaw].every(Number.isFinite)||Math.abs(x)>.5||Math.abs(y)>.5||size<.01||size>.1)throw Error('物件位置限制在 ±50 cm，边长 1–10 cm');
  if(scene.objects.length>=30)throw Error('最多 30 个模拟物件');scene.objects.push({x,y,size,yaw,color:'#d7ad5a'});scene.draw();
});
$('clearObjects').onclick=()=>{scene.objects=[];scene.draw();};
$('sample').onclick=action(async()=>{const s=await api('/api/sample',{kind:$('sampleKind').value,label:$('sampleLabel').value,note:$('note').value});notify(`已保存采样 ${s.id}；坐标是模型估计，画面时间单独记录。`);});
function drawCalibration(){const c=$('calCanvas'),ctx=c.getContext('2d');ctx.clearRect(0,0,c.width,c.height);if(frozenImage)ctx.drawImage(frozenImage,0,0);ctx.font='16px system-ui';points.forEach((p,i)=>{ctx.fillStyle='#ffd081';ctx.beginPath();ctx.arc(p[0],p[1],5,0,Math.PI*2);ctx.fill();ctx.fillText(`${i+1}`,p[0]+8,p[1]-8);});const u=Number($('u').value),v=Number($('v').value);ctx.strokeStyle='#ff9f9f';ctx.beginPath();ctx.moveTo(u-10,v);ctx.lineTo(u+10,v);ctx.moveTo(u,v-10);ctx.lineTo(u,v+10);ctx.stroke();}
function listPoints(){ $('points').replaceChildren(...points.map((p,i)=>{const li=document.createElement('li');li.textContent=`${i+1}: (${p[0]}, ${p[1]}) px → (${p[2]}, ${p[3]}) cm`;return li;}));drawCalibration();}
$('freeze').onclick=action(async()=>{const r=await api('/api/freeze',{});frameId=r.frozen.id;points=[];measurePixels=[];const image=new Image();image.src='/api/frozen?t='+Date.now();await image.decode();frozenImage=image;$('calCanvas').width=image.naturalWidth;$('calCanvas').height=image.naturalHeight;listPoints();$('calResult').textContent='新冻结帧，需重新标定';notify('画面已冻结。用同一平面上的已知点建立映射。');});
$('calCanvas').onclick=e=>{const r=e.currentTarget.getBoundingClientRect();$('u').value=Math.round((e.clientX-r.left)*e.currentTarget.width/r.width);$('v').value=Math.round((e.clientY-r.top)*e.currentTarget.height/r.height);drawCalibration();};
['u','v'].forEach(id=>$(id).oninput=drawCalibration);
function pixel(){if(!frameId)throw Error('先冻结一张画面');const p=[Number($('u').value),Number($('v').value)];if(!p.every(Number.isFinite)||p[0]<0||p[1]<0||p[0]>=$('calCanvas').width||p[1]>=$('calCanvas').height)throw Error('像素必须位于冻结画面内');return p;}
$('addPoint').onclick=action(async()=>{const p=pixel(),x=Number($('x').value),y=Number($('y').value);if(![x,y].every(Number.isFinite))throw Error('请输入平面坐标');points.push([...p,x,y]);listPoints();});
$('clearPoints').onclick=()=>{points=[];measurePixels=[];listPoints();};
$('calibrate').onclick=action(async()=>{const r=await api('/api/calibrate',{frame_id:frameId,plane:$('plane').value,points});$('calResult').textContent=`拟合 RMS ${r.rms_cm.toFixed(3)} cm。${r.warning}`;measurePixels=[];});
$('measure').onclick=action(async()=>{const r=await api('/api/measure',{frame_id:frameId,pixels:[pixel()]});$('measurement').textContent=`平面坐标：${r.xy_cm[0].map(v=>v.toFixed(2)).join(' / ')} cm · ${r.plane}`;});
$('distance').onclick=action(async()=>{measurePixels.push(pixel());if(measurePixels.length===1){$('measurement').textContent='已选第一个点，选择第二个点后再次点击。';return;}const selected=measurePixels;measurePixels=[];const r=await api('/api/measure',{frame_id:frameId,pixels:selected});$('measurement').textContent=`同平面两点距离：${r.distance_cm.toFixed(2)} cm · ${r.plane}`;});
async function start(){try{const b=await api('/api/bootstrap');token=b.token;state=b.status;lastOk=Date.now();scene=new Scene($('scene'),$('cameraSim'),b.scene);buildJoints(state.limits);renderState();$('fov').textContent=`暂定内参：水平视场 ${b.scene.intrinsics.hfov_deg}°。理想投影，未模拟镜头畸变、遮挡或光照。`;notify(state.mode==='hardware'?'实机回读已接入，控制处于锁定状态；现场检查后方可使能。':'离线工作台已就绪。先查看几何，或使能后进行模拟点动；未发送任何实机指令。');poll();}catch(e){notify(e.message,true);}}
start();
// Secondary panels (3D view, cameras, calibration) remember whether they are open; per-browser only.
for(const id of ['drawerScene','drawerVision','drawerCal']){
  const d=$(id);
  try{d.open=localStorage.getItem('teleop.'+id)==='open';}catch{}
  d.addEventListener('toggle',()=>{try{localStorage.setItem('teleop.'+id,d.open?'open':'closed');}catch{}if(id==='drawerScene'&&scene)renderGeometry();if(id==='drawerVision')updateImage();});
}

$('planHome').onclick=action(async()=>{homePlan=null;$('homePlan').textContent='正在检查归位轨迹；若被拒绝，原因显示在顶部。';homePlan=await api('/api/home/plan',{});$('homePlan').textContent=`目标 ${homePlan.target.join(' / ')}°；${homePlan.segments.length} 段，约 ${homePlan.seconds} 秒，模型最小余量 ${homePlan.margin_cm} cm。30 秒内有效。`;renderState();});
$('home').onclick=action(async()=>{if(!homePlan)return;localBusy=true;renderState();try{state=await api('/api/home/execute',{plan_id:homePlan.id});notify('归位结束，检查当前画面与读数。');}finally{localBusy=false;homePlan=null;renderState();}});
document.querySelectorAll('[data-verify]').forEach(b=>b.onclick=action(async()=>{const r=await api('/api/verify',{kind:b.dataset.verify,note:$('note').value});notify(`人工校验已保存：${r.label}。它是观察记录，不代表自动检测结论。`);}));
$('saveLocalization').onclick=action(async()=>{await api('/api/localization/save',{});notify('已保存本次定位与标注图，原始坐标和误差指标一并归档。');});
let imageTimes={};
function renderLocalization(){
  const loc=state.localization;
  if(!loc)return;
  for(const [source,id,img] of [['board','boardObjects','boardDetected'],['phone','phoneObjects','phoneLive']]){
    const r=loc[source];if(!r)continue;
    $(id).textContent=r.error||`地图码 ${r.markers.join(', ')} · ${source==='board'?'重投影 '+r.reprojection_px+' px':'平面拟合 '+r.map_fit_rms_mm+' mm'}\n`+r.objects.map((o,i)=>`${i+1}. ${o.label}: 地图 (${o.map_cm.join(', ')}) cm${o.base_cm?' / 基座 ('+o.base_cm.join(', ')+') cm':''}${o.sticker?` · 官方模型 ${o.sticker.label}（${o.sticker.category}）${o.sticker.conf}`:''}`).join('\n')+(r.objects.length?'':'未检测到完整方块')+((r.rejected||[]).filter(x=>x.draw!==false).map(x=>`\n× ${x.label}: ${x.reason}${x.map_cm?' @ ('+x.map_cm.join(', ')+') cm':''}`).join(''));
    if(!r.error&&imageTimes[source]!==r.t){$(img).src='/api/annotated/'+source+'?t='+r.t;imageTimes[source]=r.t;}
    if(r.error&&source==='phone')$(img).src='/api/phone?t='+Math.floor(Date.now()/1000);
    else if(r.error)$(img).removeAttribute('src');
  }
  const b=loc.board;
  if(state.mode==='hardware'&&!preview&&$('visionScene').checked){
    const valid=b&&!b.error&&!state.busy&&JSON.stringify(b.servos)===JSON.stringify(state.pose);
    scene.objects=valid?b.objects.map(o=>({x:o.base_cm[0]/100,y:o.base_cm[1]/100,size:.03,yaw:0,color:'#d7ad5a',ground:o.base_cm[2]/100-.03})):[];
  }
}

async function step(path,data,done){
  if(preview||localBusy)return;localBusy=true;renderState();
  try{state=await api(path,data);lastOk=Date.now();notify(done(state));}
  finally{localBusy=false;renderState();}
}
function cartesian(axis,sign){
  const amount=sign*Number(axis==='pitch'?$('pitchStep').value:$('cartStep').value);
  return step('/api/cartesian',{axis,amount},s=>`夹爪尖 ${axis==='pitch'?'俯角':axis.toUpperCase()} ${amount>0?'+':''}${amount}${axis==='pitch'?'°':' cm'} 完成；当前俯角 ${s.fk?.tip_pitch_deg?.[0]}°。`);
}
function gripper(action){
  return step('/api/gripper',{action},s=>s.grip?.action==='grip'?({holding:`夹取完成：读回 ${s.grip.readback}°，判定已夹住。`,empty:`夹取完成：读回 ${s.grip.readback}°，已合到指令位置，判定空夹。`,not_closed:`夹爪几乎没动（读回 ${s.grip.readback}°），可能手指被桌面或物块卡住。先把夹爪略抬高再夹，或用 J6 小步收拢。`}[s.grip.result]||`夹取完成：读回 ${s.grip.readback}°。`):`夹爪已到 ${s.grip?.command}°。`);
}
document.querySelectorAll('[data-cart]').forEach(b=>b.onclick=action(()=>cartesian(b.dataset.cart,Number(b.dataset.sign))));
document.querySelectorAll('[data-grip]').forEach(b=>b.onclick=action(()=>gripper(b.dataset.grip)));
$('recStart').onclick=action(async()=>{state=await api('/api/record',{on:true,label:$('recLabel').value,note:$('note').value});renderState();notify('开始录制：之后每一步都会保存两路画面和读数。');});
$('recStop').onclick=action(async()=>{state=await api('/api/record',{on:false,note:$('note').value});renderState();notify('录制已结束并写入 episode.json。');});
const keyMap={w:['y',1],s:['y',-1],a:['x',-1],d:['x',1],r:['z',1],f:['z',-1],t:['pitch',1],g:['pitch',-1]};
document.addEventListener('keydown',e=>{
  if(!$('keys').checked||e.repeat||e.metaKey||e.ctrlKey||e.altKey)return;
  if(['INPUT','TEXTAREA','SELECT'].includes(document.activeElement?.tagName)&&document.activeElement.type!=='checkbox')return;
  const k=e.key.toLowerCase();
  if(keyMap[k]){e.preventDefault();action(()=>cartesian(...keyMap[k]))();}
  else if(k==='o'){e.preventDefault();action(()=>gripper('open'))();}
  else if(k==='c'){e.preventDefault();action(()=>gripper('grip'))();}
});

$('speed').oninput=()=>{$('speedVal').textContent=`${$('speed').value}°/s`;};
$('speed').onchange=action(async()=>{state=await api('/api/speed',{speed:Number($('speed').value)});renderState();notify(`速度 ${state.speed.value}°/s：每段 ≤${Math.round(state.speed.step_deg)}°，${state.speed.segment_ms}+${state.speed.settle_ms} ms。`);});
