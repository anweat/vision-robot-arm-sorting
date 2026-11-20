const rad=d=>d*Math.PI/180;
const add=(a,b)=>a.map((v,i)=>v+b[i]);
const mul=(a,k)=>a.map(v=>v*k);
const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0);
export function geometry(q, hand){
  const yaw=rad(q[0]-90), p=[rad(90-q[1]),rad(180-q[1]-q[2]),rad(270-q[1]-q[2]-q[3])];
  let r=0,z=.1075; const points=[[0,0,0],[0,0,z]];
  [.08285,.08285,.07385,.12].forEach((len,i)=>{const a=p[Math.min(i,2)];r+=len*Math.sin(a);z+=len*Math.cos(a);points.push([-r*Math.sin(yaw),r*Math.cos(yaw),z]);});
  const roll=rad(q[4]-90),x=[1,0,0],y=[0,Math.cos(p[2]),-Math.sin(p[2])],axis=[0,Math.sin(p[2]),Math.cos(p[2])];
  const rz=v=>[Math.cos(yaw)*v[0]-Math.sin(yaw)*v[1],Math.sin(yaw)*v[0]+Math.cos(yaw)*v[1],v[2]];
  const axes=[rz(add(mul(x,Math.cos(roll)),mul(y,Math.sin(roll)))),rz(add(mul(x,-Math.sin(roll)),mul(y,Math.cos(roll)))),rz(axis)];
  // the camera is fixed to the wrist (servo-5 housing): it does not roll with J5
  const mount=[rz(x),rz(y),rz(axis)];
  const rotate=v=>[0,1,2].map(i=>mount.reduce((s,a,j)=>s+a[i]*v[j],0));
  const cam=add(points[4],rotate(hand.map(row=>row[3]).slice(0,3)));
  const camAxes=[0,1,2].map(j=>rotate(hand.slice(0,3).map(row=>row[j])));
  return {points,cam,camAxes,axes,tip:points[5]};
}
export class Scene{
  constructor(canvas,cameraCanvas,config){
    this.canvas=canvas;this.cameraCanvas=cameraCanvas;this.cfg=config;this.yaw=-.65;this.pitch=.55;this.scale=850;
    this.objects=[{x:.01,y:.22,size:.03,yaw:0,color:'#d7ad5a'},{x:-.06,y:.25,size:.03,yaw:25,color:'#76b4dd'}];
    this.q=[90,105,5,5,95,30];this.showFrustum=true;this.showKeepout=true;
    let drag=null;
    canvas.onpointerdown=e=>{drag=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId);};
    canvas.onpointermove=e=>{if(drag){this.yaw+=(e.clientX-drag[0])*.008;this.pitch=Math.max(.05,Math.min(1.5,this.pitch+(e.clientY-drag[1])*.006));drag=[e.clientX,e.clientY];this.draw();}};
    canvas.onpointerup=canvas.onpointercancel=()=>drag=null;
    canvas.onwheel=e=>{e.preventDefault();this.scale=Math.max(350,Math.min(1700,this.scale*Math.exp(-e.deltaY*.001)));this.draw();};
    new ResizeObserver(()=>this.draw()).observe(canvas);
  }
  set(q){this.q=q;this.draw();}
  projection(v){
    const x=v[0],y=v[1]-.16,z=v[2]-.06,c=Math.cos(this.yaw),s=Math.sin(this.yaw),h=c*y+s*x;
    const scale=this.scale*Math.min(1,this.w/850,this.h/410);
    return [this.w/2+(c*x-s*y)*scale,this.h*.52+(-z*Math.cos(this.pitch)+h*Math.sin(this.pitch))*scale,h*Math.cos(this.pitch)+z*Math.sin(this.pitch)];
  }
  shapes(){
    const z=this.cfg.keep_out.table_surface_z_m??this.cfg.keep_out.table_z_m, shapes=[];
    const line=(p,color,width=1)=>shapes.push({p,color,width});
    const poly=(p,fill,color)=>shapes.push({p,fill,color});
    poly([[-.22,-.02,z],[.22,-.02,z],[.22,.42,z],[-.22,.42,z]],'#1d3039','#344d59');
    for(let a=-.2;a<=.201;a+=.025)line([[a,-.02,z],[a,.42,z]],'#304651');
    for(let a=0;a<=.401;a+=.025)line([[-.22,a,z],[.22,a,z]],'#304651');
    // Map is a tentative rigid placement at base y + 8 cm, not a surveyed scene.
    for(const corners of Object.values(this.cfg.map.layout_m))poly(corners.map(p=>[p[0],p[1]+.08,z+.001]),'#788690','#9aa4aa');
    if(this.showKeepout){
      const R=this.cfg.keep_out.platform_radius_m, top=this.cfg.keep_out.platform_min_z_m;
      const circle=h=>Array.from({length:49},(_,i)=>[R*Math.cos(i*Math.PI/24),R*Math.sin(i*Math.PI/24),h]);
      line(circle(top),'#bd6e76',2);line(circle(0),'#784b59');
      for(let i=0;i<8;i++){const a=i*Math.PI/4;line([[R*Math.cos(a),R*Math.sin(a),0],[R*Math.cos(a),R*Math.sin(a),top]],'#784b59');}
    }
    this.objects.forEach(o=>{
      const d=o.size/2,c=Math.cos(rad(o.yaw)),s=Math.sin(rad(o.yaw));
      const ring=h=>[[-d,-d],[d,-d],[d,d],[-d,d]].map(([x,y])=>[o.x+x*c-y*s,o.y+x*s+y*c,h]);
      const ground=o.ground??z;const b=ring(ground),t=ring(ground+o.size);poly(t,o.color,'#dfded0');
      for(let i=0;i<4;i++)poly([b[i],b[(i+1)%4],t[(i+1)%4],t[i]],o.color+'bb','#d7d5c388');
    });
    return {shapes,line,poly};
  }
  draw(){
    const ratio=window.devicePixelRatio||1;this.w=this.canvas.clientWidth;this.h=this.canvas.clientHeight;
    const g=geometry(this.q,this.cfg.hand_eye.cam2gripper);this.g=g;
    if(!this.w)return g;  // view collapsed: still report the geometry for the readouts
    this.canvas.width=this.w*ratio;this.canvas.height=this.h*ratio;
    const ctx=this.canvas.getContext('2d');ctx.scale(ratio,ratio);ctx.fillStyle='#101d27';ctx.fillRect(0,0,this.w,this.h);
    const {shapes,line}=this.shapes();
    if(this.ghost)line(geometry(this.ghost,this.cfg.hand_eye.cam2gripper).points,'#6b7a84',5);  // actual pose while a target is being posed
    line(g.points,'#55c8bd',7);
    // Stylized fingers: servo angle -> opening is illustrative, not a calibrated width.
    const opening=.005+.035*(1-Math.min(175,Math.max(5,this.q[5]))/175);
    [-1,1].forEach(side=>line([add(g.tip,mul(g.axes[0],side*opening)),add(add(g.tip,mul(g.axes[0],side*opening)),mul(g.axes[2],-.025))],'#b4dad5',4));
    if(this.showFrustum){
      const K=this.cfg.intrinsics.camera_matrix,[w,h]=this.cfg.intrinsics.image_size,d=.19;
      const rays=[[0,0],[w,0],[w,h],[0,h]].map(([u,v])=>add(g.cam,add(add(mul(g.camAxes[0],(u-K[0][2])/K[0][0]*d),mul(g.camAxes[1],(v-K[1][2])/K[1][1]*d)),mul(g.camAxes[2],d))));
      rays.forEach(p=>line([g.cam,p],'#ddbe6890'));line([...rays,rays[0]],'#ddbe68');line([g.cam,add(g.cam,mul(g.camAxes[2],d))],'#f3d581',2);
    }
    shapes.sort((a,b)=>a.p.reduce((s,v)=>s+this.projection(v)[2],0)/a.p.length-b.p.reduce((s,v)=>s+this.projection(v)[2],0)/b.p.length);
    shapes.forEach(s=>{ctx.beginPath();s.p.forEach((v,i)=>{const p=this.projection(v);i?ctx.lineTo(...p.slice(0,2)):ctx.moveTo(...p.slice(0,2));});if(s.fill){ctx.closePath();ctx.fillStyle=s.fill;ctx.fill();}ctx.strokeStyle=s.color;ctx.lineWidth=s.width||1;ctx.stroke();});
    g.points.forEach((v,i)=>{const p=this.projection(v);ctx.fillStyle='#d9ede9';ctx.beginPath();ctx.arc(p[0],p[1],5,0,Math.PI*2);ctx.fill();ctx.font='11px system-ui';ctx.fillText(['基座','肩 J2','肘 J3','腕 J4','旋腕 J5','夹爪 J6'][i],p[0]+9,p[1]-8);});
    const cp=this.projection(g.cam);ctx.fillStyle='#ffd081';ctx.fillRect(cp[0]-5,cp[1]-5,10,10);ctx.fillText('相机',cp[0]+9,cp[1]+14);
    [[.08,0,0,'#e197a0','X'],[0,.08,0,'#86c6ae','Y'],[0,0,.08,'#8fb5e7','Z']].forEach(v=>{const a=this.projection([0,0,0]),b=this.projection(v.slice(0,3));ctx.beginPath();ctx.moveTo(a[0],a[1]);ctx.lineTo(b[0],b[1]);ctx.strokeStyle=v[3];ctx.lineWidth=2;ctx.stroke();ctx.fillStyle=v[3];ctx.fillText(v[4],b[0],b[1]);});
    this.drawCamera(g);return g;
  }
  drawCamera(g){
    const canvas=this.cameraCanvas,ctx=canvas.getContext('2d'),K=this.cfg.intrinsics.camera_matrix;
    ctx.fillStyle='#0b1218';ctx.fillRect(0,0,640,480);
    const camera=v=>{const rel=add(v,mul(g.cam,-1));return g.camAxes.map(a=>dot(a,rel));};
    const project=v=>[K[0][0]*v[0]/v[2]+K[0][2],K[1][1]*v[1]/v[2]+K[1][2]];
    const shapes=this.shapes().shapes.filter(s=>s.p.every(p=>camera(p)[2]>.005));
    shapes.sort((a,b)=>b.p.reduce((s,p)=>s+camera(p)[2],0)/b.p.length-a.p.reduce((s,p)=>s+camera(p)[2],0)/a.p.length);
    shapes.forEach(s=>{ctx.beginPath();s.p.forEach((p,i)=>{const q=project(camera(p));i?ctx.lineTo(...q):ctx.moveTo(...q);});if(s.fill){ctx.closePath();ctx.fillStyle=s.fill;ctx.fill();}ctx.strokeStyle=s.color;ctx.lineWidth=1;ctx.stroke();});
    ctx.strokeStyle='#ffd081';ctx.beginPath();ctx.moveTo(308,240);ctx.lineTo(332,240);ctx.moveTo(320,228);ctx.lineTo(320,252);ctx.stroke();
    ctx.fillStyle='#b7c6cc';ctx.font='14px system-ui';ctx.fillText('SIMULATION · 640 × 480',16,26);
  }
}
