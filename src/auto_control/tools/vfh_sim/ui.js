(function () {
  'use strict';
  const root=document.getElementById('vfh-course-sim'), M=window.VFHSim;
  const el=s=>root.querySelector(s), all=s=>[...root.querySelectorAll(s)];
  const saved=window.openai?.widgetState;
  const savedModel=saved?.modelContent||saved;
  const initial=window.VFHSimInitialScene;
  const baseline=window.VFHSimBenchmark;
  const planningProfiles=window.VFHSimProfiles?.profiles||[];
  let jetsonRatio=Number(saved?.privateContent?.jetsonRatio)||baseline?.jetson?.timeRatio||4;
  let lastPerfAt=-Infinity;const renderTimes=[];
  let config={...M.defaults,...initial?.config}, objects=initial?.obstacles?JSON.parse(JSON.stringify(initial.obstacles)):M.presets('guidance'), selected=null, sim, running=false, drag=null, lastFrame=0, accumulator=0;
  let frameRequest=null;
  let worldMap={scale:1,left:38,top:20,height:240};
  const fields=M.native?[
    ['courseLength','코스 길이(m)',4,32,1],['speed','목표 속도(m/s)',.05,1.8,.05],
    ['planHz','계획 주기(Hz)',5,40,1],['horizon','계획 거리(m)',.3,2.5,.1],
    ['sampleStep','검사 간격(m)',.02,.08,.01],['margin','패딩(m)',0,.2,.01],
    ['laneLookahead','차선 lookahead(m)',.01,.8,.01],
    ['referenceDistance','회피 목표 거리(m)',.3,3,.1],['lowThreshold','히스토그램 하한',0,999,.1],
    ['highThreshold','히스토그램 상한',.001,1000,.1],['deceleration','계획 감속도(m/s²)',.1,.5,.05],
    ['responseDelay','입력 반응 지연(s)',0,1,.01],['hazardEnabled','연속성 필터(0/1)',0,1,1],
    ['hazardFrames','확인 프레임',1,5,1],['hazardSmall','작은 덩어리 최대 셀',1,50,1],
    ['hazardRadius','대응 반경(셀)',0,2,1],['hazardGap','최대 관측 간격(s)',.02,.5,.01],
    ['extraCurves','추가 곡선(0/1)',0,1,1],['curveSamples','곡선 샘플(홀수)',3,17,2],
    ['lengthSlack','길이 비교 여유(m)',0,.5,.01],['laneWeight','복귀 가중치',0,5,.1],
    ['steeringWeight','조향 연속성 가중치',0,5,.1],
    ['laneTrustSeconds','차선 연속 확인(s)',.05,3,.05],
    ['laneTrustLength','신뢰 최소 경로(m)',.15,2,.05],
    ['laneTrustOffset','관측 위치 일관성(m)',.02,.5,.01],
    ['laneTrustAngle','관측 방향 일관성(°)',1,45,1],
    ['laneFadeSeconds','기억 감쇠 시간(s)',.1,30,.1],
    ['laneFadeDistance','기억 감쇠 거리(m)',.1,30,.1],
    ['laneVisibleLength','보이는 차선 길이(m)',.05,3,.05]
  ]:[
    ['courseLength','코스 길이(m)',4,32,1],
    ['speed','목표 속도(m/s)',.1,3,.1],['range','VFH 전방 거리(m)',.4,3,.1],
    ['angularStep','VFH 각도 간격(°)',.5,10,.5],['planHz','계획 주기(Hz)',5,40,1],
    ['horizon','후보 경로 길이(m)',.3,3,.1],['sampleStep','경로 검사 간격(m)',.01,.1,.01],
    ['margin','안전 여유(m)',0,.3,.01],['laneWidth','차선 폭(m)',.3,1.5,.01],
    ['departure','차선 밖 이탈 한도(m)',0,1.5,.05],['width','차량 폭(m)',.1,.8,.01],
    ['length','차량 전장(m)',.2,1.5,.01],['wheelbase','축간거리(m)',.1,1,.01],
    ['frontOverhang','앞차축→앞범퍼(m)',0,.4,.01],['maxSteer','최대 조향각(°)',5,45,1],
    ['steerRate','조향 변화율(°/s)',10,500,10],['acceleration','가속도(m/s²)',.1,3,.1],
    ['deceleration','제동 감속도(m/s²)',.1,4,.1],['responseDelay','제동 계획 지연(s)',0,.5,.01],
    ['memoryTime','콘 미관측 한도(s)',.1,5,.1],['memoryDistance','콘 미관측 한도(m)',.1,2,.05],
    ['targetWeight','목표 방향 가중치',0,10,.1],['previousWeight','이전 방향 가중치',0,5,.1],
    ['clearanceWeight','통과 여유 가중치',0,3,.05],['lowThreshold','히스토그램 하한',0,.8,.05],
    ['highThreshold','히스토그램 상한',.1,1,.05],
    ['entryReferenceWeight','입구·통로 기준 가중치',0,20,.5],['approachSpeed','입구 탐색 속도 상한(m/s)',.1,2,.1],
    ['corridorLookahead','통로 추종 전방 주시(m)',.3,1.5,.05]
  ];
  function parseScene(state) {
    if(!state||!(M.native?[1,2,3]:[1,2]).includes(state.version))throw Error('지원하지 않는 장면 버전입니다.');
    const migrated={...state.config};if(state.version===1&&migrated.courseLength===8)migrated.courseLength=16;
    // Legacy scenes preserve layout and sensor toggles. Their JS-only tuning
    // and old braking limits are not meaningful for the native planner.
    const input=M.native && state.version<3?Object.fromEntries(
      ['courseLength','laneVisible','occlusion'].filter(key=>key in migrated).map(key=>[key,migrated[key]])):migrated;
    const c=M.validate(input);
    if(M.native)for(const [key,,min,max,step] of fields) {
      if(c[key]<min || c[key]>max || (step>=1 && Math.abs((c[key]-min)/step-Math.round((c[key]-min)/step))>1e-8))
        throw Error(key+': 설정 범위를 확인하세요.');
    }
    if(!Array.isArray(state.obstacles)||state.obstacles.length>180)throw Error('장애물은 최대 180개까지 가능합니다.');
    const obs=state.obstacles.map((o,i)=>{
      if(!['circle','box'].includes(o.shape)||![o.x,o.y,o.w,o.h].every(Number.isFinite)||
        o.x<0||o.x>c.courseLength||Math.abs(o.y)>1.5||o.w<.04||o.w>2||o.h<.04||o.h>3)throw Error('장애물 '+(i+1)+'의 형상·좌표·크기를 확인하세요.');
      return {id:'o'+i,shape:o.shape,x:o.x,y:o.y,w:o.w,h:o.shape==='circle'?o.w:o.h};
    });
    return {config:c,obstacles:obs};
  }
  function snapshot() {return {version:M.native?3:2,config,obstacles:objects};}
  function save() {
    const data=snapshot();
    const summary={scene:data,summary:sim.summary()};
    if(JSON.stringify(summary).length<15500)window.openai?.setWidgetState?.({modelContent:summary,privateContent:{selected,jetsonRatio}})?.catch(()=>{});
  }
  if(savedModel?.scene)try {const s=parseScene(savedModel.scene);config=s.config;objects=s.obstacles;selected=saved?.privateContent?.selected||null;}catch{}
  const colourProbe=document.createElement('span');colourProbe.hidden=true;colourProbe.setAttribute('aria-hidden','true');root.append(colourProbe);
  // Resolve nested theme tokens / light-dark() through CSS before using Canvas2D.
  function colour(name) {colourProbe.style.color=`var(${name})`;return getComputedStyle(colourProbe).color;}
  function palette() {return {fg:colour('--foreground'),muted:colour('--muted-foreground'),bg:colour('--background'),grid:colour('--border'),
    neutral:colour('--muted'),route:colour('--viz-series-1'),hit:colour('--viz-series-2'),memory:colour('--viz-series-3'),danger:colour('--destructive')};}
  function matchedProfile() {
    return planningProfiles.find(p=>Object.entries(p.config).every(([key,value])=>Math.abs(config[key]-value)<1e-9));
  }
  function profileSettings() {
    if(M.native) {
      el('[data-field="planning-profile"]').closest('label').hidden=true;
      el('[data-out="profile-detail"]').textContent='차선 신뢰도·이탈 한도 해제 실험 · C++ · '+config.planHz+'Hz';
      return;
    }
    const select=el('[data-field="planning-profile"]');select.replaceChildren();
    for(const preset of planningProfiles){const option=document.createElement('option');option.value=preset.id;option.textContent=preset.label;select.append(option);}
    const match=matchedProfile();
    if(!match){const option=document.createElement('option');option.value='custom';option.textContent='사용자 설정';option.disabled=true;select.append(option);}
    select.value=match?.id||'custom';
    el('[data-out="profile-detail"]').textContent=`${match?.label||'사용자 설정'} · 계획 ${config.planHz}Hz · 경로 ${config.horizon}m · 검사 ${(config.sampleStep*100).toFixed(0)}cm · 각도 ${config.angularStep}° · 범위 ${config.range}m · 목표 ${config.speed.toFixed(1)}m/s`;
  }
  function settings() {
    const area=el('[data-area="settings"]');area.replaceChildren();
    for(const [key,label,min,max,step]of fields) {
      const l=document.createElement('label');l.className='form-label';l.textContent=label;
      const i=document.createElement('input');i.className='form-control';i.type='number';i.min=min;i.max=max;i.step=M.native && step<1?'any':step;i.value=config[key];i.dataset.param=key;
      l.append(i);area.append(l);
    }
    el('[data-field="occlusion"]').checked=config.occlusion;
    el('[data-field="lane-visible"]').checked=config.laneVisible;
    el('[data-field="jetson-ratio"]').value=jetsonRatio;
    el('[data-object="x"]').max=config.courseLength;
    profileSettings();
  }
  function stopFrames() {running=false;if(frameRequest!==null)cancelAnimationFrame(frameRequest);frameRequest=null;lastFrame=0;}
  function reset() {stopFrames();el('[data-act="play"]').textContent='주행 시작';sim=new M.Simulation(config,objects);accumulator=0;lastPerfAt=-Infinity;renderTimes.length=0;draw();}
  function objectFields() {
    const o=objects.find(v=>v.id===selected);
    el('[data-out="selected"]').textContent=o?`${o.shape==='circle'?'원형':'사각형'} · 전방 ${o.x.toFixed(2)}m · 좌측 ${o.y.toFixed(2)}m · ${objects.length}개`:`선택한 장애물 없음 · ${objects.length}개`;
    for(const input of all('[data-object]')){input.disabled=!o||(o.shape==='circle'&&input.dataset.object==='h');input.value=o?o[input.dataset.object]:'';}
  }
  function worldToCanvas(p) {return {x:worldMap.left+p.x*worldMap.scale,y:worldMap.top+(1.5-p.y)*worldMap.scale};}
  function canvasToWorld(event) {
    const r=el('[data-canvas="world"]').getBoundingClientRect();
    return {x:M.clamp((event.clientX-r.left-worldMap.left)/worldMap.scale,0,config.courseLength),y:M.clamp(1.5-(event.clientY-r.top-worldMap.top)/worldMap.scale,-1.4,1.4)};
  }
  function path(ctx,points,transform,stroke,width=1,dash=[]) {
    if(!points?.length)return;ctx.beginPath();points.forEach((p,i)=>{const q=transform(p);i?ctx.lineTo(q.x,q.y):ctx.moveTo(q.x,q.y);});
    ctx.strokeStyle=stroke;ctx.lineWidth=width;ctx.setLineDash(dash);ctx.stroke();ctx.setLineDash([]);
  }
  function worldDraw(pal) {
    const canvas=el('[data-canvas="world"]'),width=canvas.clientWidth,dpr=window.devicePixelRatio||1;
    const scale=(width-55)/(config.courseLength+.5);
    worldMap={left:35+.5*scale,top:18,scale,height:Math.round(scale*3+48)};
    canvas.width=Math.round(width*dpr);canvas.height=Math.round(worldMap.height*dpr);canvas.style.height=worldMap.height+'px';
    const ctx=canvas.getContext('2d');ctx.scale(dpr,dpr);ctx.clearRect(0,0,width,worldMap.height);
    ctx.font='12px sans-serif';ctx.fillStyle=pal.fg;
    const tickStep=Math.ceil(config.courseLength/8);
    for(let x=0;x<=config.courseLength;x+=tickStep){path(ctx,[{x,y:-1.5},{x,y:1.5}],worldToCanvas,pal.grid);const a=worldToCanvas({x,y:-1.5});ctx.fillText(String(x),a.x-3,a.y+16);}
    for(const y of [-1,0,1]) {path(ctx,[{x:0,y},{x:config.courseLength,y}],worldToCanvas,pal.grid);const a=worldToCanvas({x:0,y});ctx.fillText((y>0?'+':'')+y,3,a.y+4);}
    ctx.fillText('전방(m)',Math.max(35,width-65),worldMap.height-2);ctx.fillText('좌측(m)',0,12);
    const lane=worldToCanvas({x:0,y:config.laneWidth/2});ctx.globalAlpha=.35;ctx.fillStyle=pal.neutral;
    ctx.fillRect(lane.x,lane.y,config.courseLength*worldMap.scale,config.laneWidth*worldMap.scale);ctx.globalAlpha=1;
    for(const y of [-config.laneWidth/2,config.laneWidth/2])path(ctx,[{x:0,y},{x:config.courseLength,y}],worldToCanvas,pal.fg,1);
    if(!M.native)for(const y of [-config.laneWidth/2-config.departure,config.laneWidth/2+config.departure])path(ctx,[{x:0,y},{x:config.courseLength,y}],worldToCanvas,pal.muted,1,[3,4]);
    const half=M.native?config.terrainWidth/2:.6, forward=M.native?config.terrainForward:3;
    const viewport=[{x:0,y:half},{x:forward,y:half},{x:forward,y:-half},{x:0,y:-half},{x:0,y:half}].map(p=>M.world(p,sim.pose));
    ctx.save();ctx.beginPath();ctx.rect(worldMap.left-.5*worldMap.scale,worldMap.top,(config.courseLength+.5)*worldMap.scale,3*worldMap.scale);ctx.clip();
    path(ctx,viewport,worldToCanvas,pal.route,1,[4,3]);
    for(const o of objects) {
      const q=worldToCanvas(o);ctx.fillStyle=pal.neutral;ctx.strokeStyle=o.id===selected?pal.fg:pal.muted;ctx.lineWidth=o.id===selected?2:1;
      ctx.beginPath();if(o.shape==='circle')ctx.arc(q.x,q.y,o.w/2*worldMap.scale,0,Math.PI*2);
      else ctx.rect(q.x-o.w/2*worldMap.scale,q.y-o.h/2*worldMap.scale,o.w*worldMap.scale,o.h*worldMap.scale);
      ctx.fill();ctx.stroke();
    }
    ctx.fillStyle=pal.memory;ctx.globalAlpha=.7;
    for(const p of sim.planner.memory.values()){const q=worldToCanvas(p);ctx.fillRect(q.x-1,q.y-1,2,2);}ctx.globalAlpha=1;
    ctx.fillStyle=pal.hit;for(const p of sim.observation.hits){const q=worldToCanvas(p);ctx.fillRect(q.x-1.5,q.y-1.5,3,3);}
    path(ctx,sim.trace,worldToCanvas,pal.route,2);
    path(ctx,sim.result.path,worldToCanvas,pal.route,1.5,[4,3]);
    path(ctx,sim.result.referencePath,worldToCanvas,pal.muted,1.2,[2,3]);
    const car=M.corners(sim.pose,config);car.push(car[0]);path(ctx,car,worldToCanvas,sim.collided?pal.danger:pal.fg,2);
    path(ctx,[sim.pose,M.world({x:.30,y:0},sim.pose)],worldToCanvas,pal.fg,2);
    ctx.restore();
  }
  function bevDraw(pal) {
    const canvas=el('[data-canvas="bev"]'),ctx=canvas.getContext('2d');ctx.clearRect(0,0,120,300);
    if(M.native) {
      ctx.fillStyle=pal.neutral;ctx.fillRect(0,0,120,300);
      const g=sim.result.grid, sx=120/(g.height*g.res),sy=300/(g.width*g.res);
      const toPixel=p=>({x:120-(p.y-g.y0)*sx,y:300-p.x*sy});
      const pt=p=>toPixel(M.local(p,g.pose));
      ctx.fillStyle=pal.hit;
      for(const index of sim.result.hazards) {
        const p=toPixel({x:(index%g.width+1)*g.res,y:g.y0+(Math.floor(index/g.width)+1)*g.res});
        ctx.fillRect(p.x,p.y,g.res*sx,g.res*sy);
      }
      path(ctx,sim.result.referencePath,pt,pal.fg,.9,[2,2]);
      path(ctx,sim.result.path,pt,pal.route,1.5);
      return;
    }
    ctx.fillStyle=pal.neutral;ctx.globalAlpha=.4;ctx.fillRect(0,0,120,300);ctx.globalAlpha=1;
    const pt=p=>{const q=M.local(p,sim.pose);return {x:60-q.y*100,y:300-q.x*100};};
    // Ray fans expose only measured free space; the unobserved region remains shaded.
    ctx.fillStyle=pal.bg;
    const rays=sim.observation.rays;
    for(let i=1;i<rays.length;i++) {
      const a=rays[i-1],b=rays[i];ctx.beginPath();ctx.moveTo(60,300);
      ctx.lineTo(60-a.distance*Math.sin(a.angle)*100,300-a.distance*Math.cos(a.angle)*100);
      ctx.lineTo(60-b.distance*Math.sin(b.angle)*100,300-b.distance*Math.cos(b.angle)*100);ctx.closePath();ctx.fill();
    }
    ctx.strokeStyle=pal.grid;ctx.lineWidth=.5;
    for(let x=10;x<120;x+=10){ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,300);ctx.stroke();}
    for(let y=0;y<300;y+=10){ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(120,y);ctx.stroke();}
    for(const y of [-config.laneWidth/2,config.laneWidth/2])path(ctx,[{x:sim.pose.x-4,y},{x:sim.pose.x+5,y}],pt,pal.fg,.7);
    ctx.fillStyle=pal.memory;for(const p of sim.planner.memory.values()){const q=pt(p);ctx.fillRect(Math.floor(q.x),Math.floor(q.y),1,1);}
    ctx.fillStyle=pal.hit;for(const p of sim.observation.hits)ctx.fillRect(p.px,p.py,1,1);
    path(ctx,sim.result.path,pt,pal.route,1.1);
    path(ctx,sim.result.referencePath,pt,pal.muted,.7,[2,2]);
    const car=M.corners(sim.pose,config);car.push(car[0]);path(ctx,car,pt,pal.fg,1);
    const y=300-config.range*100;ctx.setLineDash([2,2]);ctx.strokeStyle=pal.muted;ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(120,y);ctx.stroke();ctx.setLineDash([]);
  }
  function histDraw(pal) {
    const cv=el('[data-canvas="hist"]'),w=cv.clientWidth,dpr=window.devicePixelRatio||1;cv.width=w*dpr;cv.height=115*dpr;
    const ctx=cv.getContext('2d');ctx.scale(dpr,dpr);ctx.clearRect(0,0,w,115);
    const left=30,right=w-12,top=10,bottom=83,hist=sim.result.histogram,dx=(right-left)/hist.length;
    for(let i=0;i<hist.length;i++) {ctx.fillStyle=hist[i].blocked?pal.hit:pal.neutral;ctx.globalAlpha=hist[i].blocked?.6:1;
      ctx.fillRect(left+i*dx,bottom-hist[i].density*(bottom-top),dx+.2,hist[i].density*(bottom-top));}ctx.globalAlpha=1;
    ctx.strokeStyle=pal.grid;ctx.strokeRect(left,top,right-left,bottom-top);
    ctx.font='12px sans-serif';ctx.fillStyle=pal.fg;ctx.fillText('밀도',0,18);
    const sectorLimit=M.native?75:90;
    for(const a of [-sectorLimit,0,sectorLimit])ctx.fillText(String(a)+'°',M.clamp(left+(a+sectorLimit)/(2*sectorLimit)*(right-left)-10,left,right-28),103);
    const selectedAngle=sim.planner.lastDirection-(M.native?sim.result.grid.pose.yaw:sim.pose.yaw);const x=M.clamp(left+(selectedAngle/M.DEG+sectorLimit)/(2*sectorLimit)*(right-left),left,right);
    ctx.strokeStyle=pal.route;ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(x,top);ctx.lineTo(x,bottom);ctx.stroke();
  }
  let lastStatus='';
  function performanceDraw(force=false) {
    const now=performance.now();if(!force&&now-lastPerfAt<500)return;lastPerfAt=now;
    if(M.native) {
      const stats=sim.performanceStats(), body=el('[data-out="perf-rows"]');
      body.replaceChildren();
      for(const [key,label] of [['planner','C++ VFH 계획'],['hazard','C++ hazard 확인'],['sensor','브라우저 센서 모사'],['pipeline','센서 + 로컬 왕복 + 계획'],['integration','차량 모사 / 0.01초']]) {
        const row=document.createElement('tr'),sample=stats[key];
        for(const value of [label,sample.count?`${sample.meanMs.toFixed(3)} / ${sample.p95Ms.toFixed(3)} ms`:'—']) {
          const cell=document.createElement('td');cell.textContent=value;row.append(cell);
        }
        body.append(row);
      }
      el('[data-out="perf-source"]').textContent=`Mac 로컬 측정 · 워밍업 20회 제외 · 계획 ${stats.planner.count}회`;
      const cpu=(stats.planner.meanMs+stats.hazard.meanMs)*config.planHz/10;
      el('[data-out="perf-budget"]').textContent=stats.planner.count?
        `필터 + 계획의 단일 코어 시간 환산 ${cpu.toFixed(2)}% · ${config.planHz}Hz`:'주행 또는 0.1초 진행으로 표본을 수집합니다.';
      return;
    }
    const live=sim.performanceStats(),matches=baseline&&baseline.fingerprint===M.fingerprint(config,objects);
    const useLive=live.planner.count>=10,stats=useLive?live:matches?baseline.stages:null;
    const source=useLive?`현재 브라우저 · 워밍업 20회 제외 · 계획 ${live.planner.count}회`:
      matches?`${baseline.host.cpu} · ${baseline.host.memoryGiB}GB · Node ${baseline.method.runs}회 저장 실측 · 현재 배치 일치`:
      `현재 배치 측정 대기 · 주행 후 워밍업 20회 제외 · 표본 ${live.planner.count}/10회`;
    el('[data-out="perf-source"]').textContent=source;
    const body=el('[data-out="perf-rows"]');body.replaceChildren();
    for(const [key,label]of [['planner','회피 계획 / 주기'],['activePlanner','장애물 구간 계획 / 주기'],['sensor','센서 모사 / 주기'],['integration','주행·충돌 평가 / 0.01초'],['pipeline','센서+계획 / 주기']]) {
      const r=document.createElement('tr'),s=stats?.[key],values=[label,s?.count?`${s.meanMs.toFixed(3)} / ${s.p95Ms.toFixed(3)} ms`:'—',
        s?.count?`${(s.meanMs*jetsonRatio).toFixed(3)} / ${(s.p95Ms*jetsonRatio).toFixed(3)} ms`:'—'];
      for(const value of values){const td=document.createElement('td');td.textContent=value;r.append(td);}body.append(r);
    }
    const p=stats?.planner;
    el('[data-out="perf-budget"]').textContent=p?.count?
      `계획 ${config.planHz}Hz · 단일 코어 점유 환산: 맥 ${(p.meanMs*config.planHz/10).toFixed(1)}% / Jetson 가정 ${(p.meanMs*jetsonRatio*config.planHz/10).toFixed(1)}% · Jetson P95 ${(p.p95Ms*jetsonRatio/(1000/config.planHz)*100).toFixed(1)}% 주기 예산`:'주행을 시작하면 현재 배치의 측정값을 표시합니다.';
    if(renderTimes.length)el('[data-out="perf-budget"]').textContent+=` · 화면 평균 ${M.timingStats(renderTimes).meanMs.toFixed(2)}ms/그리기(환산 제외)`;
  }
  function draw() {
    const started=performance.now();
    const pal=palette();worldDraw(pal);bevDraw(pal);histDraw(pal);
    el('[data-out="clock"]').textContent=`${sim.time.toFixed(2)}초 · 전방 ${sim.pose.x.toFixed(2)}m · ${sim.pose.v.toFixed(2)}m/s`;
    let status=sim.collided?'충돌 · 주행 중단':sim.departureViolation&&!M.native&&!sim.result.laneMissing?'이탈 한도 초과 · 주행 중단':sim.done?
      (sim.summary().laneReturned?`${config.courseLength}m 도달 · 차체 전체 차선 내부`:`${config.courseLength}m 도달 · 차선 이탈 남음`):sim.result.status;
    status+=` · 최대 이탈 ${sim.maxDeparture.toFixed(2)}m`;
    if(sim.done)status+=` · 차선 중심 오차 ${Math.abs(sim.pose.y).toFixed(2)}m`;
    if(status!==lastStatus){el('[data-out="status"]').textContent=status;lastStatus=status;}
    el('[data-out="visibility"]').textContent=M.native?`위험 셀 원본 ${sim.result.raw} · 확정 ${sim.result.confirmed} · 대기 ${sim.result.pending}`:`관측 ${sim.observation.hits.length}점 · 기억 ${sim.planner.memory.size}점`;
    el('[data-out="limit"]').textContent=M.native?`이탈 한도 없음 · 차선 ${sim.result.laneTrust} · 기억 선호 ${(100*sim.result.memoryWeight).toFixed(0)}% · 기억 ${sim.result.memoryAge.toFixed(1)}초`:`차선 ${config.laneVisible?'관측':'미관측'} · 차선 경계 밖 ${config.departure.toFixed(2)}m 제한`;
    el('[data-out="course-label"]').textContent=`${config.courseLength}m 코스 · 클릭 배치 / 드래그 이동·콘 배열`;
    el('[data-object="x"]').max=config.courseLength;
    objectFields();
    renderTimes.push(performance.now()-started);if(renderTimes.length>4096)renderTimes.shift();performanceDraw();
  }
  function fail(e) {el('[data-out="error"]').textContent=e.message||String(e);}
  root.addEventListener('click',event=>{
    const action=event.target.closest('[data-act]')?.dataset.act;if(!action)return;
    el('[data-out="error"]').textContent='';
    try {
      if(action==='play'){if(sim.done||sim.collided)reset();if(running){stopFrames();el('[data-act="play"]').textContent='주행 계속';save();}
        else{running=true;lastFrame=0;el('[data-act="play"]').textContent='일시 정지';frameRequest=requestAnimationFrame(frame);}}
      if(action==='step'){stopFrames();el('[data-act="play"]').textContent='주행 계속';for(let i=0;i<10;i++)sim.step();draw();save();}
      if(action==='reset'){reset();save();}
      if(action==='delete'){objects=objects.filter(o=>o.id!==selected);selected=null;reset();save();}
      if(action==='clear'){objects=[];selected=null;reset();save();}
      if(action==='apply'){
        const next={...config};
        for(const input of all('[data-param]')) {
          if(M.native && (!input.value || !input.checkValidity()))throw Error(input.dataset.param+': 설정 범위를 확인하세요.');
          next[input.dataset.param]=Number(input.value);
        }
        next.occlusion=el('[data-field="occlusion"]').checked;config=M.validate(next);settings();reset();save();
      }
      if(action==='export'){el('[data-field="json"]').value=JSON.stringify(snapshot(),null,2);el('[data-field="json"]').focus();el('[data-field="json"]').select();}
      if(action==='import'){const data=parseScene(JSON.parse(el('[data-field="json"]').value));config=data.config;objects=data.obstacles;selected=null;settings();reset();save();}
    }catch(e){fail(e);}
  });
  el('[data-field="preset"]').value=initial||savedModel?.scene?'user':'guidance';
  el('[data-field="preset"]').addEventListener('change',event=>{objects=event.target.value==='user'&&initial?.obstacles?JSON.parse(JSON.stringify(initial.obstacles)):M.presets(event.target.value);selected=null;reset();save();});
  el('[data-field="planning-profile"]').addEventListener('change',event=>{
    const preset=planningProfiles.find(p=>p.id===event.target.value);if(!preset)return;
    try {config=M.validate({...config,...preset.config});settings();reset();save();el('[data-out="error"]').textContent='';}
    catch(error){fail(error);profileSettings();}
  });
  el('[data-field="lane-visible"]').addEventListener('change',event=>{
    config.laneVisible=event.target.checked;sim.c.laneVisible=config.laneVisible;
    sim.senseAndPlan();draw();save();
  });
  document.querySelector("#vfh-course-sim").addEventListener('change',event=>{
    if(!M.native || event.target.dataset.param!=='laneVisibleLength')return;
    const value=Number(event.target.value);
    if(!Number.isFinite(value)||value<.05||value>3)return;
    config.laneVisibleLength=value;sim.c.laneVisibleLength=value;
    sim.senseAndPlan();draw();save();
  });
  el('[data-field="jetson-ratio"]').addEventListener('input',event=>{
    const value=Number(event.target.value);if(!Number.isFinite(value)||value<.1||value>30){event.target.value=jetsonRatio;return;}
    jetsonRatio=value;performanceDraw(true);save();
  });
  for(const input of all('[data-object]'))input.addEventListener('change',()=>{
    const o=objects.find(o=>o.id===selected);if(!o)return;
    const value=Number(input.value);if(!Number.isFinite(value)||value<Number(input.min)||value>Number(input.max)){objectFields();return;}
    o[input.dataset.object]=value;if(o.shape==='circle')o.h=o.w;reset();save();
  });
  const canvas=el('[data-canvas="world"]');
  canvas.addEventListener('pointerdown',event=>{
    const p=canvasToWorld(event),tool=el('[data-field="tool"]').value;
    stopFrames();el('[data-act="play"]').textContent='주행 시작';canvas.setPointerCapture(event.pointerId);
    if(tool==='move') {
      const hit=[...objects].reverse().find(o=>o.shape==='circle'?Math.hypot(o.x-p.x,o.y-p.y)<Math.max(o.w/2,.07):Math.abs(o.x-p.x)<Math.max(o.w/2,.07)&&Math.abs(o.y-p.y)<Math.max(o.h/2,.07));
      selected=hit?.id||null;drag=hit?{type:'move',id:hit.id}:null;
    } else {
      const size=M.clamp(Number(el('[data-field="size"]').value)||.13,.04,1.5);
      const o={id:'o'+Date.now()+objects.length,shape:tool==='box'?'box':'circle',x:p.x,y:p.y,w:size,h:size};
      if(objects.length<180){objects.push(o);selected=o.id;}
      drag=tool==='chain'?{type:'chain',point:p,size}:null;reset();
    }
    draw();
  });
  canvas.addEventListener('pointermove',event=>{
    if(!drag)return;const p=canvasToWorld(event);
    if(drag.type==='move'){const o=objects.find(o=>o.id===drag.id);o.x=p.x;o.y=p.y;reset();}
    else {
      const dx=p.x-drag.point.x,dy=p.y-drag.point.y,len=Math.hypot(dx,dy),spacing=drag.size*.95;
      const count=Math.floor(len/spacing);
      if(count>0){for(let i=1;i<=count&&objects.length<180;i++)objects.push({id:'o'+Date.now()+objects.length,shape:'circle',x:drag.point.x+dx/len*spacing*i,y:drag.point.y+dy/len*spacing*i,w:drag.size,h:drag.size});
        drag.point={x:drag.point.x+dx/len*spacing*count,y:drag.point.y+dy/len*spacing*count};reset();}
    }
  });
  canvas.addEventListener('pointerup',()=>{drag=null;save();});canvas.addEventListener('pointercancel',()=>{drag=null;save();});
  function frame(time) {
    frameRequest=null;
    if(!running)return;
    const delta=lastFrame?Math.min(.08,(time-lastFrame)/1000):0;lastFrame=time;accumulator+=delta;
    try {while(accumulator>=.01&&!sim.done&&!sim.collided){sim.step(.01);accumulator-=.01;}}
    catch(error){stopFrames();el('[data-act="play"]').textContent='주행 계속';fail(error);return;}
    draw();
    if(sim.done||sim.collided){stopFrames();el('[data-act="play"]').textContent='다시 주행';save();}
    else if(running)frameRequest=requestAnimationFrame(frame);
  }
  window.addEventListener('openai:set_globals',event=>{
    const state=event.detail?.globals?.widgetState,model=state?.modelContent||state;if(!model?.scene)return;
    // State saves are echoed by the host. Do not reset the vehicle on our own save.
    if(JSON.stringify(model.scene)===JSON.stringify(snapshot()))return;
    try {const data=parseScene(model.scene);config=data.config;objects=data.obstacles;settings();reset();}catch{}
  });
  new ResizeObserver(()=>{if(sim)draw();}).observe(root);
  new MutationObserver(()=>{if(sim)draw();}).observe(document.documentElement,{attributes:true,attributeFilter:['class','style','data-theme']});
  if(M.native) {
    el('[data-field="jetson-ratio"]').closest('label').hidden=true;
    const timing=el('[data-field="jetson-ratio"]').closest('details');
    timing.querySelector('summary').textContent='Mac 처리 시간';
    timing.querySelector('th:last-child').closest('tr').lastElementChild.remove();
    timing.querySelector('.viz-controls').hidden=true;
    for(const child of timing.querySelectorAll(':scope > div.text-small:not([data-out])'))child.hidden=true;
    el('.sim-legend').textContent='빨강: 확정 장애물 · 회색: 미관측 · 파랑: 계획 경로 · 점선: 차선';
    el('[data-canvas="bev"]').setAttribute('aria-label','차량 전방 terrain과 VFH 선택 경로');
    root.querySelector('.sim-footer').textContent='차선 신뢰도·복귀 정책 실험 · C++ VFH와 hazard 필터 · 이상적인 차량 위치';
  }
  try {settings();reset();}catch(error){fail(error);return;}
  // Small read-only diagnostic surface for the local smoke check.
  root.vfhDiagnostics=()=>({summary:sim.summary(),scene:snapshot(),planningProfile:matchedProfile()?.id||'custom',bev:[el('[data-canvas="bev"]').width,el('[data-canvas="bev"]').height]});
})();
