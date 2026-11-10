/* Vehicle inputs are synthesized here. The ROS C++ planner owns all planning decisions. */
(() => {
  const M=window.VFHSim;
  const defaults={
    speed:1, horizon:1.2, sampleStep:.03, margin:.02,
    frontOverhang:.05, rearExtent:.45, deceleration:.4, responseDelay:.1,
    acceleration:.8, lowThreshold:1, highThreshold:2, laneLookahead:.05, referenceDistance:1,
    terrainWidth:1.5, terrainForward:4, terrainCell:.025, planHz:20,
    hazardEnabled:1, hazardFrames:4, hazardSmall:8, hazardRadius:1, hazardGap:.15,
    extraCurves:1, curveSamples:17, lengthSlack:.40, laneWeight:1.5, steeringWeight:.1, laneTrustSeconds:.8, laneTrustLength:.4, laneTrustOffset:.15, laneTrustAngle:12,
    laneVisibleLength:3,
    clearanceWeight:1
  };
  Object.assign(M.defaults,defaults);
  // Track selection never overrides the single set of VFH defaults.
  M.trackDefaults={};
  const ReferenceSimulation=M.Simulation;
  class NativePlanner {
    constructor(config) {
      this.config=config;
      this.ownsDeparturePolicy=true;
      this.session=crypto.randomUUID();
      this.memory=new Map();
      this.planCount=0;
      this.entryCount=0;
    }
    plan(observation, pose, time) {
      const c=this.config, res=c.terrainCell;
      const width=Math.ceil(c.terrainForward/res),height=Math.ceil(c.terrainWidth/res),y0=-c.terrainWidth/2;
      const hazards=new Set();
      for(const hit of observation.hits) {
        const p=M.local(hit,pose),u=Math.floor(p.x/res),v=Math.floor((p.y-y0)/res);
        if(u>=0 && u<width && v>=0 && v<height)hazards.add(v*width+u);
      }
      const lane=[];
      if(c.laneVisible) {
        const position=c.trackLayout==='loop'?M.projectTrack(pose).s:pose.x;
        for(let distance=0;distance<=3.1;distance+=.02) {
          const source=c.trackLayout==='loop'?M.trackPoint(position+distance):{x:position+distance,y:0};
          const p=M.local(source,pose);
          if(p.x>=.01 && p.x<=Math.min(2.98,c.laneVisibleLength) && Math.abs(p.y)<=.6)lane.push(p);
        }
      }
      const data=[this.session,time,pose.x,pose.y,pose.yaw,pose.v,pose.steer,
        c.planHz,c.speed,c.horizon,c.sampleStep,c.margin,c.laneWidth,c.width,c.rearExtent,
        c.frontOverhang,c.wheelbase,c.maxSteer*M.DEG,c.steerRate*M.DEG,c.deceleration,c.responseDelay,
        c.laneLookahead,c.referenceDistance,c.lowThreshold,c.highThreshold,c.hazardEnabled,c.hazardFrames,
        c.hazardSmall,c.hazardRadius,c.hazardGap,c.extraCurves,c.curveSamples,c.lengthSlack,c.laneWeight,
        c.steeringWeight,width,height,res,y0,c.laneTrustSeconds,c.laneTrustLength,c.laneTrustOffset,
        c.laneTrustAngle,c.clearanceWeight,lane.length,hazards.size,
        ...lane.flatMap(p=>[p.x,p.y]),...hazards];
      const req=new XMLHttpRequest();
      req.open('POST','/plan',false);
      req.send(data.join(' '));
      if(req.status!==200)throw Error('로컬 C++ 계획기 요청 실패');
      const result=JSON.parse(req.responseText);
      if(result.error)throw Error(result.error);
      const path=result.path.map(([x,y])=>M.world({x,y},pose));
      this.lastDirection=pose.yaw;
      if(result.path.length) {
        const [x,y]=result.path.at(-1);
        this.lastDirection+=Math.atan2(y,x);
      }
      this.planCount++;
      const peak=Math.max(1,...result.histogram.map(([value])=>value));
      return {...result,path,referencePath:(result.memoryPath||[]).map(([x,y])=>M.world({x,y},pose)),active:!!result.usedVfh,
        points:result.hazards,histogram:result.histogram.map(([value,blocked])=>({density:value/peak,blocked})),
        grid:{width,height,res,y0,pose:{...pose}}};
    }
  }
  M.Simulation=class extends ReferenceSimulation {
    constructor(config,obstacles,markers=[]) {
      super(config?.trackLayout==='loop'?{...M.trackDefaults,...config}:config,
        obstacles,c=>new NativePlanner(c),markers);
    }
  };
  M.native=true;
})();
