// Executes the actual composed Pilot-page JS with fake map/DOM/network.
// No motors, ROS, or Gazebo. Does not establish browser rendering or ROS timing.
const test=require('node:test'), assert=require('node:assert/strict');
const vm=require('node:vm'), path=require('node:path');
const {spawnSync}=require('node:child_process');
const result=spawnSync(process.env.PYTHON || 'python3',
    [path.join(__dirname,'export_pages.py')],{encoding:'utf8'});
assert.equal(result.status,0,result.stderr);
const pages=JSON.parse(result.stdout);
function scripts(html) {return [...html.matchAll(/<script(?:\s[^>]*)?>([\s\S]*?)<\/script>/g)].map(m=>m[1]);}
function page() {
    const requests=[],els=new Map(),events={},timers=[];
    const element=id=>{if(!els.has(id))els.set(id,{style:{},dataset:{},value:'',innerText:'',textContent:'',
        classList:{add(){},remove(){},toggle(){}},addEventListener(){},querySelector:()=>({style:{}}),
        replaceChildren(){},appendChild(){},setAttribute(){},removeAttribute(){},getAttribute:()=>null,focus(){},
        firstElementChild:{style:{}}});return els.get(id);};
    // Stand-in for Leaflet: every property and call returns the same object,
    // except properties a test assigns explicitly.
    const over={};let chain;chain=new Proxy(function(){},{get:(_,p)=>p==='then'?undefined:(p in over?over[p]:chain),
        set:(_,p,v)=>{over[p]=v;return true;},apply:()=>chain});
    const context={console,performance,crypto:require('node:crypto').webcrypto,Uint8Array,AbortController,Blob,
        document:{getElementById:element,querySelector:element,querySelectorAll:()=>[],
            createElement:()=>element(Symbol()),
            addEventListener:(name,fn)=>events[name]=fn,hidden:false},
        navigator:{sendBeacon:(url,body)=>requests.push({url,body,beacon:true})},
        addEventListener:(name,fn)=>events[name]=fn,
        setInterval:fn=>{timers.push(fn);return timers.length;},clearInterval(){},
        setTimeout:()=>1,clearTimeout(){},L:chain,
        fetch:(url,options)=> url==='/operator/prepare' ? Promise.resolve({ok:true,json:async()=>({status:'ok',operator:{ticket:'test-ticket'}})}) : new Promise((resolve,reject)=>{
            requests.push({url,body:options?.body?JSON.parse(options.body):null,resolve,reject});
        })};
    context.window=context;vm.createContext(context);
    for (const script of scripts(pages.control))new vm.Script(script).runInContext(context);
    const ack=r=>r.resolve({ok:true,json:async()=>({status:'ok',operator:{
        request_session:r.body.session,request_seq:r.body.seq,session:r.body.session,
        seq:r.body.seq,accepted:true,mode:r.body.intent==='AUTO'?'WAYPOINT':r.body.intent,
    }})});
    return {c:context,requests,events,els,timers,ack};
}
const flush=()=>new Promise(resolve=>setImmediate(resolve));

test('heading readout shows fused, unknown and raw IMU headings distinctly',()=>{
    const p=page();
    p.c.updateHeading({quality:2,heading_deg:90,source:'FUSED'},null);
    assert.equal(p.els.get('heading_val').innerText,'90.0°');
    assert.equal(p.els.get('heading_val').className,'grn');
    assert.equal(p.els.get('hdgq_val').innerText,'FUSED');
    p.c.updateHeading({quality:0,heading_deg:0},null);
    assert.equal(p.els.get('heading_val').innerText,'--');
    assert.equal(p.els.get('hdgq_val').className,'red');
    p.c.updateHeading({},270);
    assert.equal(p.els.get('heading_val').innerText,'270.0°');
    assert.equal(p.els.get('hdgq_val').innerText,'NO ESTIMATOR');
});

test('telemetry renders with no GPS fix and places the boat at the fence centre',()=>{
    const p=page();
    p.c.renderTelemetry({operator_age_s:0.1,operator:{mode:'HOLD'},state:'IDLE',lat:null,lon:null,
        heading:null,roll:null,pitch:null,estop:false,
        safety:{level:0,fence_enabled:true,fence_center_lat:28.54,fence_center_lon:-81.379,
            fence_radius_m:100,fence_warn_radius_m:90,fence_margin_m:null,fence_outside_sec:null,
            gcs_heartbeat_age_sec:0.5},
        failsafe:{level:0,heading_usable:false},recording:{status_fresh:true,recording:false,library:[]},
        power:{seen:false},system:{},camera:{},lidar:{fresh:true,timestamp_mode:'receive'}});
    assert.equal(p.els.get('gps_val').innerText,'NO FIX');
    assert.equal(p.els.get('pos_note').hidden,false);
    assert.equal(p.els.get('mode_state').innerText,'HOLD');
    assert.equal(p.els.get('lidar_badge').innerText,'LIDAR LIVE');
    assert.equal(p.els.get('fs_word').innerText,'NO USABLE HEADING');
    assert.equal(p.els.get('record_start').disabled,false);
    assert.equal(p.els.get('reset_btn').hidden,true);
    const pose=vm.runInContext('boatPose',p.c);
    assert.equal(pose.assumed,true);assert.equal(pose.lat,28.54);
});

test('a still boat eases toward jittery fixes; motion and big jumps show the raw fix',()=>{
    const p=page();
    let f=p.c.smoothFix(28.5404,-81.3790,false);
    assert.deepEqual([...f],[28.5404,-81.3790]);
    f=p.c.smoothFix(28.5405,-81.3790,false);            // ~11 m jitter
    assert.ok(Math.abs(f[0]-28.540415)<1e-6,'moves 15% of the way');
    f=p.c.smoothFix(28.5405,-81.3790,true);             // moving: raw
    assert.deepEqual([...f],[28.5405,-81.3790]);
    f=p.c.smoothFix(28.5500,-81.3790,false);            // >50 m: raw
    assert.deepEqual([...f],[28.5500,-81.3790]);
});

test('LiDAR points rotate with heading: forward is north at 0 and east at 90',()=>{
    const p=page();const drawn=[];
    p.els.set('lidar_canvas',{width:0,height:0,getContext:()=>({clearRect(){},fillRect:(x,y)=>drawn.push([x,y]),fillStyle:''})});
    vm.runInContext(`map.getSize=()=>({x:800,y:600});map.getZoom=()=>20;
        map.latLngToContainerPoint=()=>({x:400,y:300});
        layers.lidar=true;lidarYaw=0;lidarScan={fresh:true,xy:[1000,0]};
        boatPose={lat:0,lon:0,heading:0,assumed:false};`,p.c);
    p.c.redrawLidar();
    vm.runInContext('boatPose.heading=90',p.c);p.c.redrawLidar();
    const [north,east]=drawn;
    assert.ok(Math.abs(north[0]-399)<1&&north[1]<300-50,'forward point should be drawn up the screen');
    assert.ok(east[0]>400+50&&Math.abs(east[1]-299)<1,'forward point should be drawn to the right');
});

test('operator history is bounded and renders messages as plain text',()=>{
    const p=page();
    for(let i=0;i<100;i++)p.c.notifyOperator('event '+i);
    p.c.notifyOperator('<img src=x onerror=unsafe()>');
    const lines=p.els.get('operator_event_log').textContent.split('\n');
    assert.equal(lines.length,80);
    assert.match(lines[0],/<img src=x onerror=unsafe\(\)>/);
    assert.doesNotMatch(lines.join('\n'),/event 0\n/);
});
test('both composed pages have valid JS and no blocking alert/prompt',()=>{
    for(const html of Object.values(pages))for(const s of scripts(html)){
        new vm.Script(s);assert.doesNotMatch(s,/\b(?:alert|prompt)\s*\(/);
    }
});
test('Pilot page initializes and closing a spectator sends no control',()=>{
    const p=page();p.events.beforeunload();
    assert.equal(p.requests.filter(r=>r.url==='/operator').length,0);
});
test('actual Start Mission waits for Hold and mission acceptance before Auto',async()=>{
    const p=page();const run=p.c.startRoute([[60,5]]);
    let r=p.requests.at(-1);assert.equal(r.body.intent,'HOLD');p.ack(r);await flush();
    r=p.requests.at(-1);assert.equal(r.url,'/mission');
    r.resolve({ok:true,json:async()=>({status:'ok',execution_id:'test-run'})});await flush();
    r=p.requests.at(-1);assert.equal(r.body.intent,'AUTO');p.ack(r);
    assert.equal(await run,true);
});
test('manual takeover while mission acceptance is pending prevents Auto',async()=>{
    const p=page();const run=p.c.startRoute([[60,5]]);
    p.ack(p.requests.at(-1));await flush();
    const mission=p.requests.at(-1);assert.equal(mission.url,'/mission');
    const take=p.c.acquireLease();await flush();p.ack(p.requests.at(-1));await take;
    mission.resolve({ok:true,json:async()=>({status:'ok',execution_id:'test-run'})});
    assert.equal(await run,false);
    assert.equal(p.c.operator.intent,'MANUAL');
    assert.equal(p.requests.filter(r=>r.body?.intent==='AUTO').length,0);
});
test('actual Continue Mission has no trailing Hold from releasing Manual',async()=>{
    const p=page();vm.runInContext('missionActive=true',p.c);
    let run=p.c.acquireLease();await flush();p.ack(p.requests.at(-1));await run;
    run=p.c.resumeAuto();await flush();p.ack(p.requests.at(-1));await run;
    assert.deepEqual(p.requests.filter(r=>r.url==='/operator').map(r=>r.body.intent),['MANUAL','AUTO']);
});
test('a second route request cannot race the first accepted route',async()=>{
    const p=page();const run=p.c.startRoute([[60,5]]);
    p.ack(p.requests.at(-1));await flush();
    const mission=p.requests.at(-1);
    assert.equal(await p.c.startRoute([[61,6]]),false);
    assert.equal(p.requests.filter(r=>r.url==='/mission').length,1);
    mission.resolve({ok:true,json:async()=>({status:'ok',execution_id:'test-run'})});await flush();
    p.ack(p.requests.at(-1));await run;
});
test('Cancel waits for in-flight mission acceptance and prevents Auto',async()=>{
    const p=page();const run=p.c.startRoute([[60,5]]);
    p.ack(p.requests.at(-1));await flush();
    const mission=p.requests.at(-1);
    const cancel=p.c.cancelMission();p.ack(p.requests.at(-1));await flush();
    assert.equal(p.requests.filter(r=>r.url==='/mission/cancel').length,0);
    mission.resolve({ok:true,json:async()=>({status:'ok',execution_id:'test-run'})});await flush();
    assert.equal(p.requests.at(-1).url,'/mission/cancel');
    p.requests.at(-1).resolve({ok:true,json:async()=>({status:'ok',execution_id:'test-run'})});
    await cancel;assert.equal(await run,false);
    assert.equal(p.requests.filter(r=>r.body?.intent==='AUTO').length,0);
});
test('WASD takes manual control, key release sends neutral, blur holds',async()=>{
    const p=page();
    p.events.keydown({key:'w',target:{closest:()=>false},preventDefault(){},repeat:false});
    await flush();p.ack(p.requests.at(-1));await flush();
    p.c.updateThrust();let r=p.requests.at(-1);
    assert.equal(r.body.left,35);assert.equal(r.body.right,35);p.ack(r);await flush();
    p.events.keyup({key:'w'});p.c.updateThrust();r=p.requests.at(-1);
    assert.equal(r.body.left,0);assert.equal(r.body.right,0);p.ack(r);await flush();
    p.events.blur();r=p.requests.at(-1);assert.equal(r.body.intent,'HOLD');p.ack(r);await flush();
});
