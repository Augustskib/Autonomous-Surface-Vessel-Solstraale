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
    const element=id=>{if(!els.has(id))els.set(id,{style:{},value:'',innerText:'',textContent:'',
        classList:{add(){},remove(){},toggle(){}},addEventListener(){},querySelector:()=>({style:{}})});return els.get(id);};
    let chain;chain=new Proxy(function(){},{get:(_,p)=>p==='then'?undefined:chain,apply:()=>chain});
    const context={console,crypto:require('node:crypto').webcrypto,Uint8Array,AbortController,Blob,
        document:{getElementById:element,querySelector:element,querySelectorAll:()=>[],
            addEventListener:(name,fn)=>events[name]=fn,hidden:false},
        navigator:{sendBeacon:(url,body)=>requests.push({url,body,beacon:true})},
        addEventListener:(name,fn)=>events[name]=fn,
        setInterval:fn=>{timers.push(fn);return timers.length;},clearInterval(){},
        setTimeout:()=>1,clearTimeout(){},L:chain,
        fetch:(url,options)=>new Promise((resolve,reject)=>{
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
    r.resolve({ok:true,json:async()=>({status:'ok'})});await flush();
    r=p.requests.at(-1);assert.equal(r.body.intent,'AUTO');p.ack(r);
    assert.equal(await run,true);
});
test('manual takeover while mission acceptance is pending prevents Auto',async()=>{
    const p=page();const run=p.c.startRoute([[60,5]]);
    p.ack(p.requests.at(-1));await flush();
    const mission=p.requests.at(-1);assert.equal(mission.url,'/mission');
    const take=p.c.acquireLease();p.ack(p.requests.at(-1));await take;
    mission.resolve({ok:true,json:async()=>({status:'ok'})});
    assert.equal(await run,false);
    assert.equal(p.c.operator.intent,'MANUAL');
    assert.equal(p.requests.filter(r=>r.body?.intent==='AUTO').length,0);
});
test('actual Continue Mission has no trailing Hold from releasing Manual',async()=>{
    const p=page();vm.runInContext('missionActive=true',p.c);
    let run=p.c.acquireLease();p.ack(p.requests.at(-1));await run;
    run=p.c.resumeAuto();p.ack(p.requests.at(-1));await run;
    assert.deepEqual(p.requests.filter(r=>r.url==='/operator').map(r=>r.body.intent),['MANUAL','AUTO']);
});
test('a second route request cannot race the first accepted route',async()=>{
    const p=page();const run=p.c.startRoute([[60,5]]);
    p.ack(p.requests.at(-1));await flush();
    const mission=p.requests.at(-1);
    assert.equal(await p.c.startRoute([[61,6]]),false);
    assert.equal(p.requests.filter(r=>r.url==='/mission').length,1);
    mission.resolve({ok:true,json:async()=>({status:'ok'})});await flush();
    p.ack(p.requests.at(-1));await run;
});
test('Cancel waits for in-flight mission acceptance and prevents Auto',async()=>{
    const p=page();const run=p.c.startRoute([[60,5]]);
    p.ack(p.requests.at(-1));await flush();
    const mission=p.requests.at(-1);
    const cancel=p.c.cancelMission();p.ack(p.requests.at(-1));await flush();
    assert.equal(p.requests.filter(r=>r.url==='/mission/cancel').length,0);
    mission.resolve({ok:true,json:async()=>({status:'ok'})});await flush();
    assert.equal(p.requests.at(-1).url,'/mission/cancel');
    p.requests.at(-1).resolve({ok:true,json:async()=>({status:'ok'})});
    await cancel;assert.equal(await run,false);
    assert.equal(p.requests.filter(r=>r.body?.intent==='AUTO').length,0);
});
test('WASD takes manual control, key release sends neutral, blur holds',async()=>{
    const p=page();
    p.events.keydown({key:'w',target:{closest:()=>false},preventDefault(){},repeat:false});
    p.ack(p.requests.at(-1));await flush();
    p.c.updateThrust();let r=p.requests.at(-1);
    assert.equal(r.body.left,35);assert.equal(r.body.right,35);p.ack(r);await flush();
    p.events.keyup({key:'w'});p.c.updateThrust();r=p.requests.at(-1);
    assert.equal(r.body.left,0);assert.equal(r.body.right,0);p.ack(r);await flush();
    p.events.blur();r=p.requests.at(-1);assert.equal(r.body.intent,'HOLD');p.ack(r);await flush();
});
