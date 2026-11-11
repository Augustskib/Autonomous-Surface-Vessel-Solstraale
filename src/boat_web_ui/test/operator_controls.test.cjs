const test = require('node:test');
const assert = require('node:assert/strict');
const {OperatorControls} = require('../boat_web_ui/operator_controls.js');

function harness() {
    const requests=[], notices=[];
    const control=new OperatorControls({session:'pilot', notice:m=>notices.push(m),
        changed:()=>{}, post:(url,body)=> url==='/operator/prepare' ? Promise.resolve({operator:{ticket:'test-ticket'}}) : new Promise((resolve,reject)=>{
            requests.push({url,body,resolve,reject});
        })});
    const accept=async i=>{await new Promise(resolve=>setImmediate(resolve));const r=requests[i];r.resolve({status:'ok',operator:{
        request_session:r.body.session, request_seq:r.body.seq,
        session:r.body.session, seq:r.body.seq, accepted:true,
        mode:r.body.intent === 'AUTO' ? 'WAYPOINT' : r.body.intent,
    }});};
    return {control,requests,notices,accept};
}

test('Manual -> Hold -> Auto is ordered with no trailing Release', async()=>{
    const {control:c, requests:r, accept}=harness();
    let p=c.transition('MANUAL');await accept(0);await p;
    p=c.transition('HOLD');await accept(1);await p;
    p=c.transition('AUTO');await accept(2);await p;
    assert.deepEqual(r.map(x=>x.body.intent),['MANUAL','HOLD','AUTO']);
    assert.deepEqual(r.map(x=>x.body.seq),[1,2,3]);
    assert.equal(c.confirmed,true);
});
test('a late heartbeat response cannot undo a newer Hold',async()=>{
    const {control:c,requests:r,accept}=harness();
    let p=c.transition('MANUAL');await accept(0);await p;
    c.left=30;c.right=20;
    const old=c.heartbeat();
    const hold=c.transition('HOLD');await accept(2);await hold;
    await accept(1);await old;
    assert.equal(c.intent,'HOLD');assert.equal(c.ack.mode,'HOLD');
    assert.equal(r[2].body.left,0);
});
test('a failed control request stops renewal and requests Hold',async()=>{
    const {control:c,requests:r,accept,notices}=harness();
    let p=c.transition('AUTO');await accept(0);await p;
    p=c.heartbeat();r[1].reject(new Error('link down'));await p;
    assert.equal(c.intent,'HOLD');assert.equal(c.confirmed,false);
    assert.equal(c.pending,false);
    assert.equal(r[2].body.intent,'HOLD');
    await c.heartbeat();assert.equal(r.length,3);
    assert.match(notices[0],/link down/);
});
test('an old failed request cannot stop a newer manual takeover',async()=>{
    const {control:c,requests:r,accept}=harness();
    const old=c.transition('AUTO');await new Promise(resolve=>setImmediate(resolve));
    const current=c.transition('MANUAL');await accept(1);await current;
    r[0].reject(new Error('old timeout'));await old;
    assert.equal(c.intent,'MANUAL');assert.equal(c.confirmed,true);
    assert.equal(r.length,2);
});
test('unmatched acknowledgement never confirms control',async()=>{
    const {control:c,requests:r}=harness();
    const p=c.transition('MANUAL');await new Promise(resolve=>setImmediate(resolve));
    r[0].resolve({operator:{request_session:'other',request_seq:1,accepted:true}});
    await p;assert.equal(c.confirmed,false);assert.equal(c.intent,'HOLD');
});
test('lost ownership or stale telemetry never silently resumes',async()=>{
    for (const [status,age] of [[{session:'other'},0],[{session:'pilot'},2],
            [{session:'pilot',expired:true},0]]) {
        const {control:c,requests:r,accept}=harness();
        let p=c.transition('AUTO');await accept(0);await p;
        c.observe(status,age);
        await c.heartbeat();
        assert.equal(c.intent,'HOLD');assert.equal(c.confirmed,false);
        assert.equal(r.length,1);
    }
});

test('a superseded preparation never sends its drive command', async()=>{
    const requests=[];
    const c=new OperatorControls({session:'pilot',notice:()=>{},changed:()=>{},
        post:(url,body)=>new Promise(resolve=>requests.push({url,body,resolve}))});
    const old=c.transition('AUTO');
    const hold=c.transition('HOLD');
    requests[1].resolve({operator:{request_session:'pilot',request_seq:2,accepted:true,mode:'HOLD'}});
    await hold;
    requests[0].resolve({operator:{ticket:'late-ticket'}});
    assert.equal(await old,false);
    assert.equal(requests.length,2);
    assert.equal(c.intent,'HOLD');
});

test('an expired preparation sends only a best-effort Hold', async()=>{
    const descriptor=Object.getOwnPropertyDescriptor(globalThis,'performance');
    let now=0;
    Object.defineProperty(globalThis,'performance',{configurable:true,value:{now:()=>now}});
    try {
        const sent=[];
        const c=new OperatorControls({session:'pilot',notice:()=>{},changed:()=>{},post:async(url,body)=>{
            sent.push({url,body});
            if(url==='/operator/prepare'){now=1001;return {operator:{ticket:'late'}};}
            return {};
        }});
        assert.equal(await c.transition('MANUAL'),false);
        assert.deepEqual(sent.map(r=>r.body.intent),['MANUAL','HOLD']);
        assert.equal(sent[0].url,'/operator/prepare');
        assert.equal(c.confirmed,false);
    } finally {Object.defineProperty(globalThis,'performance',descriptor);}
});

test('authority change in fresh telemetry stops renewals', async()=>{
    const {control:c,requests:r,accept}=harness();
    const p=c.transition('MANUAL');await accept(0);await p;
    c.ack.authority='old';
    c.observe({session:'pilot',authority:'replacement',mode:'MANUAL',expired:false},0);
    assert.equal(c.intent,'HOLD');assert.equal(c.confirmed,false);
    await c.heartbeat();assert.equal(r.length,1);
});
