const test=require('node:test'),assert=require('node:assert/strict'),vm=require('node:vm'),path=require('node:path');
const {spawnSync}=require('node:child_process');
const result=spawnSync(process.env.PYTHON||'python3',[path.join(__dirname,'export_pages.py')],{encoding:'utf8'});
assert.equal(result.status,0,result.stderr);
const html=JSON.parse(result.stdout).data;
function setup(){
 const els=new Map(),requests=[];
 const el=id=>{if(!els.has(id))els.set(id,{value:'',textContent:'',disabled:false,children:[],replaceChildren(){this.children=[];this.value='';},appendChild(o){this.children.push(o);if(!this.value)this.value=o.value;}});return els.get(id);};
 const c={document:{getElementById:el,createElement:()=>({})},fetch:async(url,args)=>{requests.push({url,body:JSON.parse(args.body)});return {ok:true,json:async()=>({status:'queued'})};}};
 vm.createContext(c);
 vm.runInContext(html.slice(html.indexOf('let recordingRows ='),html.indexOf('function toggleDetections()')),c);
 return {c,el,requests};
}
test('library renders names as text and only permits naming a stopped recording',()=>{
 const {c,el}=setup();const row={run_id:'run001',name:'<img onerror=bad>',state:'stopped',can_name:true,bytes:123,folder:'/data/run001'};
 c.renderRecordingLibrary({status_fresh:true,recording:false,library:[row]});
 assert.equal(el('record_save').disabled,false);assert.equal(el('record_start').disabled,false);assert.equal(el('record_stop').disabled,true);
 assert.match(el('saved_recordings').children[0].textContent,/<img onerror=bad>/);
 c.renderRecordingLibrary({status_fresh:true,recording:true,library:[{...row,state:'recording',can_name:false}]});
 assert.equal(el('record_save').disabled,true);assert.equal(el('record_start').disabled,true);assert.equal(el('record_stop').disabled,false);
 c.renderRecordingLibrary({status_fresh:false});
 for(const id of ['record_start','record_stop','record_save'])assert.equal(el(id).disabled,true);
});
test('save sends explicit recording ID and waits for recorder acknowledgement',async()=>{
 const {c,el,requests}=setup();el('saved_recordings').value='run001';el('recording_name').value='Lake morning';
 c.saveRecordingName();await new Promise(setImmediate);
 assert.deepEqual(requests[0],{url:'/data/save_name',body:{run_id:'run001',name:'Lake morning'}});
 assert.match(el('recording_feedback').textContent,/waiting for recorder/);
 c.renderRecordingLibrary({status_fresh:true,library:[],command_result:{detail:'Name saved: Lake morning',ok:true,at:1}});
 assert.equal(el('recording_feedback').textContent,'Name saved: Lake morning');
});
test('failed transport does not report a successful save',async()=>{
 const {c,el}=setup();c.fetch=async()=>{throw Error('offline');};
 await c.recordingRequest('/data/save_name',{run_id:'run001',name:'test'});
 assert.match(el('recording_feedback').textContent,/Connection lost/);
});
