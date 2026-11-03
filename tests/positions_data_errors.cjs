// A reachable server with a bad pose file must not be called disconnected.
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const vm=require('node:vm');
class Element {
  constructor(){this.listeners={};this.classList={toggle(){}};this.open=false;this.value='90';}
  addEventListener(name,callback){this.listeners[name]=callback;}
}
const elements=new Map();
function element(id){if(!elements.has(id))elements.set(id,new Element());return elements.get(id);}
const save=new Element(),move=new Element();
save.dataset=move.dataset={position:'LEFT-A',pose:'approach'};
const profile={positions:{},common:{}};
for(const name of ['LEFT-A','RIGHT-A','LEFT-B','RIGHT-B'])profile.positions[name]={approach:null,pick:null,place:null};
for(const name of ['gripper_open','gripper_closed','center_clear','center_carry'])profile.common[name]=null;
const data={profile,preview:false,manual:true,angles:{base:90,shoulder:90,elbow:90,gripper:90},move:null,
  teaching_active:true,can_switch_teaching:true,control_revision:1,session_id:'pose-data'};
let mode='bad-file';
const timers=[],calls=[];
const context={AbortController,console,document:{getElementById:element,querySelectorAll:selector=>
  selector==='[data-save]'?[save]:selector==='[data-move]'?[move]:[]},
  setTimeout:(callback,delay)=>{timers.push({callback,delay});return timers.length;},clearTimeout:()=>{},
  fetch:async(url,options)=>{
    calls.push({url,method:options.method});
    if(mode==='offline')throw Error('connection closed');
    if(mode==='bad-file')return {ok:false,status:400,json:async()=>({error:'old pose file names'})};
    return {ok:true,status:200,json:async()=>mode==='invalid-json'?{}:structuredClone(data)};
  }};
vm.runInNewContext(fs.readFileSync(path.join(__dirname,'../picar_mission/positions.js'),'utf8'),context);
const drain=()=>new Promise(resolve=>setImmediate(resolve));
async function poll(){const i=timers.findIndex(t=>t.delay===700);assert(i>=0);await timers.splice(i,1)[0].callback();await drain();}
(async()=>{
  await drain();
  assert.equal(element('connection-label').textContent,'로봇 서버 연결됨');
  assert.match(element('mode-notice').textContent,/데이터/);
  assert.match(element('mode-notice').textContent,/old pose file names/);
  assert.equal(element('slider-base').disabled,true);
  assert.equal(element('arm-stop').disabled,false,'Stop stays available when the server responds');
  mode='offline';await poll();
  assert.equal(element('connection-label').textContent,'서버 연결 끊김');
  assert.equal(element('slider-base').disabled,true);
  mode='valid';await poll();
  assert.equal(element('connection-label').textContent,'로봇 서버 연결됨');
  assert.equal(element('mode-notice').hidden,true);
  assert.equal(element('slider-base').disabled,false);
  assert.equal(save.disabled,false);
  mode='invalid-json';await poll();
  assert.equal(element('connection-label').textContent,'로봇 서버 연결됨');
  assert.match(element('mode-notice').textContent,/저장 데이터를/);
  assert.equal(element('slider-base').disabled,true);
  assert.equal(save.disabled,true,'A stale valid response cannot re-enable controls');
  assert(calls.every(call=>call.method==='GET'));
  console.log('Pose-data errors, actual disconnects and recovery are distinguished.');
})().catch(error=>{console.error(error);process.exitCode=1;});
