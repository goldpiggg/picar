// Real browser client: recall, adjust, explicit overwrite and mode gates.
const assert=require('node:assert/strict');
const fs=require('node:fs'),path=require('node:path'),vm=require('node:vm');
class Element {
  constructor(){this.listeners={};this.classList={toggle(){}};this.value='90';this.open=false;}
  addEventListener(name,fn){this.listeners[name]=fn;}
  showModal(){this.open=true;}
  close(value){this.open=false;this.returnValue=value;this.listeners.close?.();}
}
const elements=new Map();
function element(id){if(!elements.has(id))elements.set(id,new Element());return elements.get(id);}
const save=new Element(),move=new Element();
for(const button of [save,move])button.dataset={position:'LEFT-A',pose:'pick'};
save.hasAttribute=name=>name==='data-save';move.hasAttribute=()=>false;
const profile={positions:{},common:{}};
for(const name of ['LEFT-A','RIGHT-A','LEFT-B','RIGHT-B'])profile.positions[name]={approach:null,pick:null,place:null};
for(const name of ['gripper_open','gripper_closed','center_clear','center_carry'])profile.common[name]=null;
const saved={angles_deg:{base:70,shoulder:80,elbow:100},saved_at:'2026-11-04T00:00:00Z'};
profile.positions['LEFT-A'].pick=structuredClone(saved);
profile.positions['RIGHT-B'].place=structuredClone(saved);
const data={profile,preview:false,manual:false,angles:{base:90,shoulder:90,elbow:90,gripper:72},
  move:null,teaching_active:false,can_switch_teaching:true,control_revision:1,session_id:'recall-test'};
const calls=[],timers=[];
const context={AbortController,console,document:{getElementById:element,querySelectorAll:s=>
  s==='[data-save]'?[save]:s==='[data-move]'?[move]:s==='[data-save],[data-move]'?[save,move]:[]},
  setTimeout:(fn,delay)=>{timers.push({fn,delay});return timers.length;},clearTimeout:()=>{},
  fetch:async(url,options)=>{
    const body=options.body?JSON.parse(options.body):null;calls.push({url,body});
    if(url==='/positions/move'){
      assert.equal(body.expected_revision,data.control_revision);
      assert.equal(body.session_id,data.session_id);
      data.control_revision++;
      data.move={name:'LEFT-A / pick',state:'MOVING',targets:structuredClone(saved.angles_deg)};
    }
    if(url==='/positions/save')profile.positions[body.position][body.pose]={
      angles_deg:{base:data.angles.base,shoulder:data.angles.shoulder,elbow:data.angles.elbow},saved_at:'2026-11-04T01:00:00Z'};
    return {ok:true,status:200,json:async()=>structuredClone(data)};
  }};
vm.runInNewContext(fs.readFileSync(path.join(__dirname,'../picar_mission/positions.js'),'utf8'),context);
const drain=()=>new Promise(resolve=>setImmediate(resolve));
async function poll(){const i=timers.findIndex(t=>t.delay===700);assert(i>=0);await timers.splice(i,1)[0].fn();await drain();}
const posts=url=>calls.filter(c=>c.url===url);
(async()=>{
  await drain();
  assert.equal(move.disabled,true);assert.equal(save.disabled,true);
  move.listeners.click();await drain();assert.equal(posts('/positions/move').length,0);
  data.manual=data.teaching_active=true;await poll();
  assert.equal(move.textContent,'▶ 이동');assert.equal(move.disabled,false);
  assert.equal(save.textContent,'다시 저장');
  const original=structuredClone(profile);
  move.listeners.click();await drain();
  assert.equal(element('confirm-dialog').open,false);
  assert.equal(posts('/positions/move').length,1);
  assert.equal(posts('/positions/save').length,0);
  assert.deepEqual(profile,original,'Recall must never change saved values');
  assert.equal(move.textContent,'이동 중…');
  assert.equal(move.disabled,true);assert.equal(save.disabled,true);
  assert.equal(element('arm-stop').disabled,false);
  save.listeners.click();await drain();assert.equal(element('confirm-dialog').open,false);
  data.move.state='DONE';data.angles.base=72;await poll();
  assert.equal(save.disabled,false);save.listeners.click();
  assert.equal(element('confirm-dialog').open,true);
  assert.equal(posts('/positions/save').length,0,'Overwrite needs an explicit confirmation');
  element('confirm-dialog').close('confirm');await drain();
  assert.equal(posts('/positions/save').length,1);
  assert.equal(profile.positions['LEFT-A'].pick.angles_deg.base,72);
  assert.deepEqual(profile.positions['RIGHT-B'],original.positions['RIGHT-B']);
  profile.positions['LEFT-A'].pick=null;await poll();
  assert.equal(move.disabled,true);assert.equal(save.textContent,'현재 자세 저장');
  move.listeners.click();await drain();assert.equal(posts('/positions/move').length,1);
  console.log('Recall preserves data; only explicit save overwrites the selected pose.');
})().catch(error=>{console.error(error);process.exitCode=1;});
