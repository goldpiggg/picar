// Run the real dashboard against all four plans, a restart and stale responses.
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const vm=require('node:vm');
class Element {
  constructor() {
    this.listeners={};this.children=[];this.attributes={};this.dataset={};this.style={};
    this.lastChild={textContent:''};this.textContent='';this.hidden=false;
    const classes=new Set();
    this.classList={toggle(name,enabled){if(enabled)classes.add(name);else classes.delete(name);},
      add:name=>classes.add(name),remove:name=>classes.delete(name),contains:name=>classes.has(name)};
    this.parts=new Map();
  }
  addEventListener(event,callback){this.listeners[event]=callback;}
  setAttribute(name,value){this.attributes[name]=value;}
  replaceChildren(...children){this.children=children;}
  append(...children){this.children.push(...children);}
  querySelector(selector){if(!this.parts.has(selector))this.parts.set(selector,new Element());return this.parts.get(selector);}
}
const positions=['LEFT-A','LEFT-B','RIGHT-A','RIGHT-B'];
const options=positions.map(position=>{const e=new Element();e.dataset.sequenceStart=position;return e;});
const stages=[1,2,3,4,5].map(stage=>{const e=new Element();e.dataset.stage=stage;return e;});
const elements=new Map();
function element(id){if(!elements.has(id))elements.set(id,new Element());return elements.get(id);}
let state={first_direction:null,sequence_selection:null,target_stop_latched:false,
  dashboard:{state:'ARM_RESET',attempt:1,control_revision:1,session_id:'selection-session',
    started:true,finished:false,can_start:false,camera_online:true,target_distance_cm:30,
    distance_cm:80,remaining_cm:50,elapsed_seconds:0,arm_mode:'sequence_preview'}};
let fail=false;
const timers=[],calls=[];
const context={AbortController,Date,console,performance:{now:()=>1000},
  document:{getElementById:element,createElement:()=>new Element(),querySelectorAll:selector=>
    selector==='[data-sequence-start]'?options:selector==='[data-stage]'?stages:[]},
  setTimeout:(callback,delay)=>{timers.push({callback,delay});return timers.length;},
  clearTimeout:()=>{},setInterval:()=>{},fetch:async(url,options)=>{
    calls.push({url,method:options.method});if(fail)throw Error('offline');
    return {ok:true,json:async()=>structuredClone(state)};
  }};
vm.runInNewContext(fs.readFileSync(path.join(__dirname,'../picar_mission/dashboard.js'),'utf8'),context);
const drain=()=>new Promise(resolve=>setImmediate(resolve));
async function poll(){const i=timers.findIndex(t=>t.delay===500);assert(i>=0);await timers.splice(i,1)[0].callback();await drain();}
const paths=[['LEFT-A','RIGHT-A','RIGHT-B','LEFT-B'],['LEFT-B','RIGHT-B','RIGHT-A','LEFT-A'],
  ['RIGHT-A','LEFT-A','LEFT-B','RIGHT-B'],['RIGHT-B','LEFT-B','LEFT-A','RIGHT-A']];
(async()=>{
  await drain();
  assert.equal(element('state-pill').textContent,'팔 초기화 중');
  for(let i=0;i<4;i++){
    state.first_direction=positions[i].split('-')[0];state.target_stop_latched=true;
    state.sequence_selection={number:i+1,start_position:positions[i],steps:paths[i].map((position,j)=>({position,action:j%2?'PLACE':'PICK'}))};
    Object.assign(state.dashboard,{state:'SEQUENCE_READY',control_revision:i+2,finished:true});
    await poll();
    assert.equal(element('sequence-title').textContent,`${i+1}번 · ${positions[i]} PICK으로 시작`);
    assert.deepEqual(options.map(e=>e.classList.contains('selected')),positions.map((_,j)=>j===i));
    assert.deepEqual(element('selected-sequence').children.map(e=>e.children[0].textContent),paths[i]);
    assert.equal(element('step-counter').textContent,'2 / 5');
    assert.equal(stages[2].className,'','Selecting a plan must not mark transport active');
  }
  const previous=structuredClone(state);
  Object.assign(state,{first_direction:null,sequence_selection:null,target_stop_latched:false});
  Object.assign(state.dashboard,{state:'ARM_RESET',attempt:2,control_revision:6,finished:false});
  await poll();
  assert(options.every(e=>!e.classList.contains('selected')));
  assert.equal(element('selected-sequence').hidden,true);
  assert.equal(element('selected-sequence').children.length,0);
  const fresh=structuredClone(state);state=previous;await poll();
  assert.equal(element('state-pill').textContent,'팔 초기화 중','A delayed old poll cannot restore the prior selection');
  assert(options.every(e=>!e.classList.contains('selected')));
  state=previous;state.dashboard.control_revision=7;await poll();
  fail=true;await poll();
  assert(options.every(e=>!e.classList.contains('selected')));
  assert.equal(element('sequence-status').textContent,'연결 확인 필요');
  assert(calls.every(c=>c.method==='GET'),'Displaying selections must not trigger any motion request');
  console.log('All four plans, restart clearing, stale polls and offline state passed.');
})().catch(error=>{console.error(error);process.exitCode=1;});
