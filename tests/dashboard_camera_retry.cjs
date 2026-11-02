// Exercise the actual dashboard script, including an offline camera and a
// reopen so brief that the UI never observed camera_online=false.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
let now = 100000;
class ClockDate extends Date { static now() { return now; } }
class Element {
  constructor() {
    this.listeners={}; this.classList={toggle(){},add(){},remove(){}};
    this.lastChild={textContent:''}; this.hidden=false;
    this.style={};
  }
  addEventListener(name,callback) { this.listeners[name]=callback; }
  setAttribute() {}
  querySelector() { return new Element(); }
}
const elements = new Map(), streams = [], calls = [], timers = [];
const element = id => {
  if (!elements.has(id)) elements.set(id,new Element());
  return elements.get(id);
};
Object.defineProperty(element('camera'),'src',{set(value){streams.push(value);}});
const state={first_direction:null,error:null,target_stop_latched:false,arm:null,
  dashboard:{state:'READY',session_id:'camera-session',control_revision:0,attempt:0,
    started:false,finished:false,can_start:false,elapsed_seconds:null,
    arm_enabled:false,arm_calibrated:false,distance_cm:null,remaining_cm:null,
    target_distance_cm:30,camera_online:false,camera_generation:1}};
const context={AbortController,console,Date:ClockDate,performance:{now:()=>now},
  document:{getElementById:element,querySelectorAll:()=>[]},
  setTimeout:(callback,delay)=>{timers.push({callback,delay});return timers.length;},
  clearTimeout:()=>{},setInterval:()=>{},fetch:async(url,options)=>{
    calls.push({url,method:options.method});
    return {ok:true,json:async()=>structuredClone(state)};
  }};
vm.runInNewContext(fs.readFileSync(path.join(__dirname,'../picar_mission/dashboard.js'),'utf8'),context);
const drain=()=>new Promise(resolve=>setImmediate(resolve));
async function poll() {
  const index=timers.findIndex(timer=>timer.delay===500);
  assert(index>=0);
  const timer=timers.splice(index,1)[0];
  await timer.callback();
  await drain();
}
(async()=>{
  await drain();
  assert.equal(streams.length,1);
  now+=6000;
  await poll();
  assert.equal(streams.length,2,'Offline camera must still retry its stream');
  element('camera').listeners.load();
  state.dashboard.camera_online=true;
  now+=1000;
  await poll();
  assert.equal(element('camera-placeholder').hidden,true);
  state.dashboard.camera_generation=2;
  now+=6000;
  await poll();
  assert.equal(streams.length,3,'A new capture generation requires a new MJPEG subscription');
  assert.equal(element('camera-placeholder').hidden,false,'An old image must not be shown as live');
  element('camera').listeners.load();
  assert.equal(element('camera-placeholder').hidden,true);
  assert(calls.every(call=>call.method==='GET'),'Camera recovery must never send motion commands');
  console.log('Offline and missed-disconnect recovery resubscribed without motion commands.');
})().catch(error=>{console.error(error);process.exitCode=1;});
