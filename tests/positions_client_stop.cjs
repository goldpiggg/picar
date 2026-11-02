// Exercise the actual browser script with a delayed preview request.
// A stop click must cancel recall even before its move request is sent.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');

class Element {
  constructor() { this.listeners={}; this.classList={toggle(){}}; this.value='90'; this.open=false; }
  addEventListener(name,listener) { this.listeners[name]=listener; }
  showModal() { this.open=true; }
  close(value) { this.open=false; this.returnValue=value; this.listeners.close?.(); }
}
const elements = new Map();
const element = id => {
  if (!elements.has(id)) elements.set(id,new Element());
  return elements.get(id);
};
const save = new Element(), move = new Element();
for (const button of [save,move]) button.dataset={position:'LEFT-A',pose:'approach'};
save.hasAttribute = name => name==='data-save';
move.hasAttribute = () => false;
const profile = {positions:{},common:{}};
for (const position of ['LEFT-A','RIGHT-A','LEFT-B','RIGHT-B']) {
  profile.positions[position]={approach:null,pick:null,place:null};
}
for (const pose of ['gripper_open','gripper_closed','center_clear','center_carry']) profile.common[pose]=null;
profile.positions['LEFT-A'].approach={angles_deg:{base:70,shoulder:80,elbow:100},saved_at:'2026-11-02T00:00:00Z'};
const data={profile,preview:true,manual:true,legacy_available:false,
  angles:{base:90,shoulder:90,elbow:90,gripper:90},move:null,control_revision:0,session_id:'preview-test'};
const calls=[];
let finishPreview;
const response = () => ({ok:true,status:200,json:async()=>structuredClone(data)});
const context={AbortController,console,document:{activeElement:null,getElementById:element,
  querySelectorAll: selector => selector==='[data-save]'?[save]:selector==='[data-move]'?[move]:selector==='[data-save],[data-move]'?[save,move]:[]},
  setTimeout:()=>1,clearTimeout:()=>{},fetch:async(url)=>{
    calls.push(url);
    if (url==='/positions/preview/angles') return new Promise(resolve=>{ finishPreview=()=>resolve(response()); });
    if (url==='/positions/stop') data.control_revision++;
    return response();
  }};
const source=fs.readFileSync(path.join(__dirname,'../picar_mission/positions.js'),'utf8');
vm.runInNewContext(source,context,{filename:'positions.js'});
const drain=()=>new Promise(resolve=>setImmediate(resolve));
(async()=>{
  await drain();
  move.listeners.click();
  const dialog=element('confirm-dialog');
  assert.equal(dialog.open,true);
  dialog.close('confirm');
  await drain();
  assert.equal(typeof finishPreview,'function');
  await element('arm-stop').listeners.click();
  finishPreview();
  await drain();
  await drain();
  assert(calls.includes('/positions/stop'));
  assert(!calls.includes('/positions/move'),'A cancelled preflight must never send a move request');
  assert.match(element('feedback').textContent,/정지/);
  console.log('Stop cancelled delayed recall before any move request.');
})().catch(error=>{console.error(error);process.exitCode=1;});
