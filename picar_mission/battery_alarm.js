'use strict';
(() => {
  const root = document.getElementById('battery-alarm');
  if (!root) return;
  root.innerHTML = `<div class="battery-main"><span class="battery-icon" id="battery-icon" aria-hidden="true">◌</span><div><span class="battery-title" id="battery-title" role="status" aria-live="polite">전압 확인 중</span><span class="battery-percent" id="battery-percent">—%</span><span class="battery-voltage" id="battery-voltage">— V</span><p class="battery-detail" id="battery-detail">전압이 낮을 때만 경고음이 울립니다.</p></div></div><div class="battery-controls"><label class="battery-preview-field" id="battery-preview-field" hidden><span class="battery-preview-label">미리보기 가상 전압</span><select class="battery-preview-select" id="battery-preview-voltage" aria-label="미리보기 가상 전압"><option value="7.4">정상 · 7.4 V</option><option value="5.8">부족 · 5.8 V</option></select></label><button class="battery-toggle" id="battery-toggle" type="button" disabled>경고음 확인 중</button></div>`;
  const byId = id => document.getElementById(id);
  const feedback = document.createElement('p');
  feedback.className = 'battery-feedback'; feedback.setAttribute('role','status');
  root.after(feedback);
  let latest = null, busy = false;
  function setText(id, text) { if (byId(id).textContent!==text) byId(id).textContent = text; }
  async function request(path, method='GET', body) {
    const abort = new AbortController();
    const timer = setTimeout(() => abort.abort(),4000);
    try {
      const options = {method,cache:'no-store',signal:abort.signal};
      if (method==='POST') { options.headers={'Content-Type':'application/json'}; options.body=JSON.stringify(body); }
      const response = await fetch(path,options);
      const result = await response.json().catch(() => ({}));
      if (!response.ok) throw Error(result.error || `요청 실패 (${response.status})`);
      return result;
    } catch(error) { if (error.name==='AbortError') throw Error('서버 응답 시간이 초과됐습니다'); throw error; }
    finally { clearTimeout(timer); }
  }
  function render(data) {
    if (typeof data.sound_enabled!=='boolean') throw Error('경고음 상태를 확인할 수 없습니다');
    if (latest && data.session_id===latest.session_id && data.revision<latest.revision) return;
    latest = data;
    const muted = !data.sound_enabled;
    const stale = data.sample_age_seconds!==null && data.sample_age_seconds>6;
    const known = typeof data.voltage_volts==='number' && !stale;
    const warning = known && data.low_voltage;
    const alerting = data.sounding || (warning && !muted);
    root.classList.toggle('alarming',alerting);
    root.classList.toggle('muted',muted);
    root.classList.remove('offline');
    setText('battery-icon',muted?'🔇':alerting?'🔊':'🔈');
    setText('battery-voltage',known?`${data.voltage_volts.toFixed(2)} V`:'— V');
    setText('battery-percent',known&&typeof data.battery_percent==='number'?`${data.battery_percent}%`:'—%');
    setText('battery-title',muted?(warning?'전압 낮음 · 경고음 음소거':'경고음 음소거'):data.sounding?'전압 경고음 울리는 중':warning?'전압 부족 · 경고음 켜짐':known?'전압 정상 · 경고음 대기':'전압 측정 대기');
    setText('battery-detail',data.settings_error?'경고음 저장 설정을 확인해주세요.':muted?`다시 켤 때까지 묵음 유지 · 전압 경고 기준 ${data.threshold_volts} V 미만`:`${data.threshold_volts} V 미만일 때만 경고음이 울립니다. 켜기로 시험 소리를 내지 않습니다.`);
    setText('battery-toggle',busy?'처리 중…':muted?'경고음 켜기':warning?'무시하기':'경고음 끄기');
    byId('battery-toggle').disabled = busy;
    byId('battery-toggle').setAttribute('aria-label',muted?'전압 경고음 켜기':'전압 경고음 끄기');
    byId('battery-preview-field').hidden = !data.preview;
    byId('battery-preview-voltage').disabled = busy;
    if (data.preview && known) byId('battery-preview-voltage').value = data.voltage_volts<6.3?'5.8':'7.4';
  }
  async function poll() {
    if (!busy) {
      try { render(await request('/battery/status')); }
      catch(error) {
        root.classList.add('offline');
        root.classList.remove('alarming');
        setText('battery-title','전압·경고음 상태 확인 필요');
        setText('battery-voltage','— V');
        setText('battery-percent','—%');
        byId('battery-toggle').disabled = true;
      }
    }
    setTimeout(poll,700);
  }
  byId('battery-toggle').addEventListener('click',async() => {
    if (busy || !latest) return;
    busy = true; render(latest);
    feedback.textContent = ''; feedback.classList.remove('error');
    try {
      const enabled = !latest.sound_enabled;
      render(await request('/battery/sound','POST',{enabled}));
      feedback.textContent = enabled?'경고음을 켰습니다. 전압이 낮을 때만 울립니다.':'경고음을 껐습니다. 전압 측정과 표시는 계속됩니다.';
    } catch(error) { feedback.textContent = error.message; feedback.classList.add('error'); }
    finally { busy = false; if (latest) render(latest); }
  });
  byId('battery-preview-voltage').addEventListener('change',async() => {
    if (!latest?.preview || busy) return;
    busy = true;
    try { render(await request('/battery/preview/voltage','POST',{volts:Number(byId('battery-preview-voltage').value)})); }
    catch(error) { feedback.textContent = error.message; feedback.classList.add('error'); }
    finally { busy = false; if (latest) render(latest); }
  });
  poll();
})();
