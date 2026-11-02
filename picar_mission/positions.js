'use strict';
(() => {
  const $ = id => document.getElementById(id);
  const positions = ['LEFT-A', 'RIGHT-A', 'LEFT-B', 'RIGHT-B'];
  const poses = [['approach','접근 자세'], ['pick','집기 자세'], ['place','놓기 자세']];
  const common = [['gripper_open','집게 열기'], ['gripper_closed','집게 닫기'], ['center_clear','중앙 대기'], ['center_carry','중앙 운반']];
  const joints = [['base','회전'], ['shoulder','어깨'], ['elbow','팔꿈치'], ['gripper','집게']];
  let latest = null, connected = false, busy = false, pending = null, dialogAction = null, actionVersion = 0;
  const previewAngles = Object.fromEntries(joints.map(([name]) => [name,90]));

  function row(position, pose, label) {
    const key = `${position}-${pose}`;
    return `<div class="pose-row"><div><div class="pose-label">${label}<span class="pose-status" id="status-${key}">미저장</span></div><div class="stored-angles" id="angles-${key}">아직 저장한 자세가 없어요</div></div><div class="pose-actions"><button type="button" data-save data-position="${position}" data-pose="${pose}" disabled>현재 자세 저장</button><button type="button" class="recall" data-move data-position="${position}" data-pose="${pose}" disabled>이동 확인</button></div></div>`;
  }
  $('position-grid').innerHTML = positions.map(position => {
    const side = position.startsWith('LEFT')?'왼쪽':'오른쪽', letter = position.slice(-1);
    return `<article class="position-card ${letter==='B'?'b-row':''}" aria-label="${position}"><header><div class="position-name"><span class="position-letter">${letter}</span><div><h3>${position}</h3><p>${side} · ${letter==='A'?'출발 QR에 가까운 자리':'출발선에 가까운 자리'}</p></div></div><span class="saved-count" id="count-${position}">0 / 3 저장</span></header>${poses.map(([pose,label]) => row(position,pose,label)).join('')}</article>`;
  }).join('');
  $('common-grid').innerHTML = common.map(([pose,label]) => row('COMMON',pose,label)).join('');
  $('joint-grid').innerHTML = joints.map(([name,label]) => `<div class="joint"><div class="joint-top"><span>${label}</span><span>${name.toUpperCase()}</span></div><div class="joint-value"><span id="joint-${name}">—</span><small>°</small></div><input type="range" min="0" max="189" step="1" value="90" id="slider-${name}" aria-label="가상 ${label} 각도" hidden></div>`).join('');

  function feedback(text, error = false) {
    $('feedback').textContent = text;
    $('feedback').classList.toggle('error', error);
  }
  async function request(path, method = 'GET', data) {
    const abort = new AbortController();
    const timeout = setTimeout(() => abort.abort(),5000);
    try {
      const options = {method, cache:'no-store', signal:abort.signal};
      if (method==='POST') { options.headers = {'Content-Type':'application/json'}; options.body = JSON.stringify(data || {}); }
      const response = await fetch(path, options);
      const result = await response.json().catch(() => ({}));
      if (!response.ok) throw Error(result.error || `요청 실패 (${response.status})`);
      return result;
    } catch (error) { if (error.name==='AbortError') throw Error('서버 응답 시간이 초과됐습니다'); throw error; }
    finally { clearTimeout(timeout); }
  }
  function record(position, pose) {
    return position==='COMMON'?latest?.profile?.common?.[pose]:latest?.profile?.positions?.[position]?.[pose];
  }
  function buttons() {
    const canTeach = connected && latest?.manual && latest?.angles;
    const moving = latest?.move?.state==='MOVING';
    document.querySelectorAll('[data-save]').forEach(button => { button.disabled = !canTeach || busy || moving; });
    document.querySelectorAll('[data-move]').forEach(button => { button.disabled = !canTeach || busy || !record(button.dataset.position,button.dataset.pose); });
    $('arm-stop').disabled = !connected;
    joints.forEach(([name]) => { $(`slider-${name}`).disabled = busy || moving || !connected; });
  }
  function connection(value) {
    connected = value;
    ['connection-dot','side-dot'].forEach(id => { $(id).className = 'dot '+(value?'online':'offline'); });
    $('connection-label').textContent = value?(latest?.preview?'PC 미리보기 연결됨':'로봇 서버 연결됨'):'서버 연결 끊김';
    $('side-status').textContent = value?(latest?.preview?'가상 데이터 모드':'로봇 서버 연결됨'):'서버 연결 끊김';
    if (!value) {
      $('mode-notice').hidden = false;
      $('mode-notice').textContent = '서버 연결을 확인해주세요. 저장과 이동은 연결이 돌아오면 사용할 수 있습니다.';
      joints.forEach(([name]) => { $(`joint-${name}`).textContent = '—'; });
      $('move-label').textContent = '현재 팔 상태 확인 불가';
    }
    buttons();
  }
  function angleText(angles) {
    return joints.filter(([name]) => name in angles).map(([name,label]) => `${label} ${Number(angles[name]).toFixed(1)}°`).join(' · ');
  }
  function render(data) {
    if (!data.profile?.positions || !data.profile?.common) throw Error('자리별 자세 데이터를 읽을 수 없습니다');
    if (latest && data.session_id===latest.session_id && data.control_revision<latest.control_revision) return;
    latest = data;
    $('preview-banner').hidden = !data.preview;
    $('legacy-link').hidden = $('legacy-nav').hidden = !data.legacy_available;
    $('manual-link').hidden = Boolean(data.preview);
    $('angle-eyebrow').textContent = data.preview?'SIMULATED ANGLES · PREVIEW ONLY':'CURRENT COMMANDED ANGLES';
    $('angle-title').textContent = data.preview?'가상 팔 각도 조절':'현재 팔 명령각';
    $('angle-help').textContent = data.preview?'슬라이더로 가상 각도를 바꾸고 각 자리의 저장·이동 확인을 눌러보세요.':'기본 웹 제어에서 팔을 맞추고, 실제로 도착한 뒤 해당 자세를 저장하세요.';
    $('mode-tag').textContent = data.preview?'가상 저장 모드':data.manual?'수동 저장 모드':'자동 미션 모드';
    $('mode-notice').hidden = Boolean(data.manual);
    if (!data.manual) $('mode-notice').textContent = '지금은 자동 미션 모드입니다. 자세 저장은 auto_approach_enabled=false, arm_enabled=false로 설정하고 서버를 재시작한 뒤 사용할 수 있습니다.';
    joints.forEach(([name]) => {
      const input = $(`slider-${name}`);
      input.hidden = !data.preview;
      if (data.angles && document.activeElement!==input) {
        const angle = data.angles[name];
        $(`joint-${name}`).textContent = Number(angle).toFixed(1);
        if (data.preview) { input.value = angle; previewAngles[name] = angle; }
      }
    });
    let total = 0;
    function show(position, pose) {
      const value = record(position,pose), key = `${position}-${pose}`;
      $(`status-${key}`).textContent = value?'저장됨':'미저장';
      $(`status-${key}`).classList.toggle('saved', Boolean(value));
      $(`angles-${key}`).textContent = value?angleText(value.angles_deg):'아직 저장한 자세가 없어요';
      $(`angles-${key}`).classList.toggle('has-value',Boolean(value));
      $(`angles-${key}`).title = value?`저장 시각: ${new Date(value.saved_at).toLocaleString('ko-KR')}`:'';
      return value?1:0;
    }
    positions.forEach(position => {
      const count = poses.reduce((sum,[pose]) => sum+show(position,pose),0);
      total += count;
      $(`count-${position}`).textContent = `${count} / 3 저장`;
      $(`count-${position}`).classList.toggle('complete',count===3);
    });
    $('total-progress').textContent = `${total} / 12 SAVED`;
    $('common-progress').textContent = `${common.reduce((sum,[pose]) => sum+show('COMMON',pose),0)} / 4 SAVED`;
    const move = data.move;
    const labels = {MOVING:'이동 중',DONE:'도착',STOPPED:'정지됨',ERROR:'오류'};
    const [movePosition, movePose] = move?.name?.split(' / ') || [];
    const moveName = (movePosition==='COMMON'?'공통':movePosition)+' · '+([...poses,...common].find(([name]) => name===movePose)?.[1] || movePose);
    $('move-label').textContent = move?`${data.preview?'가상 ':''}${moveName} · ${labels[move.state] || move.state}${move.error?' · '+move.error:''}`:'팔 동작 대기';
    connection(true);
  }
  async function refresh() { render(await request('/positions/data')); }
  async function poll() {
    if (!busy && !$('confirm-dialog').open && !joints.some(([name]) => document.activeElement===$(`slider-${name}`))) {
      try { await refresh(); } catch (error) { connection(false); }
    }
    setTimeout(poll,700);
  }
  async function flushPreview() {
    if (latest?.preview) render(await request('/positions/preview/angles','POST',{angles:previewAngles}));
  }
  async function act(kind, position, pose) {
    if (busy || !connected) return;
    const version = actionVersion;
    busy = true; buttons();
    try {
      await flushPreview();
      if (version!==actionVersion) return;
      await request('/positions/'+kind,'POST',{position,pose,expected_revision:latest.control_revision,session_id:latest.session_id});
      await refresh();
      if (version!==actionVersion) return;
      const label = (position==='COMMON'?common:poses).find(([name]) => name===pose)?.[1] || pose;
      feedback(`${latest.preview?'가상 ':''}${position==='COMMON'?'공통':position} · ${label} ${kind==='save'?'저장 완료':'이동 확인 시작'}`);
    } catch (error) { if (version===actionVersion) feedback(error.message,true); }
    finally { busy = false; buttons(); }
  }
  function confirm(kind, position, pose) {
    pending = {kind,position,pose};
    const label = (position==='COMMON'?common:poses).find(([name]) => name===pose)[1];
    const location = position==='COMMON'?'공통':position;
    $('dialog-tag').textContent = latest.preview?'PC PREVIEW · VIRTUAL ONLY':location+' · '+label;
    $('dialog-title').textContent = kind==='save'?'기존 자세를 덮어쓸까요?':latest.preview?'가상 팔을 이동할까요?':'저장한 자세로 팔을 이동할까요?';
    $('dialog-text').textContent = kind==='save'?`${location}의 ${label}를 현재 각도로 다시 저장합니다.`:latest.preview?`${location}의 ${label} 각도로 가상 표시만 이동합니다.`:`${location}의 ${label}로 실제 팔이 이동합니다. 이동 경로를 확인한 뒤 실행하세요.`;
    $('confirm-action').textContent = kind==='save'?'다시 저장':latest.preview?'가상 이동':'팔 이동';
    $('confirm-dialog').returnValue = 'cancel';
    $('confirm-dialog').showModal();
  }
  document.querySelectorAll('[data-save],[data-move]').forEach(button => button.addEventListener('click',() => {
    const {position,pose} = button.dataset, kind = button.hasAttribute('data-save')?'save':'move';
    if (kind==='move' || record(position,pose)) confirm(kind,position,pose);
    else act(kind,position,pose);
  }));
  $('confirm-dialog').addEventListener('close',() => {
    dialogAction = pending; pending = null;
    if ($('confirm-dialog').returnValue==='confirm' && dialogAction) act(dialogAction.kind,dialogAction.position,dialogAction.pose);
  });
  joints.forEach(([name]) => {
    const slider = $(`slider-${name}`);
    slider.addEventListener('input',() => { previewAngles[name] = Number(slider.value); $(`joint-${name}`).textContent = Number(slider.value).toFixed(1); });
    slider.addEventListener('change',async() => {
      try { await flushPreview(); } catch(error) { feedback(error.message,true); }
    });
  });
  $('arm-stop').addEventListener('click',async() => {
    actionVersion += 1;
    // Stop may run during another request; cancel an open recall dialog too.
    if ($('confirm-dialog').open) { pending = null; $('confirm-dialog').close('cancel'); }
    try { await request('/positions/stop','POST',{}); await refresh(); feedback(latest.preview?'가상 이동을 정지했습니다.':'팔 이동을 정지했습니다. 현재 자세와 집게를 유지합니다.'); }
    catch(error) { feedback('정지 확인 필요: '+error.message,true); }
  });
  poll();
})();
