'use strict';
(() => {
  const $ = id => document.getElementById(id);
  const TERMINAL = new Set(['STOPPED_WAIT','QR_VIEW_READY','COMPLETE','CANCELLED','ERROR','DISABLED']);
  const COPY = {
    READY: ['시작 준비 완료','준비됐나요? 이제 미션을 시작하세요.','출발선에 차와 물체를 배치한 뒤 시작 버튼을 눌러주세요.'],
    INITIALIZING: ['준비 중','로봇을 준비하고 있어요','카메라와 제어 장치의 초기화를 기다리고 있습니다.'],
    ARM_TEACHING: ['팔 조절 중','팔 자세 저장 화면에서 조절 중이에요.','팔 수동 조절을 종료하면 새 미션을 시작할 수 있습니다.'],
    WAIT_QR: ['QR 대기 중','출발 QR을 기다리고 있어요','카메라에 LEFT 또는 RIGHT QR을 보여주세요. 인식하면 직진합니다.'],
    STEER_PRELOAD: ['출발 준비','QR 확인. 조향을 정렬하고 있어요.','방향을 확정하고 출발 전 조향 유격을 정리합니다.'],
    STEER_SETTLE: ['출발 준비','곧 직진 주행을 시작합니다.','조향이 가운데에 도착할 때까지 기다립니다.'],
    DRIVING: ['주행 중','정지선을 향해 직진하고 있어요.','초음파 거리와 자이로 방향을 확인하며 접근합니다.'],
    WAIT_DISTANCE: ['거리 재측정','잠시 멈추고 거리를 다시 확인해요.','유효한 초음파 측정값이 돌아올 때까지 바퀴를 멈춥니다.'],
    PULSE_SETTLE: ['정밀 접근','정지 위치를 정밀하게 맞추고 있어요.','멈춘 상태에서 측정한 거리로 다음 이동을 결정합니다.'],
    PULSING: ['정밀 접근','조금씩 움직여 정지선에 맞추고 있어요.','짧은 전진·후진 펄스로 목표 거리까지 조절합니다.'],
    ARM_WAIT: ['QR 확인 준비','정지 완료. 팔 회전을 준비해요.','바퀴를 잠근 상태로 안정 대기한 뒤 QR 방향으로 BASE만 돌립니다.'],
    QR_TURNING: ['팔 회전 중','PICK·PLACE QR을 볼 방향으로 돌고 있어요.','BASE만 회전합니다. 도착하면 그 자세에서 대기합니다.'],
    QR_VIEW_READY: ['QR 확인 준비 완료','팔 방향을 맞췄어요. 다음 단계를 기다립니다.','PICK·PLACE QR을 볼 준비가 됐습니다. 자동 집기·놓기는 시작하지 않습니다.'],
    LAYOUT_WAIT: ['배치 확인 중','물체가 어느 대각선에 있는지 보고 있어요.','PICK/PLACE QR로 배치를 확정한 뒤 팔을 움직입니다. 확정하지 못하면 팔을 움직이지 않습니다.'],
    ARM_TRANSFER: ['물체 운반 중','저장한 자세로 물체를 옮기고 있어요.','차체를 정지한 채 지정한 순서대로 두 물체를 운반합니다.'],
    STOPPED_WAIT: ['주행 완료','정지선에 도착했어요.','팔 회전 준비 상태를 확인할 때까지 정지해 있습니다.'],
    COMPLETE: ['미션 완료','두 물체 운반 순서를 완료했어요.','경과 시간 기록을 종료했습니다. 실제 물체가 옮겨졌는지 확인해주세요.'],
    CANCELLED: ['중지됨','미션을 중지했어요.','차와 물체를 출발 위치로 옮긴 뒤 새 미션을 시작할 수 있습니다.'],
    ERROR: ['오류 발생','미션이 중단됐어요.','아래 오류 내용을 확인하고 원인을 해결한 뒤 다시 시작해주세요.'],
    DISABLED: ['수동 모드','지금은 수동 조종 모드예요.','자동 미션 설정을 켜고 서버를 재시작하면 여기서 시작할 수 있습니다.']
  };
  let latest = null, connected = false, busy = false, stopping = false, lastAttempt = null, lastSession = null;
  let startDistance = null, elapsed = 0, timerAt = performance.now(), timerRunning = false;
  let cameraRetryAt = 0, cameraLoaded = false, cameraFailed = false;
  let cameraGeneration = null, cameraWasOnline = false;
  let armSpeedEditing = false, armSpeedSaving = false;

  function feedback(message, error = false) {
    $('action-feedback').textContent = message;
    $('action-feedback').classList.toggle('error',error);
  }
  async function request(url, method = 'GET', body = {}) {
    const abort = new AbortController();
    const timeout = setTimeout(() => abort.abort(),4000);
    try {
      const options = {method,cache:'no-store',signal:abort.signal};
      if (method === 'POST') {
        options.headers = {'Content-Type':'application/json'};
        options.body = JSON.stringify(body);
      }
      const response = await fetch(url,options);
      const data = await response.json().catch(() => ({}));
      if (!response.ok) throw new Error(data.error || `요청 실패 (HTTP ${response.status})`);
      return data;
    } catch (error) {
      if (error.name === 'AbortError') throw new Error('서버 응답 시간이 초과됐습니다');
      throw error;
    } finally { clearTimeout(timeout); }
  }
  function setConnected(value) {
    connected = value;
    for (const id of ['connection-dot','side-dot']) $(id).className = 'dot '+(value?'online':'offline');
    $('connection-label').textContent = value?'로봇 서버 연결됨':'서버 연결 끊김';
    $('side-status').textContent = value?'로봇 서버 연결됨':'서버 연결 끊김';
    $('connection-notice').hidden = value;
    if (!value) {
      $('connection-notice').textContent = '로봇 서버에 연결되지 않았습니다. 전원과 같은 Wi-Fi 연결을 확인해주세요. 실시간 수치는 표시하지 않습니다.';
      $('state-pill').textContent = '연결 끊김';
      $('state-pill').classList.add('error');
      $('state-title').textContent = '로봇과의 연결을 확인해주세요.';
      $('state-detail').textContent = '서버에 다시 연결되면 현재 진행 상태를 불러옵니다.';
      for (const id of ['distance','remaining','direction','qr-value']) $(id).textContent = '—';
      $('qr-note').textContent = '서버 연결 확인 필요';
      $('qr-value').classList.remove('confirmed');
      $('distance-note').textContent = '실시간 측정값 없음';
      $('remaining-note').textContent = '연결 끊김';
      $('direction-note').textContent = '실시간 상태 확인 불가';
      $('timer-state').textContent = '연결 끊김';
      timerRunning = false;
      $('arm-speed').disabled = true;
      renderCamera(false);
    }
    buttons();
  }
  function buttons() {
    $('start').disabled = busy || stopping || !connected || !latest?.dashboard?.can_start;
    // Stop remains available on a dropped connection so the operator can retry.
    $('stop').disabled = stopping || (!latest && !connected);
  }
  function renderCamera(online) {
    const generation = latest?.dashboard?.camera_generation;
    if (typeof generation==='number') {
      if (cameraGeneration!==null && generation!==cameraGeneration) { cameraLoaded=false; cameraFailed=true; }
      cameraGeneration=generation;
    }
    if (!online && cameraWasOnline) cameraLoaded=false;
    cameraWasOnline=online;
    const healthy = online && cameraLoaded && !cameraFailed;
    $('camera-badge').className = 'live-badge'+(healthy?' online':'');
    $('camera-badge').lastChild.textContent = healthy?' LIVE':' 영상 대기';
    $('camera-dot').className = 'dot '+(healthy?'online':'muted');
    $('camera-note').textContent = healthy?'실시간 영상 수신 중':'카메라 영상 자동 재연결 중';
    $('camera-placeholder').hidden = healthy && cameraLoaded;
    if (connected && (!cameraLoaded || cameraFailed) && Date.now()-cameraRetryAt > 5000) connectCamera();
  }
  function connectCamera() {
    cameraRetryAt = Date.now();
    cameraFailed = false;
    $('camera').src = '/video_feed?dashboard='+cameraRetryAt;
  }
  $('camera').addEventListener('load',() => {
    cameraLoaded = true; cameraFailed = false;
    if (connected) renderCamera(Boolean(latest?.dashboard?.camera_online));
  });
  $('camera').addEventListener('error',() => {
    cameraLoaded = false; cameraFailed = true;
    $('camera-placeholder').hidden = false;
    $('camera-placeholder').querySelector('strong').textContent = '카메라 영상 연결 확인 필요';
    renderCamera(false);
  });
  function speedText(value) { return typeof value==='number'?`${Math.round(value)}°/s`:'—'; }
  function renderArmSpeed(speed) {
    const input = $('arm-speed');
    input.disabled = !connected || !speed || armSpeedSaving;
    if (!speed) { $('arm-speed-note').textContent = '속도 설정을 불러올 수 없습니다'; return; }
    if (!armSpeedEditing && !armSpeedSaving) {
      input.value = String(speed.percent);
      $('arm-speed-value').textContent = String(speed.percent);
    }
    $('arm-speed-note').textContent = `자세 이동 ${speedText(speed.move_deg_per_second)} · 운반 ${speedText(speed.carry_deg_per_second)} · 버튼 ${speedText(speed.jog_deg_per_second)}`;
  }
  function renderSteps(data, info, direction) {
    const state = info.state;
    const complete = state === 'COMPLETE';
    let done = direction?1:0, active = info.started?1:0;
    if (direction) active = 2;
    if (data.target_stop_latched) { done = 2; active = 3; }
    if ((data.arm?.completed_transfers || 0) >= 1) { done = 3; active = 4; }
    if (complete) { done = 5; active = 0; }
    const preparing = ['ARM_WAIT','QR_TURNING','QR_VIEW_READY'].includes(state);
    const blocked = state === 'STOPPED_WAIT' || preparing;
    const failed = state === 'ERROR' || state === 'CANCELLED';
    document.querySelectorAll('[data-stage]').forEach(item => {
      const stage = Number(item.dataset.stage);
      const isDone = stage <= done, isActive = stage === active && !blocked;
      item.className = isDone?'done':isActive?(failed?'failed':'active'):'';
      item.querySelector('.step-number').textContent = isDone?'✓':String(stage).padStart(2,'0');
      item.querySelector('.step-status').textContent = isDone?'완료':(stage===3 && blocked)?(state==='QR_VIEW_READY'?'QR 확인 대기':preparing?'QR 확인 준비 중':'팔 회전 대기'):isActive?(failed?'중단':'진행 중'):'대기';
    });
    $('step-counter').textContent = `${done} / 5`;
    if (direction) {
      const first = direction==='LEFT'?'왼쪽':'오른쪽', other = direction==='LEFT'?'오른쪽':'왼쪽';
      $('first-transfer').textContent = `${first} 물체 → ${other} 빈 상자`;
      $('second-transfer').textContent = `${other} 물체 → ${first} 빈 상자`;
    } else {
      $('first-transfer').textContent = 'QR 방향 → 반대편 빈 상자';
      $('second-transfer').textContent = '반대편 → 처음 방향의 빈 상자';
    }
  }
  function render(data) {
    if (!data.dashboard) throw new Error('미션 컨트롤 API를 사용할 수 없습니다');
    if (latest && data.dashboard.session_id === latest.dashboard.session_id && data.dashboard.control_revision < latest.dashboard.control_revision) return;
    latest = data;
    const info = data.dashboard, state = info.state;
    const direction = ['LEFT','RIGHT'].includes(data.first_direction)?data.first_direction:null;
    if (lastAttempt !== info.attempt || lastSession !== info.session_id) {
      if (lastSession!==null && lastSession!==info.session_id) { cameraLoaded=false; cameraFailed=true; cameraGeneration=null; }
      startDistance = null; lastAttempt = info.attempt; lastSession = info.session_id;
    }
    const copy = COPY[state] || ['진행 중',state,'현재 미션 상태를 확인하고 있습니다.'];
    $('state-pill').textContent = copy[0];
    $('state-pill').classList.toggle('error',['ERROR','CANCELLED'].includes(state));
    $('state-title').textContent = copy[1];
    $('state-detail').textContent = data.error && state==='ERROR'?`오류: ${data.error}`:copy[2];
    if (state==='READY' && !info.camera_online) {
      $('state-title').textContent = '카메라 연결을 기다리고 있어요.';
      $('state-detail').textContent = '영상이 준비되면 미션 시작 버튼이 활성화됩니다.';
    }
    $('attempt').textContent = info.attempt?`MISSION ${String(info.attempt).padStart(2,'0')}`:'MISSION —';
    $('start-label').textContent = busy?'처리 중…':info.started || state==='CANCELLED'?'새 미션 시작':'미션 시작';
    $('qr-value').textContent = direction?'인식 완료':info.started?'인식 대기':'시작 대기';
    $('qr-value').classList.toggle('confirmed',Boolean(direction));
    $('qr-note').textContent = direction?'출발 QR 방향이 확정됐어요':info.started?'LEFT 또는 RIGHT QR을 보여주세요':'시작 후 QR을 인식합니다';
    $('direction').textContent = direction || '—';
    $('direction').className = 'metric-value direction'+(direction?' '+direction.toLowerCase():'');
    $('direction-note').textContent = direction?`${direction==='LEFT'?'왼쪽':'오른쪽'} PICK·PLACE QR 확인 준비`:'LEFT / RIGHT QR 대기';
    $('camera-direction').textContent = direction?`QR ${direction} ✓`:'QR 대기';
    const distance = typeof info.distance_cm==='number' && Number.isFinite(info.distance_cm)?info.distance_cm:null;
    $('distance').textContent = distance===null?'—':distance.toFixed(1);
    $('remaining').textContent = distance===null?'—':Number(info.remaining_cm).toFixed(1);
    $('distance-note').textContent = info.distance_source==='FINAL'?'정지 시 측정값 · 팔 동작 중 고정':distance!==null?'초음파 센서로 측정한 거리':'유효한 초음파 측정값 대기';
    $('remaining-note').textContent = data.target_stop_latched?'도착 완료':distance===null?'측정 대기':'접근 거리';
    $('target').textContent = Number(info.target_distance_cm).toString();
    $('track-target').textContent = `${info.target_distance_cm} cm`;
    $('step-distance').textContent = `모니터 앞 ${info.target_distance_cm} cm에 정지`;
    if (info.started && distance!==null && startDistance===null) startDistance = distance;
    let progress = 0;
    if (data.target_stop_latched) progress = 100;
    else if (info.started && distance!==null && startDistance>info.target_distance_cm) progress = Math.max(0,Math.min(100,100*(startDistance-distance)/(startDistance-info.target_distance_cm)));
    $('track-fill').style.width = `${progress}%`;
    $('track-car').style.left = `${progress}%`;
    $('approach-progress').setAttribute('aria-valuenow',String(Math.round(progress)));
    $('track-label').textContent = data.target_stop_latched?'정지선 도착':!info.started?'출발 대기':distance===null?'측정 대기':`남은 거리 ${Number(info.remaining_cm).toFixed(1)} cm`;
    $('arm-note').textContent = info.arm_mode==='qr_prepare_only'?'정지 후 BASE만 좌우 90° 회전 · QR을 볼 준비 후 대기합니다.':!info.arm_enabled?'팔 자동 동작 꺼짐 · 이번 미션은 주행·정지까지 실행합니다.':!info.arm_calibrated?'팔 자세 보정 미확인 · 주행·정지까지만 실행합니다.':'팔 자동 동작 켜짐 · 저장한 자세로 두 물체를 운반합니다.';
    renderSteps(data,info,direction);
    elapsed = info.elapsed_seconds || 0; timerAt = performance.now();
    timerRunning = info.started && !info.finished && !TERMINAL.has(state);
    $('timer-state').textContent = state==='QR_VIEW_READY'?'준비 완료 · 대기':info.finished?(state==='COMPLETE'?'기록 종료':'정지됨'):info.started?'측정 중':'시작 대기';
    setConnected(true);
    renderArmSpeed(info.arm_speed);
    renderCamera(Boolean(info.camera_online));
  }
  async function poll() {
    try { render(await request('/mission/status')); }
    catch (error) { setConnected(false); }
    finally { setTimeout(poll,500); }
  }
  async function start() {
    if (busy || stopping || !latest?.dashboard?.can_start) return;
    busy = true; buttons(); feedback('새 미션을 준비하고 있습니다…');
    try {
      const response = await request('/dashboard/start','POST',{expected_revision:latest.dashboard.control_revision,session_id:latest.dashboard.session_id});
      if (!stopping && response.dashboard.session_id === latest.dashboard.session_id && response.dashboard.control_revision >= latest.dashboard.control_revision) {
        render(response); feedback('미션을 시작했습니다. QR을 보여주세요.');
      }
    }
    catch (error) { feedback(`시작 요청 확인 필요: ${error.message}. 현재 상태를 확인한 뒤 다시 눌러주세요.`,true); }
    finally { busy = false; buttons(); }
  }
  $('start').addEventListener('click',() => {
    if (latest?.dashboard?.started || latest?.dashboard?.state==='CANCELLED') $('restart-dialog').showModal();
    else start();
  });
  $('restart-dialog').addEventListener('close',() => { if ($('restart-dialog').returnValue==='start') start(); });
  $('arm-speed').addEventListener('input',() => {
    armSpeedEditing = true;
    $('arm-speed-value').textContent = $('arm-speed').value;
  });
  $('arm-speed').addEventListener('change',async() => {
    const percent = Number($('arm-speed').value);
    armSpeedSaving = true; $('arm-speed').disabled = true;
    try {
      const response = await request('/dashboard/arm-speed','POST',{percent});
      if (latest?.dashboard) latest.dashboard.arm_speed = response.arm_speed;
      feedback(`로봇팔 속도를 ${percent}%로 바꿨습니다. 다음 팔 동작부터 적용됩니다.`);
    }
    catch (error) { feedback(`팔 속도 변경 실패: ${error.message}`,true); }
    finally {
      armSpeedEditing = false; armSpeedSaving = false;
      renderArmSpeed(latest?.dashboard?.arm_speed);
    }
  });
  $('stop').addEventListener('click',async() => {
    if (stopping) return;
    stopping = true; buttons(); feedback('전체 동작을 중지하고 있습니다…');
    try { render(await request('/mission/stop','POST')); feedback('중지 요청이 처리됐습니다. 팔은 현재 자세를 유지합니다.'); }
    catch (error) { feedback(`중지 확인 불가: ${error.message}. 실제 로봇을 확인하고 필요하면 전원을 직접 차단하세요.`,true); }
    finally { stopping = false; buttons(); }
  });
  setInterval(() => {
    const seconds = elapsed+(timerRunning && connected?(performance.now()-timerAt)/1000:0);
    const minutes = Math.floor(seconds/60), rest = seconds-minutes*60;
    $('timer').textContent = `${String(minutes).padStart(2,'0')}:${rest.toFixed(1).padStart(4,'0')}`;
  },100);
  connectCamera();
  poll();
})();
