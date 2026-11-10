// ArduHumanoid pages. They only talk to /api/* (python/api.py), which turns every request into MAVLink.
'use strict';

const $ = (id) => document.getElementById(id);
const JOINT_LABEL = {
  left_hip_yaw: 'Anca sx · imbardata', left_hip_roll: 'Anca sx · rollio', left_hip_pitch: 'Anca sx · beccheggio',
  left_knee: 'Ginocchio sx', left_ankle: 'Caviglia sx',
  right_hip_yaw: 'Anca dx · imbardata', right_hip_roll: 'Anca dx · rollio', right_hip_pitch: 'Anca dx · beccheggio',
  right_knee: 'Ginocchio dx', right_ankle: 'Caviglia dx',
  neck_pitch: 'Collo', head_pitch: 'Testa · beccheggio', head_yaw: 'Testa · imbardata', head_roll: 'Testa · rollio',
  mouth: 'Bocca',
};
const POLICY_LABEL = { walk: 'Camminata', getup: 'Rialzata', roller: 'Pattini', pick: 'Raccolta', roulade: 'Capriola',
  sitstand: 'Seduta', kick_left: 'Calcio sinistro', kick_right: 'Calcio destro' };
const SKILL_LABEL = { pick: 'Raccolta', roulade: 'Capriola', sit: 'Seduto', stand: 'In piedi', kick_left: 'Calcio sinistro', kick_right: 'Calcio destro' };
const GETUP_LABEL = { idle: 'In piedi', falling: 'Caduta rilevata', active: 'Si sta rialzando', recovered: 'Rialzato' };
const ENCODER_CENTER = 2047;

let status = null;          // last /api/status
let statusAge = 0;          // failed polls in a row
const twist = { vx: 0, vy: 0, wz: 0 };
const keys = new Set();
const head = [0, 0, 0, 0];

// ───────────────────────────── helpers
function say(text, kind = '') {
  const m = $('message');
  m.textContent = text;
  m.className = kind;
  clearTimeout(say.timer);
  if (text) say.timer = setTimeout(() => { m.textContent = ''; m.className = ''; }, 6000);
}

async function post(path, body = {}, quiet = false) {
  try {
    const r = await fetch('/api/' + path, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
    const data = await r.json().catch(() => ({}));
    if (!r.ok) throw new Error(data.detail || `errore ${r.status}`);
    return data;
  } catch (e) {
    if (!quiet) say(e.message, 'error');
    throw e;
  }
}

const fmt = (v, digits = 1) => (v === null || v === undefined || Number.isNaN(v)) ? '—' : Number(v).toFixed(digits);
const vec = (v, digits = 2) => v ? v.map((x) => fmt(x, digits).padStart(digits + 4)).join(' ') : '—';
function setText(id, text, cls) {
  const e = $(id);
  if (e.textContent !== text) e.textContent = text;
  if (cls !== undefined) e.className = cls;
}
function card(id, level, title, sub) {
  $(id).parentElement.className = level;
  setText(id, title);
  setText(id + '-sub', sub);
}

// ───────────────────────────── tabs
document.querySelectorAll('#tabs button').forEach((b) => b.addEventListener('click', () => showTab(b.dataset.tab)));
function showTab(name) {
  document.querySelectorAll('#tabs button').forEach((b) => b.classList.toggle('on', b.dataset.tab === name));
  document.querySelectorAll('.tab').forEach((s) => s.classList.toggle('on', s.id === 'tab-' + name));
  history.replaceState(null, '', '#' + name);
}
if (location.hash && $('tab-' + location.hash.slice(1))) showTab(location.hash.slice(1));

// ───────────────────────────── status
async function poll() {
  try {
    const r = await fetch('/api/status', { cache: 'no-store' });
    if (!r.ok) throw new Error(`stato ${r.status}`);
    status = await r.json();
    statusAge = 0;
  } catch (e) {
    statusAge += 1;
  }
  render();
  setTimeout(poll, 200);
}

function render() {
  const alerts = [];
  if (!status || statusAge > 5) {
    alerts.push(['bad', "La pagina non riceve più dati dall'app sulla scheda."]);
    renderAlerts(alerts);
    return;
  }
  const s = status, link = s.link, ap = s.autopilot, bus = ap.bus, robot = ap.robot;
  const armed = !!ap.armed;
  const busHas = (flag) => !!bus && bus.status.includes(flag);
  setText('clock', s.time);

  // header
  setText('armed-pill', !ap.connected ? 'NON COLLEGATO' : armed ? 'ARMATO' : 'DISARMATO', 'pill ' + (!ap.connected ? '' : armed ? 'armed' : 'safe'));
  setText('arm-state', armed ? 'ARMATO' : 'DISARMATO', 'tag ' + (armed ? 'armed' : 'ok'));

  // cards
  const proc = s.process;
  card('c-ap', !proc.running || !ap.connected ? 'bad' : 'ok',
    !proc.running ? 'Fermo' : !ap.connected ? 'Senza MAVLink' : armed ? 'Armato' : 'Disarmato',
    !proc.running ? (proc.error || 'ardurover non è in esecuzione') : `ardurover · modo ${ap.mode || '—'}`);
  const slow = link.mcu_link && link.state_hz < 90;
  card('c-mcu', !link.mcu_link ? 'bad' : slow ? 'warn' : 'ok', link.mcu_link ? `${fmt(link.state_hz, 0)} Hz` : 'Non risponde',
    link.mcu_link ? (slow ? 'stato ridotto: router a 115200' : 'stato dallo sketch') : 'sketch o router fermi');
  const imu = link.imu || {};
  card('c-imu', link.imu_ok ? (s.tilt_deg > 60 ? 'warn' : 'ok') : 'bad', link.imu_ok ? (imu.kind || 'OK') : 'Assente',
    link.imu_ok ? `inclinazione ${fmt(s.tilt_deg, 0)}°` : (imu.spi_who_am_i ? `SPI risponde ${imu.spi_who_am_i}` : 'nessun sensore trovato'));
  const answering = (link.servos_answering || []).length;
  const volts = (ap.joints || []).map((j) => j.voltage).filter((v) => v !== null && v > 0);
  card('c-servo', answering === 15 ? 'ok' : answering ? 'warn' : 'bad', `${answering} / 15`,
    volts.length ? `${fmt(Math.min(...volts))}–${fmt(Math.max(...volts))} V · coppia ${link.torque ? 'ON' : 'off'}` : `nessuna tensione letta · coppia ${link.torque ? 'ON' : 'off'}`);
  const task = ap.policy_task || {};
  card('c-policy', robot ? (armed ? 'ok' : '') : 'bad', robot ? (POLICY_LABEL[robot.policy_name] || robot.policy_name) : '—',
    robot ? `catena ${robot.chain === 'xduck' ? 'XDuck' : 'Standard'} · ${fmt(task.hz, 0)} Hz` : 'nessuna telemetria');

  // alerts
  if (!proc.running) alerts.push(['bad', `ardurover non è in esecuzione. ${proc.error || ''}`]);
  else if (!ap.connected) alerts.push(['bad', "ardurover è in esecuzione ma non risponde su MAVLink."]);
  if (!link.mcu_link) alerts.push(['bad', 'Lo sketch sul microcontrollore non manda lo stato: nessun dato da IMU e servo.']);
  else {
    if (slow) alerts.push(['', `Il micro manda lo stato a ${fmt(link.state_hz, 0)} Hz invece di 100: il router della scheda è a 115200 baud. Per camminare serve «sudo bash tools/router-2m.sh» e lo sketch a piena velocità.`]);
    if (!link.imu_ok) alerts.push(['bad', "L'IMU non risponde: il microcontrollore non dà coppia ai servo."]);
    else if (s.tilt_deg > 60) alerts.push(['', `Per l'IMU il robot è inclinato di ${fmt(s.tilt_deg, 0)}°: da armato partirebbe la rialzata. Se il robot è in piedi, il sensore è montato con un altro orientamento.`]);
    if (answering < 15) alerts.push([answering ? '' : 'bad', answering ? `Rispondono ${answering} servo su 15 (mancano gli ID ${link.servos_missing.join(', ')}).` : 'Nessun servo risponde sul bus: alimentazione o collegamento del bus (D0/D1).']);
    if (link.timed_out) alerts.push(['bad', 'Il microcontrollore ha tolto coppia da solo: comandi mancanti per troppo tempo.']);
  }
  if (bus && (bus.refused || bus.crc_errors)) alerts.push(['', `Collegamento autopilota–micro: ${bus.refused} comandi rifiutati, ${bus.crc_errors} errori CRC.`]);
  renderAlerts(alerts);

  // IMU
  setText('imu-acc', link.acc_g ? vec(link.acc_g, 3) + '  g' : '—');
  setText('imu-gyro', link.gyro_dps ? vec(link.gyro_dps, 1) + '  °/s' : '—');
  setText('imu-tilt', s.tilt_deg === undefined ? '—' : `${fmt(s.tilt_deg, 1)}°`, s.tilt_deg > 60 ? 'bad' : '');
  setText('imu-att', ap.attitude ? `rollio ${fmt(ap.attitude.roll)}°  beccheggio ${fmt(ap.attitude.pitch)}°  imbardata ${fmt(ap.attitude.yaw)}°` : '—');
  setText('imu-kind', imu.kind ? `${imu.kind}${imu.address ? ' · ' + imu.address : ''}` : '—');
  const dot = $('level-dot');
  if (link.imu_ok && link.acc_g) {
    // top view: x of the sensor up, y to the right; the dot is where gravity points
    const clamp = (v) => Math.max(-1, Math.min(1, v));
    dot.setAttribute('cx', (clamp(link.acc_g[1]) * 50).toFixed(1));
    dot.setAttribute('cy', (-clamp(link.acc_g[0]) * 50).toFixed(1));
    dot.setAttribute('class', 'dot');
  } else dot.setAttribute('class', 'dot off');

  // links
  setText('l-proc', proc.running ? `in esecuzione (avvii: ${proc.starts})` : (proc.error || 'fermo'), proc.running ? '' : 'bad');
  setText('l-mav', ap.connected ? `collegato · modo ${ap.mode || '—'}` : (ap.error || 'nessun heartbeat'), ap.connected ? '' : 'bad');
  setText('l-gcs', proc.mavlink_tcp_port ? `MAVLink 2 su TCP ${location.hostname}:${proc.mavlink_tcp_port}` : 'non attiva');
  setText('l-state', `${fmt(link.state_hz)} Hz al relay · ${bus ? fmt(bus.state_hz) : '—'} Hz all'autopilota`);
  setText('l-cmd', `${fmt(link.command_hz)} Hz · ultimo: ${link.last_command_mode || '—'}`);
  const c = link.control;
  setText('l-loop', c ? `periodo ${(c.period_us / 1000).toFixed(2)} ms · lavoro ${(c.work_us / 1000).toFixed(2)} ms · ${c.overruns} sforamenti` : '—');
  setText('l-task', task.hz == null ? '—' : `${fmt(task.hz)} Hz · intervallo più lungo ${fmt(task.longest_gap_ms)} ms · inferenza ${fmt(task.inference_ms, 2)} ms`);
  setText('l-err', `CRC ${link.crc_errors} · comandi rifiutati ${c ? c.invalid_commands : '—'} · invii falliti ${link.counts.send_errors}`);

  renderJoints(ap.joints || [], busHas('calibration'));
  renderTexts(ap.texts || []);
  renderControl(s, armed, busHas);
}

function renderAlerts(alerts) {
  const html = alerts.map(([cls, text]) => `<li class="${cls}">${text}</li>`).join('');
  if ($('alerts').innerHTML !== html) $('alerts').innerHTML = html;
}

function renderJoints(joints, inCal) {
  const rows = joints.map((j) => `<tr class="${j.present ? '' : 'missing'}"><td>${JOINT_LABEL[j.name] || j.name}</td><td>${j.id}</td>
    <td><span class="tag ${j.present ? 'ok' : ''}">${j.present ? 'presente' : 'assente'}</span></td>
    <td class="num">${j.present ? fmt(j.angle) : '—'}</td><td class="num">${fmt(j.target)}</td>
    <td class="num">${j.present && j.raw ? j.raw : '—'}</td><td class="num">${j.voltage ? fmt(j.voltage) : '—'}</td></tr>`).join('');
  if ($('joints').innerHTML !== rows) $('joints').innerHTML = rows;

  // calibration rows are built once and then updated in place, so a click is never lost to a redraw
  const body = $('cal-rows');
  if (body.children.length !== joints.length) {
    body.innerHTML = joints.map((j) => `<tr><td>${JOINT_LABEL[j.name] || j.name}</td><td>${j.id}</td>
      <td class="num"></td><td class="num"></td><td><span class="tag ok" hidden>centrato</span><button data-center="${j.slot}">Centra</button></td></tr>`).join('');
  }
  joints.forEach((j, i) => {
    const row = body.children[i], has = inCal && j.present && j.raw > 0;
    const cells = row.children;
    row.className = j.present ? '' : 'missing';
    const raw = has ? String(j.raw) : '—', off = has ? String(j.raw - ENCODER_CENTER) : '—';
    if (cells[2].textContent !== raw) cells[2].textContent = raw;
    if (cells[3].textContent !== off) cells[3].textContent = off;
    cells[4].firstElementChild.hidden = !j.held;
    cells[4].lastElementChild.hidden = j.held;
    cells[4].lastElementChild.disabled = !has;
  });
}

function renderTexts(texts) {
  const html = texts.length ? texts.slice().reverse().map((t) => `<li><time>${t.time}</time>${t.text.replace(/</g, '&lt;')}</li>`).join('') : '<li>Nessun messaggio.</li>';
  if ($('texts').innerHTML !== html) $('texts').innerHTML = html;
}

function renderControl(s, armed, busHas) {
  const ap = s.autopilot, p = ap.params || {}, robot = ap.robot;
  const ready = ap.connected;
  $('arm').disabled = !ready || armed;
  $('disarm').disabled = !ready || !armed;
  $('torque-off').disabled = $('torque-on').disabled = !ready || armed;

  const disPose = p.NNM_DIS_POSE === 1;
  if (document.activeElement !== $('dis-pose')) $('dis-pose').checked = disPose;
  $('dis-pose').disabled = !ready || p.NNM_DIS_POSE === undefined;
  const released = busHas('torque_released');
  setText('dis-pose-state', p.NNM_DIS_POSE === undefined ? '—' : !disPose ? 'servo liberi' : released ? 'rilasciati su richiesta' : 'attiva',
    'tag ' + (disPose && !released ? 'warn' : ''));

  for (const [group, name] of [['chain', 'NNM_DEPLOY'], ['policy', 'NNM_POLICY']]) {
    document.querySelectorAll(`#${group} button`).forEach((b) => {
      b.classList.toggle('on', Number(b.dataset.value) === p[name]);
      b.disabled = !ready || (group === 'chain' && armed);
    });
  }
  setText('chain-state', robot ? (robot.chain === 'xduck' ? 'XDuck in uso' : 'Standard in uso') : '—');
  setText('policy-state', p.NNM_POLICY === undefined ? '—' : p.NNM_POLICY === 2 ? 'pattini' : 'camminata');
  setText('run-policy', robot ? (POLICY_LABEL[robot.policy_name] || robot.policy_name) + (armed ? '' : ' (ferma: disarmato)') : '—');
  setText('run-skill', robot ? (robot.skill ? SKILL_LABEL[robot.skill] : 'nessuna') : '—');
  setText('run-getup', robot ? (GETUP_LABEL[robot.getup] || robot.getup) : '—', robot && robot.getup !== 'idle' ? 'bad' : '');

  const sitting = robot && robot.skill === 'sit';
  document.querySelectorAll('#skills button').forEach((b) => {
    if (b.id === 'sitstand') {
      b.textContent = sitting ? 'Alzati' : 'Siediti';
      b.dataset.skill = sitting ? 'stand' : 'sit';
      b.disabled = !armed || (robot && robot.skill && !sitting) || (robot && robot.getup !== 'idle');
    } else b.disabled = !armed || !robot || !!robot.skill || robot.getup !== 'idle';
  });

  const vxMax = p.NNM_VX_MAX ?? 0, vyMax = p.NNM_VY_MAX ?? 0, wzMax = p.NNM_WZ_MAX ?? 0;
  setText('vx', fmt(twist.vx * vxMax, 2));
  setText('vy', fmt(twist.vy * vyMax, 2));
  setText('wz', fmt(twist.wz * wzMax, 2));
  setText('limits', p.NNM_VX_MAX === undefined ? '' : `Limiti: vx ±${fmt(vxMax, 2)}, vy ±${fmt(vyMax, 2)} m/s, ωz ±${fmt(wzMax, 2)} rad/s.` + (armed ? '' : ' Il movimento ha effetto solo da armato.'));

  const inCal = busHas('calibration'), inServo = busHas('servo_setup');
  setText('cal-state', inCal ? 'sessione aperta' : 'chiusa', 'tag ' + (inCal ? 'warn' : ''));
  setText('servo-state', inServo ? 'sessione aperta' : 'chiusa', 'tag ' + (inServo ? 'warn' : ''));
  $('cal-enter').disabled = !ready || armed || inCal;
  $('cal-exit').disabled = !inCal;
  $('servo-enter').disabled = !ready || armed || inServo;
  $('servo-exit').disabled = !inServo;
  document.querySelectorAll('[data-servo]').forEach((b) => { b.disabled = !inServo; });
  session.cal = inCal;
  session.servo = inServo;
}

// ───────────────────────────── arming and torque
$('arm').addEventListener('click', async () => {
  const s = status, link = s.link;
  const lines = ['Armare il robot?', '', 'La policy comincia a comandare i servo: il robot deve essere a terra, libero di muoversi, con spazio intorno.'];
  if (s.tilt_deg > 60) lines.push('', `ATTENZIONE: per l'IMU il robot è inclinato di ${fmt(s.tilt_deg, 0)}°, quindi partirà la rialzata.`);
  if ((link.servos_answering || []).length < 15 || !link.imu_ok) lines.push('', 'Servo o IMU non rispondono: la policy girerà senza dare coppia.');
  if (!confirm(lines.join('\n'))) return;
  const r = await post('arm', { armed: true });
  say(r.note || 'Armato.', r.note ? '' : 'good');
});
$('disarm').addEventListener('click', async () => { await post('arm', { armed: false }); say('Disarmato.', 'good'); });

async function stop() {
  twist.vx = twist.vy = twist.wz = 0;
  try { await post('stop'); say('Disarmato, servo rilasciati.', 'good'); } catch (e) { /* shown by post */ }
}
$('stop').addEventListener('click', stop);

$('torque-off').addEventListener('click', async () => { await post('torque', { on: false }); say('Servo rilasciati.', 'good'); });
$('torque-on').addEventListener('click', async () => {
  if (!confirm('I servo vanno in coppia e portano il robot nella posa di equilibrio in un secondo. Procedo?')) return;
  await post('torque', { on: true });
  say('Posa di equilibrio richiesta.', 'good');
});
$('dis-pose').addEventListener('change', async (e) => {
  const on = e.target.checked;
  if (on && !confirm('Da disarmato i servo resteranno in coppia nella posa di equilibrio, anche ai prossimi avvii. Procedo?')) { e.target.checked = false; return; }
  try { await post('param', { name: 'NNM_DIS_POSE', value: on ? 1 : 0 }); say(on ? 'Posa di equilibrio da disarmato: attiva.' : 'Da disarmato i servo restano liberi.', 'good'); }
  catch (err) { e.target.checked = !on; }
});

for (const [group, name, label] of [['chain', 'NNM_DEPLOY', 'Catena'], ['policy', 'NNM_POLICY', 'Policy']]) {
  document.querySelectorAll(`#${group} button`).forEach((b) => b.addEventListener('click', async () => {
    await post('param', { name, value: Number(b.dataset.value) });
    say(`${label}: ${b.textContent}.`, 'good');
  }));
}

document.querySelectorAll('#skills button').forEach((b) => b.addEventListener('click', async () => {
  await post('skill', { name: b.dataset.skill });
  say(`Skill avviata: ${b.textContent}.`, 'good');
}));

// ───────────────────────────── motion: two pads and the keyboard
function joystick(id, onMove) {
  const pad = $(id), knob = pad.querySelector('.knob');
  let pointer = null;
  const place = (x, y) => { knob.style.left = `${33 + x * 33}%`; knob.style.top = `${33 - y * 33}%`; };
  const move = (e) => {
    const r = pad.getBoundingClientRect();
    let x = ((e.clientX - r.left) / r.width - 0.5) * 2, y = (0.5 - (e.clientY - r.top) / r.height) * 2;
    const n = Math.hypot(x, y);
    if (n > 1) { x /= n; y /= n; }
    place(x, y);
    onMove(x, y);
  };
  const release = () => { pointer = null; pad.classList.remove('active'); place(0, 0); onMove(0, 0); };
  pad.addEventListener('pointerdown', (e) => { pointer = e.pointerId; pad.setPointerCapture(pointer); pad.classList.add('active'); move(e); });
  pad.addEventListener('pointermove', (e) => { if (e.pointerId === pointer) move(e); });
  pad.addEventListener('pointerup', release);
  pad.addEventListener('pointercancel', release);
}
const pads = { vx: 0, vy: 0, wz: 0 };
joystick('joy-left', (x, y) => { pads.vx = y; pads.vy = -x; });      // up = forward, left = +vy
joystick('joy-right', (x) => { pads.wz = -x; });                      // left = turn left

const typing = () => ['INPUT', 'TEXTAREA', 'SELECT'].includes(document.activeElement.tagName) && document.activeElement.type !== 'range' && document.activeElement.type !== 'checkbox';
window.addEventListener('keydown', (e) => {
  if (typing()) return;
  if (e.code === 'Space') { e.preventDefault(); stop(); return; }
  if ('wasdqe'.includes(e.key.toLowerCase()) && e.key.length === 1) keys.add(e.key.toLowerCase());
});
window.addEventListener('keyup', (e) => keys.delete(e.key.toLowerCase()));
window.addEventListener('blur', () => keys.clear());

let moving = false;
setInterval(() => {
  const k = (plus, minus) => (keys.has(plus) ? 1 : 0) - (keys.has(minus) ? 1 : 0);
  twist.vx = pads.vx || k('w', 's');
  twist.vy = pads.vy || k('a', 'd');
  twist.wz = pads.wz || k('q', 'e');
  const active = twist.vx !== 0 || twist.vy !== 0 || twist.wz !== 0;
  // repeated while held; one last zero when released. The gateway stops by itself after 0.5 s of silence.
  if (active || moving) post('motion', twist, true).catch(() => {});
  moving = active;
}, 100);

// ───────────────────────────── head and mouth
['Collo', 'Testa · beccheggio', 'Testa · imbardata', 'Testa · rollio'].forEach((label, i) => {
  $('head').insertAdjacentHTML('beforeend', `<label class="slider"><span>${label}</span><input type="range" data-head="${i}" min="-1" max="1" step="0.02" value="0"></label>`);
});
let headTimer = null;
function sendHead() {
  clearTimeout(headTimer);
  headTimer = setTimeout(() => post('head', { head, mouth: Number($('mouth').value) }).catch(() => {}), 60);
}
document.querySelectorAll('[data-head]').forEach((input) => input.addEventListener('input', () => { head[Number(input.dataset.head)] = Number(input.value); sendHead(); }));
$('mouth').addEventListener('input', () => { setText('mouth-value', $('mouth').value); sendHead(); });
$('head-zero').addEventListener('click', () => {
  head.fill(0);
  document.querySelectorAll('[data-head]').forEach((i) => { i.value = 0; });
  $('mouth').value = 0;
  setText('mouth-value', '0');
  sendHead();
});

// ───────────────────────────── calibration and servo setup sessions
// The autopilot closes a session that nobody keeps alive for 2 s: this page does it while it is open.
const session = { cal: false, servo: false };
setInterval(() => { if (session.cal || session.servo) post('service', { action: 'heartbeat' }, true).catch(() => {}); }, 600);

async function service(body, what) {
  const r = await post('service', body);
  if (!r.ok) { say(`${what}: rifiutato${r.source === 'autopilot' ? " dall'autopilota" : ' dal servo'}.`, 'error'); throw new Error('refused'); }
  return r;
}
$('cal-enter').addEventListener('click', async () => {
  if (!confirm('I servo vengono rilasciati: sostieni il robot. Procedo?')) return;
  await service({ action: 'cal_enter' }, 'Calibrazione');
  say('Calibrazione aperta: i servo sono liberi.', 'good');
});
$('cal-exit').addEventListener('click', async () => { await service({ action: 'cal_exit' }, 'Chiusura'); say('Calibrazione chiusa.', 'good'); });
$('cal-rows').addEventListener('click', async (e) => {
  const slot = e.target.dataset.center;
  if (slot === undefined) return;
  const name = e.target.closest('tr').firstElementChild.textContent;
  if (!confirm(`Memorizzare la posizione attuale come centro di «${name}»?`)) return;
  await service({ action: 'cal_center', index: Number(slot) }, 'Centratura');
  say(`${name}: centro memorizzato.`, 'good');
});

$('servo-enter').addEventListener('click', async () => {
  if (!confirm('I servo vengono rilasciati: sostieni il robot. Procedo?')) return;
  await service({ action: 'servo_enter' }, 'Sessione servo');
  say('Sessione servo aperta.', 'good');
});
$('servo-exit').addEventListener('click', async () => { await service({ action: 'servo_exit' }, 'Chiusura'); say('Sessione servo chiusa.', 'good'); });
document.querySelectorAll('[data-servo]').forEach((b) => b.addEventListener('click', async () => {
  const action = b.dataset.servo;
  const body = { action, id: Number($('s-id').value) };
  if (action === 'set_id') {
    body.new_id = Number($('s-new').value);
    if (!confirm(`Cambiare l'ID del servo ${body.id} in ${body.new_id}? Deve esserci un solo servo con ID ${body.id} sul bus.`)) return;
  }
  if (action === 'set_gains') { body.kp = Number($('s-kp').value); body.kd = Number($('s-kd').value); }
  if (action === 'set_protection') body.enabled = b.dataset.enabled === '1';
  if (action === 'set_protection_current') body.current_a = Number($('s-amp').value);
  if (action === 'reset_mid' && !confirm(`Memorizzare la posizione attuale come centro del servo ${body.id}?`)) return;
  const out = $('servo-result');
  try {
    const r = await post('service', body);
    out.className = 'result' + (r.ok ? '' : ' bad');
    const lines = [`${b.textContent.trim()} · servo ${r.id}: ${r.ok ? 'riuscito' : 'NON riuscito (' + (r.source === 'autopilot' ? "rifiutato dall'autopilota" : 'nessuna risposta dal servo') + ')'}`];
    if (r.ok && (action === 'read' || action === 'set_gains' || action === 'reset_mid')) lines.push(`posizione ${r.raw_pos} · centro ${r.center} · Kp ${r.kp} · Kd ${r.kd}`);
    if (r.ok && action.startsWith('set_protection')) lines.push(`protezione ${r.protection ? 'ON' : 'OFF'} · corrente ${r.current_a} A`);
    out.textContent = lines.join('\n');
  } catch (e) {
    out.className = 'result bad';
    out.textContent = e.message;
  }
}));

poll();
