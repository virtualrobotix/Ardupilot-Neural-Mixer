// ArduDuck pages. They only talk to /api/* (python/api.py), which turns every request into MAVLink.
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
const jointName = (name) => t(JOINT_LABEL[name] || name);
const policyName = (name) => t(POLICY_LABEL[name] || name);
const chainName = (chain) => chain === 'xduck' ? 'ArduDuck' : 'Standard';   // 'xduck' is the name on the wire
// Text written by the app on the board, which is in Italian.
function serverText(text) {
  const missing = /^servo assenti \((\d+)\): /.exec(text || '');
  return missing ? t('servo assenti ({n}): la policy gira ma il microcontrollore non dà coppia', { n: missing[1] }) : t(text);
}
const ENCODER_CENTER = 2047;
const FALLBACK_SERVOS = 15;   // until the status has the joints: the robot has 15 bus slots
const configuredServos = (ap) => (ap.joints || []).length || FALLBACK_SERVOS;

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
    if (!r.ok) throw new Error(serverText(data.detail) || t('errore {n}', { n: r.status }));
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

// ───────────────────────────── theme: the system choice, unless the button was used
const savedTheme = localStorage.getItem('theme');
if (savedTheme) document.documentElement.dataset.theme = savedTheme;
$('theme').addEventListener('click', () => {
  const root = document.documentElement;
  const dark = root.dataset.theme ? root.dataset.theme === 'dark' : matchMedia('(prefers-color-scheme: dark)').matches;
  root.dataset.theme = dark ? 'light' : 'dark';
  localStorage.setItem('theme', root.dataset.theme);
});

// ───────────────────────────── language: static text by i18n.js, the rest is drawn again here
$('lang').value = lang;
$('lang').addEventListener('change', (e) => setLanguage(e.target.value));
translatePage();
languageListeners.push(() => {
  $('cal-rows').innerHTML = $('servo-rows').innerHTML = '';       // rebuilt by the next render
  document.querySelectorAll('#head label span').forEach((span, i) => { span.textContent = t(HEAD_LABEL[i]); });
  render();
});

// ───────────────────────────── tabs
document.querySelectorAll('#tabs button').forEach((b) => b.addEventListener('click', () => showTab(b.dataset.tab)));
document.querySelectorAll('#sim-mode button').forEach((b) => b.addEventListener('click', async () => {
  try {
    await post('sim', { mode: b.dataset.sim });
    say(t(b.dataset.sim === 'off' ? 'Ambiente virtuale spento.' : b.dataset.sim === 'simulate' ? 'Simulazione attiva: i motori reali sono spenti.' : 'Debug attivo: la scena segue i comandi del robot.'), 'good');
  } catch (e) { /* shown by post */ }
}));
$('sim-reset').addEventListener('click', async () => {
  const mode = (status && status.sim && status.sim.mode) || 'off';
  if (mode === 'off') return;
  try { await post('sim', { mode, reset: true }); say(t('Scena riportata in piedi.'), 'good'); } catch (e) { /* shown by post */ }
});
function showTab(name) {
  document.querySelectorAll('#tabs button').forEach((b) => b.classList.toggle('on', b.dataset.tab === name));
  document.querySelectorAll('.tab').forEach((s) => s.classList.toggle('on', s.id === 'tab-' + name));
  history.replaceState(null, '', '#' + name);
  if (name === 'rete' || name === 'info') pollNet();
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
    alerts.push(['bad', t("La pagina non riceve più dati dall'app sulla scheda.")]);
    renderAlerts(alerts);
    return;
  }
  const s = status, link = s.link, ap = s.autopilot, bus = ap.bus, robot = ap.robot;
  const armed = !!ap.armed;
  const busHas = (flag) => !!bus && bus.status.includes(flag);
  setText('clock', s.time);

  // header
  setText('armed-pill', t(!ap.connected ? 'NON COLLEGATO' : armed ? 'ARMATO' : 'DISARMATO'), 'pill ' + (!ap.connected ? '' : armed ? 'armed' : 'safe'));
  setText('arm-state', t(armed ? 'ARMATO' : 'DISARMATO'), 'tag ' + (armed ? 'armed' : 'ok'));

  // cards
  const proc = s.process;
  card('c-ap', !proc.running || !ap.connected ? 'bad' : 'ok',
    t(!proc.running ? 'Fermo' : !ap.connected ? 'Senza MAVLink' : armed ? 'Armato' : 'Disarmato'),
    !proc.running ? (proc.error || t('ardurover non è in esecuzione')) : t('ardurover · modo {mode}', { mode: ap.mode || '—' }));
  const slow = link.mcu_link && link.state_hz < 90;
  card('c-mcu', !link.mcu_link ? 'bad' : slow ? 'warn' : 'ok', link.mcu_link ? `${fmt(link.state_hz, 0)} Hz` : t('Non risponde'),
    t(link.mcu_link ? (slow ? 'stato ridotto: router a 115200' : 'stato dallo sketch') : 'sketch o router fermi'));
  const imu = link.imu || {};
  card('c-imu', link.imu_ok ? (s.tilt_deg > 60 ? 'warn' : 'ok') : 'bad', link.imu_ok ? (imu.kind || 'OK') : t('Assente'),
    link.imu_ok ? t('inclinazione {deg}°', { deg: fmt(s.tilt_deg, 0) }) : (imu.spi_who_am_i ? t('SPI risponde {id}', { id: imu.spi_who_am_i }) : t('nessun sensore trovato')));
  const answering = (link.servos_answering || []).length;
  const volts = (ap.joints || []).map((j) => j.voltage).filter((v) => v !== null && v > 0);
  const expected = configuredServos(ap);
  card('c-servo', answering === expected ? 'ok' : answering ? 'warn' : 'bad', `${answering} / ${expected}`,
    volts.length ? t('{min}–{max} V · coppia {state}', { min: fmt(Math.min(...volts)), max: fmt(Math.max(...volts)), state: link.torque ? 'ON' : 'off' })
      : t('nessuna tensione letta · coppia {state}', { state: link.torque ? 'ON' : 'off' }));
  const task = ap.policy_task || {};
  card('c-policy', robot ? (armed ? 'ok' : '') : 'bad', robot ? policyName(robot.policy_name) : '—',
    robot ? t('catena {chain} · {hz} Hz', { chain: chainName(robot.chain), hz: fmt(task.hz, 0) }) : t('nessuna telemetria'));

  // alerts
  if (!proc.running) alerts.push(['bad', t('ardurover non è in esecuzione. {error}', { error: proc.error || '' })]);
  else if (!ap.connected) alerts.push(['bad', t('ardurover è in esecuzione ma non risponde su MAVLink.')]);
  const sim = s.sim || { mode: 'off' };
  if (sim.mode === 'simulate') card('c-mcu', link.mcu_link ? 'ok' : '', link.mcu_link ? `${fmt(link.state_hz, 0)} Hz` : '—', t('stato da MuJoCo'));
  if (sim.mode === 'simulate') alerts.push(['', t('Simulazione MuJoCo: IMU e servo arrivano dalla scena. I motori reali sono spenti.')]);
  else if (sim.mode === 'debug') alerts.push(['', t('Debug MuJoCo: il robot reale comanda. La colonna Sim ripete gli stessi comandi nella scena.')]);
  if (sim.mode !== 'simulate' && !link.mcu_link) alerts.push(['bad', t('Lo sketch sul microcontrollore non manda lo stato: nessun dato da IMU e servo.')]);
  else if (sim.mode !== 'simulate') {
    if (slow) alerts.push(['', t('Il micro manda lo stato a {hz} Hz invece di 100: il router della scheda è a 115200 baud. Per camminare serve «sudo bash tools/router-2m.sh» e lo sketch a piena velocità.', { hz: fmt(link.state_hz, 0) })]);
    if (!link.imu_ok) alerts.push(['bad', t("L'IMU non risponde: il microcontrollore non dà coppia ai servo.")]);
    else if (s.tilt_deg > 60) alerts.push(['', t("Per l'IMU il robot è inclinato di {deg}°: da armato partirebbe la rialzata. Se il robot è in piedi, il sensore è montato con un altro orientamento.", { deg: fmt(s.tilt_deg, 0) })]);
    if (answering < expected) alerts.push([answering ? '' : 'bad', answering ? t('Rispondono {n} servo su {total} (mancano gli ID {ids}).', { n: answering, total: expected, ids: link.servos_missing.join(', ') }) : t('Nessun servo risponde sul bus: alimentazione o collegamento del bus (D0/D1).')]);
    if (link.timed_out) alerts.push(['bad', t('Il microcontrollore ha tolto coppia da solo: comandi mancanti per troppo tempo.')]);
  }
  renderSim(sim);
  if (bus && (bus.refused || bus.crc_errors)) alerts.push(['', t('Collegamento autopilota–micro: {refused} comandi rifiutati, {crc} errori CRC.', { refused: bus.refused, crc: bus.crc_errors })]);
  renderAlerts(alerts);

  // IMU
  setText('imu-acc', link.acc_g ? vec(link.acc_g, 3) + '  g' : '—');
  setText('imu-gyro', link.gyro_dps ? vec(link.gyro_dps, 1) + '  °/s' : '—');
  setText('imu-tilt', s.tilt_deg === undefined ? '—' : `${fmt(s.tilt_deg, 1)}°`, s.tilt_deg > 60 ? 'bad' : '');
  setText('imu-att', ap.attitude ? t('rollio {r}°  beccheggio {p}°  imbardata {y}°', { r: fmt(ap.attitude.roll), p: fmt(ap.attitude.pitch), y: fmt(ap.attitude.yaw) }) : '—');
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
  setText('l-proc', proc.running ? t('in esecuzione (avvii: {n})', { n: proc.starts }) : (proc.error || t('fermo')), proc.running ? '' : 'bad');
  setText('l-mav', ap.connected ? t('collegato · modo {mode}', { mode: ap.mode || '—' }) : (ap.error || t('nessun heartbeat')), ap.connected ? '' : 'bad');
  setText('l-gcs', proc.mavlink_tcp_port ? t('MAVLink 2 su TCP {host}:{port}', { host: location.hostname, port: proc.mavlink_tcp_port }) : t('non attiva'));
  setText('l-state', t("{a} Hz al relay · {b} Hz all'autopilota", { a: fmt(link.state_hz), b: bus ? fmt(bus.state_hz) : '—' }));
  setText('l-cmd', t('{hz} Hz · ultimo: {mode}', { hz: fmt(link.command_hz), mode: link.last_command_mode || '—' }));
  const c = link.control;
  setText('l-loop', c ? t('periodo {p} ms · lavoro {w} ms · {n} sforamenti', { p: (c.period_us / 1000).toFixed(2), w: (c.work_us / 1000).toFixed(2), n: c.overruns }) : '—');
  setText('l-task', task.hz == null ? '—' : t('{hz} Hz · intervallo più lungo {gap} ms · inferenza {inf} ms', { hz: fmt(task.hz), gap: fmt(task.longest_gap_ms), inf: fmt(task.inference_ms, 2) }));
  setText('l-err', t('CRC {crc} · comandi rifiutati {bad} · invii falliti {send}', { crc: link.crc_errors, bad: c ? c.invalid_commands : '—', send: link.counts.send_errors }));

  renderJoints(ap.joints || [], busHas('calibration'));
  renderServoRows(ap.joints || [], busHas('servo_setup'));
  renderTexts(ap.texts || []);
  renderControl(s, armed, busHas);
}

function renderAlerts(alerts) {
  const html = alerts.map(([cls, text]) => `<li class="${cls}">${text}</li>`).join('');
  if ($('alerts').innerHTML !== html) $('alerts').innerHTML = html;
}

function renderSim(sim) {
  const label = t({ off: 'spento', simulate: 'simulazione', debug: 'debug' }[sim.mode] || sim.mode);
  setText('sim-tag', label, 'tag ' + (sim.mode === 'off' ? '' : 'ok'));
  document.querySelectorAll('#sim-mode button').forEach((b) => {
    b.classList.toggle('on', b.dataset.sim === sim.mode);
    b.disabled = b.dataset.sim !== 'off' && !sim.available;
  });
  $('sim-reset').disabled = !sim.available || sim.mode === 'off';
  const engine = t(sim.engine || 'MuJoCo');
  const line = !sim.available ? t(sim.error || 'MuJoCo non è disponibile su questa scheda.')
    : sim.mode === 'off' ? t('Pronto · {engine} · {model}', { engine, model: sim.model || t('scena') })
    : t('{engine} · t {t} s · inclinazione {tilt}° · gravità {g} g', { engine, t: fmt(sim.sim_time, 2), tilt: fmt(sim.tilt_deg, 1), g: vec(sim.gravity_g, 2) });
  setText('sim-state', line);
}

function renderJoints(joints, inCal) {
  const sim = status && status.sim;
  const debug = !!(sim && sim.mode === 'debug' && sim.joints_deg);
  $('joints-sim-col').hidden = !debug;
  const rows = joints.map((j, i) => `<tr class="${j.present ? '' : 'missing'}"><td>${jointName(j.name)}</td><td>${j.id}</td>
    <td><span class="tag ${j.present ? 'ok' : ''}">${t(j.present ? 'presente' : 'assente')}</span></td>
    <td class="num">${j.present ? fmt(j.angle) : '—'}</td>${debug ? `<td class="num">${fmt(sim.joints_deg[i])}</td>` : ''}
    <td class="num">${fmt(j.target)}</td>
    <td class="num">${j.present && j.raw ? j.raw : '—'}</td><td class="num">${j.voltage ? fmt(j.voltage) : '—'}</td></tr>`).join('');
  if ($('joints').innerHTML !== rows) $('joints').innerHTML = rows;

  // calibration rows are built once and then updated in place, so a click is never lost to a redraw
  const body = $('cal-rows');
  if (body.children.length !== joints.length) {
    body.innerHTML = joints.map((j) => `<tr><td>${jointName(j.name)}</td><td>${j.id}</td>
      <td class="num"></td><td class="num"></td><td><span class="tag ok" hidden>${t('centrato')}</span><button data-center="${j.slot}">${t('Centra')}</button></td></tr>`).join('');
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

// One row per configured servo. Built once per count and updated in place, so typing or a slider
// move is not lost to a redraw. The live encoder column is the feedback of the direct control.
function renderServoRows(joints, inServo) {
  const body = $('servo-rows');
  if (body.children.length !== joints.length) {
    body.innerHTML = joints.map((j) => `<tr data-slot="${j.slot}" data-id="${j.id}">
      <td>${jointName(j.name)}</td><td>${j.id}</td><td class="st"></td><td class="num enc"></td>
      <td><input type="range" class="goto" min="0" max="4095" value="${ENCODER_CENTER}"> <span class="mono goto-val">${ENCODER_CENTER}</span></td>
      <td><input type="number" class="kp" min="0" max="255" value="5"></td>
      <td><input type="number" class="kd" min="0" max="255" value="20"></td>
      <td class="actions"><button data-row="read">${t('Leggi')}</button> <button data-row="set_gains">${t('Scrivi')}</button>
        <button data-row="ping">Ping</button> <button data-row="reset_mid">${t('Centro qui')}</button></td></tr>`).join('');
  }
  joints.forEach((j, i) => {
    const row = body.children[i];
    if (!row) return;
    row.className = j.present ? '' : 'missing';
    const st = row.querySelector('.st');
    const stText = t(j.present ? 'presente' : 'assente');
    if (st.textContent !== stText) st.textContent = stText;
    const enc = row.querySelector('.enc');
    const encText = j.present && j.raw ? String(j.raw) : '—';
    if (enc.textContent !== encText) enc.textContent = encText;
    row.querySelectorAll('button, input').forEach((el) => { el.disabled = !inServo; });
  });
}

function renderTexts(texts) {
  const html = texts.length ? texts.slice().reverse().map((t) => `<li><time>${t.time}</time>${t.text.replace(/</g, '&lt;')}</li>`).join('') : `<li>${t('Nessun messaggio.')}</li>`;
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
  setText('dis-pose-state', p.NNM_DIS_POSE === undefined ? '—' : t(!disPose ? 'servo liberi' : released ? 'rilasciati su richiesta' : 'attiva'),
    'tag ' + (disPose && !released ? 'warn' : ''));

  for (const [group, name] of [['chain', 'NNM_DEPLOY'], ['policy', 'NNM_POLICY']]) {
    document.querySelectorAll(`#${group} button`).forEach((b) => {
      b.classList.toggle('on', Number(b.dataset.value) === p[name]);
      b.disabled = !ready || (group === 'chain' && armed);
    });
  }
  setText('chain-state', robot ? t('{chain} in uso', { chain: chainName(robot.chain) }) : '—');
  setText('policy-state', p.NNM_POLICY === undefined ? '—' : t(p.NNM_POLICY === 2 ? 'pattini' : 'camminata'));
  setText('run-policy', robot ? policyName(robot.policy_name) + (armed ? '' : t(' (ferma: disarmato)')) : '—');
  setText('run-skill', robot ? (robot.skill ? t(SKILL_LABEL[robot.skill] || robot.skill) : t('nessuna')) : '—');
  setText('run-getup', robot ? t(GETUP_LABEL[robot.getup] || robot.getup) : '—', robot && robot.getup !== 'idle' ? 'bad' : '');

  const sitting = robot && robot.skill === 'sit';
  document.querySelectorAll('#skills button').forEach((b) => {
    if (b.id === 'sitstand') {
      b.textContent = t(sitting ? 'Alzati' : 'Siediti');
      b.dataset.skill = sitting ? 'stand' : 'sit';
      b.disabled = !armed || (robot && robot.skill && !sitting) || (robot && robot.getup !== 'idle');
    } else b.disabled = !armed || !robot || !!robot.skill || robot.getup !== 'idle';
  });

  const vxMax = p.NNM_VX_MAX ?? 0, vyMax = p.NNM_VY_MAX ?? 0, wzMax = p.NNM_WZ_MAX ?? 0;
  setText('vx', fmt(twist.vx * vxMax, 2));
  setText('vy', fmt(twist.vy * vyMax, 2));
  setText('wz', fmt(twist.wz * wzMax, 2));
  setText('limits', p.NNM_VX_MAX === undefined ? '' : t('Limiti: vx ±{vx}, vy ±{vy} m/s, ωz ±{wz} rad/s.', { vx: fmt(vxMax, 2), vy: fmt(vyMax, 2), wz: fmt(wzMax, 2) }) + (armed ? '' : t(' Il movimento ha effetto solo da armato.')));

  const inCal = busHas('calibration'), inServo = busHas('servo_setup');
  setText('cal-state', t(inCal ? 'sessione aperta' : 'chiusa'), 'tag ' + (inCal ? 'warn' : ''));
  setText('servo-state', t(inServo ? 'sessione aperta' : 'chiusa'), 'tag ' + (inServo ? 'warn' : ''));
  $('cal-enter').disabled = !ready || armed || inCal;
  $('cal-exit').disabled = !inCal;
  $('servo-enter').disabled = !ready || armed || inServo;
  $('servo-exit').disabled = !inServo;
  document.querySelectorAll('[data-servo]').forEach((b) => { b.disabled = !inServo; });
  $('gains-apply').disabled = !ready || armed || inServo;
  $('gains-read').disabled = !inServo;
  session.cal = inCal;
  session.servo = inServo;
}

// ───────────────────────────── arming and torque
$('arm').addEventListener('click', async () => {
  const s = status, link = s.link;
  const lines = [t('Armare il robot?'), '', t('La policy comincia a comandare i servo: il robot deve essere a terra, libero di muoversi, con spazio intorno.')];
  if (s.tilt_deg > 60) lines.push('', t("ATTENZIONE: per l'IMU il robot è inclinato di {deg}°, quindi partirà la rialzata.", { deg: fmt(s.tilt_deg, 0) }));
  if ((link.servos_answering || []).length < configuredServos(s.autopilot) || !link.imu_ok) lines.push('', t('Servo o IMU non rispondono: la policy girerà senza dare coppia.'));
  if (!confirm(lines.join('\n'))) return;
  const r = await post('arm', { armed: true });
  say(r.note ? serverText(r.note) : t('Armato.'), r.note ? '' : 'good');
});
$('disarm').addEventListener('click', async () => { await post('arm', { armed: false }); say(t('Disarmato.'), 'good'); });

async function stop() {
  twist.vx = twist.vy = twist.wz = 0;
  try { await post('stop'); say(t('Disarmato, servo rilasciati.'), 'good'); } catch (e) { /* shown by post */ }
}
$('stop').addEventListener('click', stop);

$('torque-off').addEventListener('click', async () => { await post('torque', { on: false }); say(t('Servo rilasciati.'), 'good'); });
$('torque-on').addEventListener('click', async () => {
  if (!confirm(t('I servo vanno in coppia e portano il robot nella posa di equilibrio in un secondo. Procedo?'))) return;
  await post('torque', { on: true });
  say(t('Posa di equilibrio richiesta.'), 'good');
});
$('dis-pose').addEventListener('change', async (e) => {
  const on = e.target.checked;
  if (on && !confirm(t('Da disarmato i servo resteranno in coppia nella posa di equilibrio, anche ai prossimi avvii. Procedo?'))) { e.target.checked = false; return; }
  try { await post('param', { name: 'NNM_DIS_POSE', value: on ? 1 : 0 }); say(t(on ? 'Posa di equilibrio da disarmato: attiva.' : 'Da disarmato i servo restano liberi.'), 'good'); }
  catch (err) { e.target.checked = !on; }
});

for (const [group, name, label] of [['chain', 'NNM_DEPLOY', 'Catena'], ['policy', 'NNM_POLICY', 'Policy']]) {
  document.querySelectorAll(`#${group} button`).forEach((b) => b.addEventListener('click', async () => {
    await post('param', { name, value: Number(b.dataset.value) });
    say(`${t(label)}: ${b.textContent}.`, 'good');
  }));
}

document.querySelectorAll('#skills button').forEach((b) => b.addEventListener('click', async () => {
  await post('skill', { name: b.dataset.skill });
  say(t('Skill avviata: {name}.', { name: b.textContent }), 'good');
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
const HEAD_LABEL = ['Collo', 'Testa · beccheggio', 'Testa · imbardata', 'Testa · rollio'];
HEAD_LABEL.forEach((label, i) => {
  $('head').insertAdjacentHTML('beforeend', `<label class="slider"><span>${t(label)}</span><input type="range" data-head="${i}" min="-1" max="1" step="0.02" value="0"></label>`);
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
  if (!r.ok) { say(t(r.source === 'autopilot' ? "{what}: rifiutato dall'autopilota." : '{what}: rifiutato dal servo.', { what: t(what) }), 'error'); throw new Error('refused'); }
  return r;
}
$('cal-enter').addEventListener('click', async () => {
  if (!confirm(t('I servo vengono rilasciati: sostieni il robot. Procedo?'))) return;
  await service({ action: 'cal_enter' }, 'Calibrazione');
  say(t('Calibrazione aperta: i servo sono liberi.'), 'good');
});
$('cal-exit').addEventListener('click', async () => { await service({ action: 'cal_exit' }, 'Chiusura'); say(t('Calibrazione chiusa.'), 'good'); });
$('cal-rows').addEventListener('click', async (e) => {
  const slot = e.target.dataset.center;
  if (slot === undefined) return;
  const name = e.target.closest('tr').firstElementChild.textContent;
  if (!confirm(t('Memorizzare la posizione attuale come centro di «{name}»?', { name }))) return;
  await service({ action: 'cal_center', index: Number(slot) }, 'Centratura');
  say(t('{name}: centro memorizzato.', { name }), 'good');
});

$('servo-enter').addEventListener('click', async () => {
  if (!confirm(t('I servo vengono rilasciati: sostieni il robot. Procedo?'))) return;
  await service({ action: 'servo_enter' }, 'Sessione servo');
  say(t('Sessione servo aperta.'), 'good');
});
$('servo-exit').addEventListener('click', async () => { await service({ action: 'servo_exit' }, 'Chiusura'); say(t('Sessione servo chiusa.'), 'good'); });
document.querySelectorAll('[data-servo]').forEach((b) => b.addEventListener('click', async () => {
  const action = b.dataset.servo;
  const body = { action, id: Number($('s-id').value) };
  if (action === 'set_id') {
    body.new_id = Number($('s-new').value);
    if (!confirm(t("Cambiare l'ID del servo {id} in {new}? Deve esserci un solo servo con ID {id} sul bus.", { id: body.id, new: body.new_id }))) return;
  }
  if (action === 'set_gains') { body.kp = Number($('s-kp').value); body.kd = Number($('s-kd').value); }
  if (action === 'set_protection') body.enabled = b.dataset.enabled === '1';
  if (action === 'set_protection_current') body.current_a = Number($('s-amp').value);
  if (action === 'reset_mid' && !confirm(t('Memorizzare la posizione attuale come centro del servo {id}?', { id: body.id }))) return;
  const out = $('servo-result');
  try {
    const r = await post('service', body);
    out.className = 'result' + (r.ok ? '' : ' bad');
    const lines = [`${b.textContent.trim()} · servo ${r.id}: ${outcome(r)}`];
    if (r.ok && (action === 'read' || action === 'set_gains' || action === 'reset_mid')) lines.push(servoValues(r));
    if (r.ok && action.startsWith('set_protection')) lines.push(t('protezione {state} · corrente {a} A', { state: r.protection ? 'ON' : 'OFF', a: r.current_a }));
    out.textContent = lines.join('\n');
  } catch (e) {
    out.className = 'result bad';
    out.textContent = e.message;
  }
}));

// Per-servo row: read, write KP/KD (EEPROM and live), ping, centre here, and the goto slider
// (sent on release, so one move is one command). The reply shows what the servo answered.
const outcome = (r) => r.ok ? t('riuscito') : t('NON riuscito ({why})', { why: t(r.source === 'autopilot' ? "rifiutato dall'autopilota" : 'nessuna risposta dal servo') });
const servoValues = (r) => t('posizione {pos} · centro {center} · Kp {kp} · Kd {kd}', { pos: r.raw_pos, center: r.center, kp: r.kp, kd: r.kd });
function showServoResult(lines, ok) {
  const out = $('servo-result');
  out.className = 'result' + (ok ? '' : ' bad');
  out.textContent = lines.join('\n');
}

async function rowRequest(row, action, extra = {}) {
  const body = { action, id: Number(row.dataset.id), ...extra };
  const r = await post('service', body);
  const label = t({ read: 'Leggi', set_gains: 'Scrivi KP/KD', ping: 'Ping', reset_mid: 'Centro qui', goto: 'Vai a' }[action]);
  const lines = [`${label} · servo ${r.id}: ${outcome(r)}`];
  if (r.ok && action !== 'ping') lines.push(servoValues(r));
  showServoResult(lines, r.ok);
  if (r.ok && (action === 'read' || action === 'set_gains')) {
    row.querySelector('.kp').value = r.kp;
    row.querySelector('.kd').value = r.kd;
  }
  return r;
}

$('servo-rows').addEventListener('click', async (e) => {
  const button = e.target.closest('button[data-row]');
  if (!button) return;
  const row = button.closest('tr');
  const action = button.dataset.row;
  if (action === 'reset_mid' && !confirm(t('Memorizzare la posizione attuale come centro del servo {id}?', { id: row.dataset.id }))) return;
  try {
    if (action === 'set_gains') {
      await rowRequest(row, 'set_gains', { kp: Number(row.querySelector('.kp').value), kd: Number(row.querySelector('.kd').value) });
    } else {
      await rowRequest(row, action);
    }
  } catch (err) {
    showServoResult([err.message], false);
  }
});

$('servo-rows').addEventListener('change', async (e) => {
  if (!e.target.classList.contains('goto')) return;
  const row = e.target.closest('tr');
  const raw = Number(e.target.value);
  row.querySelector('.goto-val').textContent = raw;
  try {
    await rowRequest(row, 'goto', { raw });
  } catch (err) {
    showServoResult([err.message], false);
  }
});
$('servo-rows').addEventListener('input', (e) => {
  if (e.target.classList.contains('goto')) e.target.closest('tr').querySelector('.goto-val').textContent = e.target.value;
});

$('gains-apply').addEventListener('click', async () => {
  if (!confirm(t('Mandare al microcontrollore la tabella KP/KD del robot? Viene applicata al prossimo armamento.'))) return;
  try {
    const r = await post('service', { action: 'run_gains' });
    showServoResult([t('Tabella KP/KD · {outcome}', { outcome: r.ok ? t('ricevuta dal microcontrollore') : t('NON ricevuta ({why})', { why: t(r.source === 'autopilot' ? "rifiutata dall'autopilota" : 'nessuna risposta') }) })], r.ok);
  } catch (err) {
    showServoResult([err.message], false);
  }
});

$('gains-read').addEventListener('click', async () => {
  const rows = [...$('servo-rows').children];
  for (const row of rows) {
    try {
      await rowRequest(row, 'read');
    } catch (err) {
      showServoResult([`${row.dataset.id}: ${err.message}`], false);
      return;
    }
  }
  showServoResult([t('Letti {n} servo.', { n: rows.length })], true);
});

// ───────────────────────────── network: status for everyone, the rest after the login
let net = null;                                               // last answer of /api/network
let netAuth = JSON.parse(sessionStorage.getItem('net') || 'null');   // { user, password } while logged in
const netPost = (body, quiet) => post('network', { ...netAuth, ...body }, quiet);
const NET_LAST = { connect: 'Collegamento a «{ssid}»', forget: 'Rete «{ssid}» dimenticata', ap_only: 'Solo access point',
  wifi_mode: 'Ritorno alla rete Wi-Fi', ap_password: "Password dell'access point", fallback_ap: 'Nessuna rete: access point acceso' };

async function pollNet() {
  try {
    if (netAuth) net = await netPost({ action: 'status' }, true);
    else net = await (await fetch('/api/network', { cache: 'no-store' })).json();
  } catch (e) {
    try { net = await (await fetch('/api/network', { cache: 'no-store' })).json(); } catch (e2) { net = null; }
  }
  renderNet();
}
setInterval(() => { if ($('tab-rete').classList.contains('on') || $('tab-info').classList.contains('on')) pollNet(); }, 3000);

function renderNet() {
  const logged = !!(netAuth && net && net.user);
  setText('net-login-state', t(logged ? 'accesso eseguito' : 'non eseguito'), 'tag ' + (logged ? 'ok' : ''));
  $('net-login').disabled = logged;
  $('net-logout').disabled = !netAuth;
  $('net-config').hidden = !logged;
  $('net-initial').hidden = !(logged && net.initial_password);
  const port = status && status.process ? status.process.mavlink_tcp_port : null;
  if (!net || !net.service) {
    for (const p of ['net-', 'info-net-']) {
      setText(p + 'tag', t('servizio assente'), 'tag warn');
      setText(p + 'mode', net ? t('Il servizio di rete non è attivo sulla scheda: esegui «bash tools/network-setup.sh».') : '—', net ? 'bad' : '');
      [p + 'ssid', p + 'ip'].forEach((id) => setText(id, '—'));
    }
    setText('net-last', '—');
    // without the helper the only address known is the one this page was opened with
    setText('info-net-web', location.origin);
    setText('info-net-mav', port ? `MAVLink 2 · TCP ${location.hostname}:${port}` : '—');
    setText('info-net-ap', '—');
    return;
  }
  const w = net.wifi, state = w.state, apOnly = net.mode === 'ap';
  const tag = t({ client: 'collegata', ap: 'access point', connecting: 'in collegamento', disconnected: 'non collegata' }[state]);
  const mode = t(state === 'ap' && apOnly ? 'Solo access point ArduDuck'
    : { client: 'Collegata a una rete Wi-Fi', ap: 'Access point ArduDuck (nessuna rete Wi-Fi trovata)', connecting: 'In collegamento…', disconnected: 'Non collegata' }[state]);
  for (const p of ['net-', 'info-net-']) {
    setText(p + 'tag', tag, 'tag ' + (state === 'client' ? 'ok' : state === 'ap' ? 'warn' : ''));
    setText(p + 'mode', mode, '');
    setText(p + 'ssid', w.ssid || '—');
  }
  setText('net-ip', w.ip ? `${w.ip} · http://${w.ip}:9527` : '—');
  setText('info-net-ip', w.ip || '—');
  setText('info-net-web', w.ip ? `http://${w.ip}:9527` : '—');
  setText('info-net-mav', w.ip && port ? `MAVLink 2 · TCP ${w.ip}:${port}` : port ? t('porta TCP {port}', { port }) : '—');
  setText('info-net-ap', `${net.ap.ssid} · ${net.ap.address} · ${t(net.ap.active ? 'acceso' : 'spento')}`);
  const last = net.last;
  setText('net-last', last ? `${last.time} · ${t(NET_LAST[last.action] || last.action, { ssid: last.ssid })}: ${last.ok ? t('riuscito') : t('NON riuscito ({why})', { why: last.message || '—' })}` : '—',
    last && !last.ok ? 'bad' : '');
  setText('net-ap-state', t(net.ap.active ? 'acceso' : 'spento'), 'tag ' + (net.ap.active ? 'warn' : ''));
  if (!logged) return;
  document.querySelectorAll('#net-mode-set button').forEach((b) => b.classList.toggle('on', b.dataset.mode === (net.mode || 'auto')));
  const esc = (text) => String(text).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/"/g, '&quot;');
  const rows = net.networks.length ? net.networks.map((n) => `<tr><td>${esc(n.ssid)}${n.in_use ? ` <span class="tag ok">${t('in uso')}</span>` : ''}</td>
    <td class="num">${n.signal} %</td><td>${t(n.secure ? 'protetta' : 'aperta')}</td><td><button data-ssid="${esc(n.ssid)}">${t('Usa')}</button></td></tr>`).join('')
    : `<tr><td colspan="4">${t('Nessuna rete in elenco: premi «Cerca reti».')}</td></tr>`;
  if ($('net-rows').innerHTML !== rows) $('net-rows').innerHTML = rows;
  const saved = net.saved.length ? net.saved.map((name) => `<tr><td>${esc(name)}</td><td><button data-join="${esc(name)}">${t('Collega')}</button><button data-edit="${esc(name)}">${t('Riconfigura')}</button><button data-forget="${esc(name)}">${t('Dimentica')}</button></td></tr>`).join('')
    : `<tr><td>${t('Nessuna rete salvata.')}</td></tr>`;
  if ($('net-saved').innerHTML !== saved) $('net-saved').innerHTML = saved;
}

$('net-login').addEventListener('click', async () => {
  netAuth = { user: $('net-user').value.trim(), password: $('net-pass').value };
  try {
    net = await netPost({ action: 'status' });
    sessionStorage.setItem('net', JSON.stringify(netAuth));
    $('net-pass').value = '';
    say(t('Accesso eseguito.'), 'good');
  } catch (e) { netAuth = null; }
  renderNet();
});
$('net-logout').addEventListener('click', () => { netAuth = null; sessionStorage.removeItem('net'); pollNet(); });

async function netConnect(ssid, password) {
  if (!ssid) return;
  if (!confirm(t('Collegare ArduDuck alla rete «{ssid}»? Se stai usando l\'access point o un\'altra rete Wi-Fi, questa pagina perde il collegamento: ritrova ArduDuck sulla nuova rete.', { ssid }))) return;
  try {
    net = await netPost({ action: 'connect', ssid, psk: password });
    $('net-psk').value = '';
    say(t('Collegamento a «{ssid}» in corso: può servire fino a un minuto.', { ssid }), 'good');
  } catch (e) { /* shown by post */ }
  renderNet();
}
$('net-connect').addEventListener('click', () => netConnect($('net-ssid-in').value.trim(), $('net-psk').value));
$('net-rows').addEventListener('click', (e) => {
  if (e.target.dataset.ssid === undefined) return;
  $('net-ssid-in').value = e.target.dataset.ssid;
  $('net-psk').focus();
});
$('net-scan').addEventListener('click', async () => {
  $('net-scan').disabled = true;
  try { net = await netPost({ action: 'scan' }); say(t('Trovate {n} reti.', { n: net.networks.length }), 'good'); } catch (e) { /* shown by post */ }
  $('net-scan').disabled = false;
  renderNet();
});
$('net-saved').addEventListener('click', async (e) => {
  const join = e.target.dataset.join, forget = e.target.dataset.forget, edit = e.target.dataset.edit;
  if (join !== undefined) { netConnect(join, ''); return; }
  if (edit !== undefined) {
    $('net-ssid-in').value = edit;
    $('net-psk').value = '';
    $('net-psk').focus();
    say(t('Scrivi la nuova password di «{ssid}» e premi «Collega».', { ssid: edit }));
    return;
  }
  if (forget === undefined || !confirm(t('Dimenticare la rete «{ssid}»? Se è quella in uso, ArduDuck si scollega e diventa access point.', { ssid: forget }))) return;
  try { net = await netPost({ action: 'forget', name: forget }); say(t('Rete «{ssid}» dimenticata', { ssid: forget }) + '.', 'good'); } catch (err) { /* shown by post */ }
  renderNet();
});
document.querySelectorAll('#net-mode-set button').forEach((b) => b.addEventListener('click', async () => {
  const mode = b.dataset.mode;
  if (!net || mode === (net.mode || 'auto')) return;
  const question = mode === 'ap'
    ? "Passare a «Solo access point»? La scheda si stacca dal Wi-Fi e resta l'access point «ArduDuck» anche dopo un riavvio: per continuare collegati alla rete ArduDuck e apri http://10.42.0.1:9527."
    : "Tornare a «Rete Wi-Fi»? L'access point si spegne e la scheda prova a collegarsi a una rete salvata; se non ci riesce in 45 secondi lo riaccende.";
  if (!confirm(t(question))) return;
  try { net = await netPost({ action: 'mode', mode }); say(t('Cambio di modalità in corso: può servire fino a un minuto.'), 'good'); } catch (e) { /* shown by post */ }
  renderNet();
}));
$('net-ap-save').addEventListener('click', async () => {
  const value = $('net-ap-psk').value;
  if (value.length < 8 || value.length > 63) { say(t('la password Wi-Fi deve avere da 8 a 63 caratteri'), 'error'); return; }
  if (!confirm(t('Cambiare la password Wi-Fi dell\'access point? Chi è collegato all\'access point deve ricollegarsi con la nuova password.'))) return;
  try { net = await netPost({ action: 'ap_password', new_password: value }); $('net-ap-psk').value = ''; say(t('Password dell\'access point cambiata.'), 'good'); } catch (e) { /* shown by post */ }
});
$('net-newpass-save').addEventListener('click', async () => {
  const value = $('net-newpass').value;
  try {
    net = await netPost({ action: 'set_password', new_password: value });
    netAuth.password = value;
    sessionStorage.setItem('net', JSON.stringify(netAuth));
    $('net-newpass').value = '';
    say(t('Password della pagina cambiata.'), 'good');
  } catch (e) { /* shown by post */ }
  renderNet();
});
languageListeners.push(renderNet);
pollNet();

poll();
