const DAY_LABELS = ['M','T','W','T','F'];
let scheduleData = null;
let dayState = {}; // per-row day-bitmask edits, keyed by a row prefix string

function daysHtml(prefix, days) {
  let h = '';
  for (let i = 0; i < 5; i++) {
    const on = (days & (1 << i)) ? 'on' : '';
    h += `<span class="day ${on}" onclick="toggleDay('${prefix}',${i},this)">${DAY_LABELS[i]}</span>`;
  }
  return h;
}
function toggleDay(prefix, i, elm) {
  if (!(prefix in dayState)) dayState[prefix] = initialDays(prefix);
  dayState[prefix] ^= (1 << i);
  elm.classList.toggle('on');
}
function initialDays(prefix) {
  const parts = prefix.split('_');
  if (parts.length === 2) {
    const sched = parts[0], idx = parseInt(parts[1]);
    const arr = sched === 'A' ? scheduleData.a : scheduleData.b;
    const t = arr.find(x => x.i === idx);
    return t ? t.days : 31;
  }
  return 31; // default: all weekdays, for the "add new timer" row
}

async function postJson(url, obj) {
  const r = await fetch(url, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(obj) });
  return await r.json();
}

async function loadStatus() {
  try {
    const j = await (await fetch('/api/status')).json();
    document.getElementById('status').innerHTML =
      `<b>${j.time}</b><br>Uptime: ${j.uptime} &nbsp; WiFi: ${j.wifi} (${j.rssi} dBm) &nbsp; IP: ${j.ip} &nbsp; Free heap: ${j.heap}`;
  } catch (e) { /* transient network hiccup - next poll will retry */ }
}

async function loadSchedule() {
  scheduleData = await (await fetch('/api/schedule')).json();
  render();
}

function render() {
  document.getElementById('enBtnA').innerText = scheduleData.enabledA ? 'Enabled (tap to disable)' : 'Disabled (tap to enable)';
  document.getElementById('enBtnB').innerText = scheduleData.enabledB ? 'Enabled (tap to disable)' : 'Disabled (tap to enable)';
  renderList('A', scheduleData.a);
  renderList('B', scheduleData.b);
  renderAddForm('A');
  renderAddForm('B');
}

function renderList(sched, arr) {
  const el = document.getElementById('list' + sched);
  el.innerHTML = '';
  if (arr.length === 0) { el.innerHTML = '<div class="muted">No timers yet.</div>'; return; }
  arr.forEach(t => {
    const div = document.createElement('div');
    div.className = 'card row';
    div.innerHTML = `
      <input type="number" min="0" max="23" value="${t.h}" id="${sched}_h_${t.i}">
      :
      <input type="number" min="0" max="59" value="${t.m}" id="${sched}_m_${t.i}">
      ${daysHtml(sched + '_' + t.i, t.days)}
      <label><input type="checkbox" id="${sched}_en_${t.i}" ${t.en ? 'checked' : ''}> On</label>
      <button onclick="saveTimer('${sched}',${t.i})">Save</button>
      <button class="danger" onclick="deleteTimer('${sched}',${t.i})">Delete</button>`;
    el.appendChild(div);
  });
}

function renderAddForm(sched) {
  dayState['add' + sched] = 31;
  document.getElementById('add' + sched).innerHTML = `
    <div class="row">
      <input type="number" min="0" max="23" id="new${sched}_h" value="8">
      :
      <input type="number" min="0" max="59" id="new${sched}_m" value="0">
      ${daysHtml('add' + sched, 31)}
      <button class="go" onclick="addTimer('${sched}')">Add</button>
    </div>`;
}

async function saveTimer(sched, i) {
  const h = parseInt(document.getElementById(`${sched}_h_${i}`).value);
  const m = parseInt(document.getElementById(`${sched}_m_${i}`).value);
  const en = document.getElementById(`${sched}_en_${i}`).checked;
  const key = sched + '_' + i;
  const days = (key in dayState) ? dayState[key] : initialDays(key);
  await postJson('/api/timer', { sched, i, h, m, en, days });
  await loadSchedule();
}

async function addTimer(sched) {
  const h = parseInt(document.getElementById(`new${sched}_h`).value);
  const m = parseInt(document.getElementById(`new${sched}_m`).value);
  const days = dayState['add' + sched] !== undefined ? dayState['add' + sched] : 31;
  await postJson('/api/timer', { sched, i: -1, h, m, en: true, days });
  await loadSchedule();
}

async function deleteTimer(sched, i) {
  if (!confirm('Delete this timer?')) return;
  await postJson('/api/timer/delete', { sched, i });
  await loadSchedule();
}

async function toggleSchedule(sched) {
  const cur = sched === 'A' ? scheduleData.enabledA : scheduleData.enabledB;
  await postJson('/api/schedule/enable', { sched, en: !cur });
  await loadSchedule();
}

async function testBell(sched) {
  await postJson('/api/bell/test', { sched });
}

async function changePassword() {
  const oldPass = document.getElementById('oldPass').value;
  const newPass = document.getElementById('newPass').value;
  const r = await postJson('/api/password', { oldPass, newPass });
  document.getElementById('passMsg').innerText = r.ok ? 'Password updated.' : ('Error: ' + (r.error || 'unknown'));
  if (r.ok) { document.getElementById('oldPass').value = ''; document.getElementById('newPass').value = ''; }
}

loadStatus();
loadSchedule();
setInterval(loadStatus, 5000);