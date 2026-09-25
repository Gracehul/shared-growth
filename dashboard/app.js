const state = {
  latest: null,
  history: [],
  receivedAt: 0,
  colors: ["#65b8ff", "#7bf2c4", "#f7d36d", "#ff8d72", "#c89cff", "#e8eef2"],
};

const $ = (selector) => document.querySelector(selector);
const format = (value, digits = 1) => Number.isFinite(Number(value)) ? Number(value).toFixed(digits) : "—";

async function loadBrand() {
  try {
    const brand = await fetch("/brand.json", { cache: "no-store" }).then((response) => response.json());
    $("#product-name").textContent = brand.productName;
    $("#surface-name").textContent = brand.surfaceName;
    document.title = `${brand.productName} — ${brand.surfaceName}`;
    if (brand.accent) document.documentElement.style.setProperty("--accent", brand.accent);
    if (brand.accentSecondary) document.documentElement.style.setProperty("--accent-secondary", brand.accentSecondary);
  } catch (_) {
    // Built-in identity remains usable when the optional brand file is absent.
  }
}

function renderJoints(joints = []) {
  const grid = $("#joint-grid");
  grid.replaceChildren();
  joints.forEach((joint) => {
    const range = joint.maximum_deg - joint.minimum_deg;
    const position = range > 0 ? ((joint.angle_deg - joint.minimum_deg) / range) * 100 : 50;
    const temperatureClass = joint.temperature_c >= 60 ? "danger" : joint.temperature_c >= 55 ? "warning" : "";
    const card = document.createElement("article");
    card.className = "joint-card";
    card.innerHTML = `
      <div class="joint-head">
        <span class="joint-id">Joint ${joint.id}</span>
        <span class="joint-temperature ${temperatureClass}">${format(joint.temperature_c, 0)} °C</span>
      </div>
      <div class="joint-angle">${format(joint.angle_deg, 2)}°</div>
      <div class="limit-track" aria-label="Joint ${joint.id} position within limits">
        <span class="limit-safe"></span>
        <span class="joint-marker" style="left:${Math.max(0, Math.min(100, position))}%"></span>
      </div>
      <div class="joint-numbers"><span>${format(joint.minimum_deg, 0)}°</span><span>${format(joint.maximum_deg, 0)}°</span></div>
      <div class="joint-meta">
        <span><b>${format(joint.voltage_v, 1)} V</b> voltage</span>
        <span><b>${format(joint.speed_steps_s, 0)}</b> step/s</span>
      </div>`;
    grid.appendChild(card);
  });
}

function renderAlerts(alerts = []) {
  const list = $("#alert-list");
  $("#alert-count").textContent = alerts.length;
  list.replaceChildren();
  if (!alerts.length) {
    const item = document.createElement("li");
    item.className = "empty-state";
    item.textContent = "No active signals. All read-only gates are clear.";
    list.appendChild(item);
    return;
  }
  alerts.forEach((alert) => {
    const item = document.createElement("li");
    item.innerHTML = `<div class="alert-title ${alert.severity}">${alert.title}</div><p>${alert.detail}</p>`;
    list.appendChild(item);
  });
}

function renderPose(pose = []) {
  const values = $("#pose-grid").querySelectorAll("dd");
  pose.forEach((value, index) => {
    values[index].textContent = index < 3 ? format(value, 1) : `${format(value, 2)}°`;
  });
}

function drawChart() {
  const canvas = $("#temperature-chart");
  const rect = canvas.getBoundingClientRect();
  const scale = window.devicePixelRatio || 1;
  canvas.width = Math.max(1, Math.floor(rect.width * scale));
  canvas.height = Math.max(1, Math.floor(rect.height * scale));
  const ctx = canvas.getContext("2d");
  ctx.scale(scale, scale);
  const width = rect.width;
  const height = rect.height;
  const pad = { left: 34, right: 10, top: 8, bottom: 24 };
  const chartWidth = width - pad.left - pad.right;
  const chartHeight = height - pad.top - pad.bottom;
  ctx.clearRect(0, 0, width, height);

  const all = state.history.flatMap((sample) => sample.temperatures_c || []);
  const minimum = Math.min(30, ...all);
  const maximum = Math.max(70, ...all);
  ctx.font = "11px ui-monospace, monospace";
  ctx.fillStyle = "#7d8c98";
  ctx.strokeStyle = "#202b35";
  ctx.lineWidth = 1;
  [minimum, 45, 60, maximum].filter((value, index, array) => array.indexOf(value) === index).forEach((value) => {
    const y = pad.top + chartHeight - ((value - minimum) / (maximum - minimum)) * chartHeight;
    ctx.beginPath(); ctx.moveTo(pad.left, y); ctx.lineTo(width - pad.right, y); ctx.stroke();
    ctx.fillText(`${Math.round(value)}°`, 2, y + 4);
  });
  const gateY = pad.top + chartHeight - ((60 - minimum) / (maximum - minimum)) * chartHeight;
  ctx.strokeStyle = "rgba(255,111,111,.65)";
  ctx.setLineDash([5, 5]);
  ctx.beginPath(); ctx.moveTo(pad.left, gateY); ctx.lineTo(width - pad.right, gateY); ctx.stroke();
  ctx.setLineDash([]);

  if (state.history.length < 2) return;
  for (let joint = 0; joint < 6; joint += 1) {
    ctx.strokeStyle = state.colors[joint];
    ctx.lineWidth = joint === 3 ? 2.4 : 1.5;
    ctx.beginPath();
    state.history.forEach((sample, index) => {
      const x = pad.left + (index / (state.history.length - 1)) * chartWidth;
      const value = sample.temperatures_c?.[joint];
      const y = pad.top + chartHeight - ((value - minimum) / (maximum - minimum)) * chartHeight;
      if (index === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    });
    ctx.stroke();
  }
}

function renderLegend() {
  const legend = $("#chart-legend");
  legend.innerHTML = state.colors.map((color, index) => `<span class="legend-item"><i class="legend-swatch" style="background:${color}"></i>J${index + 1}</span>`).join("");
}

function render(payload) {
  state.latest = payload.current;
  state.history = payload.history || [];
  state.receivedAt = Date.now();
  const current = payload.current;
  const dot = $("#status-dot");
  dot.className = `status-dot ${current.severity || ""}`;
  $("#system-state").textContent = current.state_label;
  $("#system-detail").textContent = current.state_detail;
  $("#controller-state").textContent = current.connected ? "Online" : "Offline";
  $("#controller-detail").textContent = current.read_only ? "Motion interface locked" : "Unknown mode";
  const maxTemp = Math.max(...(current.joints || []).map((joint) => joint.temperature_c));
  $("#maximum-temperature").textContent = Number.isFinite(maxTemp) ? `${format(maxTemp, 0)} °C` : "—";
  $("#controller-error").textContent = current.controller_error ?? "—";
  $("#controller-error-label").textContent = current.controller_error_label || "Unavailable";
  $("#sample-latency").textContent = current.sample_latency_ms != null ? `${format(current.sample_latency_ms, 0)} ms` : "—";
  $("#serial-port").textContent = current.port || "—";
  $("#robot-host").textContent = current.host || "—";
  renderJoints(current.joints);
  renderAlerts(current.alerts);
  renderPose(current.firmware_pose_mm_deg);
  drawChart();
}

function renderOffline(message) {
  $("#status-dot").className = "status-dot danger";
  $("#system-state").textContent = "Dashboard disconnected";
  $("#system-detail").textContent = message;
  $("#controller-state").textContent = "Offline";
}

async function refresh() {
  try {
    const response = await fetch("/api/status", { cache: "no-store" });
    if (!response.ok) throw new Error(`status ${response.status}`);
    render(await response.json());
  } catch (error) {
    renderOffline(`No telemetry response: ${error.message}`);
  }
}

function updateTime() {
  $("#clock").textContent = new Intl.DateTimeFormat(undefined, { hour: "2-digit", minute: "2-digit", second: "2-digit" }).format(new Date());
  const age = state.receivedAt ? (Date.now() - state.receivedAt) / 1000 : null;
  $("#data-age").textContent = age == null ? "—" : `${age.toFixed(1)} s`;
}

window.addEventListener("resize", drawChart);
loadBrand();
renderLegend();
refresh();
setInterval(refresh, 1000);
updateTime();
setInterval(updateTime, 250);
