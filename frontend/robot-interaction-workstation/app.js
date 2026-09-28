"use strict";

const state = {
  connected: false,
  mode: "OBSERVE",
  workflow: "preview",
  selectedOperation: null,
  operations: [],
  snapshot: null,
  gates: [],
  receipt: null,
  latestEvent: null,
  backend: "offline",
  requestPending: false
};

// UI-only placeholder: deliberately absent from the command API until a
// workspace-calibration backend contract exists.
const unavailableOperations = [{
  operation_id: "calibrate-workspace",
  label: "CALIBRATE WORKSPACE",
  description: "Relate robot, tool, camera and drawing surface",
  detail: "BACKEND NOT IMPLEMENTED"
}];

const elements = {
  workstation: document.querySelector("#workstation"),
  modeValue: document.querySelector("#mode-value"),
  runtimeState: document.querySelector("#runtime-state"),
  dataSource: document.querySelector("#data-source"),
  contextMode: document.querySelector("#context-mode"),
  contextContent: document.querySelector("#context-content"),
  operationList: document.querySelector("#operation-list"),
  jointTable: document.querySelector("#joint-table"),
  pipeline: document.querySelector("#pipeline"),
  perceive: document.querySelector("#perceive-value"),
  intend: document.querySelector("#intend-value"),
  gate: document.querySelector("#gate-value"),
  act: document.querySelector("#act-value"),
  toast: document.querySelector("#toast"),
  worldStatus: document.querySelector("#world-status")
};

function formatNumber(value, digits = 2) {
  return Number.isFinite(Number(value)) ? Number(value).toFixed(digits) : "—";
}

function showToast(message, bad = false) {
  elements.toast.textContent = message;
  elements.toast.className = `toast visible${bad ? " bad" : ""}`;
  window.setTimeout(() => { elements.toast.className = "toast"; }, 2600);
}

async function requestJson(url, options = {}) {
  const response = await fetch(url, {
    cache: "no-store",
    headers: { "Content-Type": "application/json", ...(options.headers || {}) },
    ...options
  });
  const payload = await response.json().catch(() => ({}));
  if (!response.ok) throw new Error(payload.detail || payload.error || `HTTP ${response.status}`);
  return payload;
}

function displayMode(mode) {
  return String(mode || "UNKNOWN").replaceAll("_", " ");
}

function toneForStatus(status) {
  if (status === "PASS") return "good";
  if (status === "WARN") return "warning";
  if (status === "BLOCK" || status === "UNKNOWN") return "bad";
  return "";
}

function visibleOperations() {
  if (state.mode === "OBSERVE") return [];
  if (state.mode === "MANUAL") {
    return state.operations.filter(item => ["GO_READY", "EXECUTE_TRAJECTORY"].includes(item.operation));
  }
  return state.operations.filter(item => ["GO_READY", "START_SHARED_GROWTH"].includes(item.operation));
}

function selectedOperation() {
  return state.operations.find(item => item.operation_id === state.selectedOperation) || null;
}

function renderOperations() {
  const operations = visibleOperations();
  if (!operations.some(item => item.operation_id === state.selectedOperation)) {
    state.selectedOperation = operations[0]?.operation_id || null;
  }
  elements.operationList.replaceChildren();
  if (!operations.length) {
    const empty = document.createElement("p");
    empty.className = "metrics";
    empty.textContent = state.connected ? "OBSERVE is read-only. STOP remains globally available." : "Backend unavailable.";
    elements.operationList.appendChild(empty);
    return;
  }
  operations.forEach(operation => {
    const button = document.createElement("button");
    button.type = "button";
    button.className = `operation${operation.operation_id === state.selectedOperation ? " selected" : ""}`;
    button.dataset.operation = operation.operation_id;
    button.disabled = !state.connected || state.requestPending;
    const label = document.createElement("strong");
    label.textContent = operation.label;
    const status = document.createElement("span");
    status.className = "operation-state";
    status.textContent = state.requestPending ? "REQUEST ACTIVE" : "PREPARED";
    const description = document.createElement("span");
    description.textContent = operation.description || "High-level core operation";
    const detail = document.createElement("span");
    detail.textContent = operation.trajectory_id || operation.operation;
    button.append(label, status, description, detail);
    elements.operationList.appendChild(button);
  });
  if (state.mode === "MANUAL") {
    unavailableOperations.forEach(operation => {
      const button = document.createElement("button");
      button.type = "button";
      button.className = "operation warning-state";
      button.disabled = true;
      const label = document.createElement("strong");
      label.textContent = operation.label;
      const status = document.createElement("span");
      status.className = "operation-state";
      status.textContent = "NOT AVAILABLE";
      const description = document.createElement("span");
      description.textContent = operation.description;
      const detail = document.createElement("span");
      detail.textContent = operation.detail;
      button.append(label, status, description, detail);
      elements.operationList.appendChild(button);
    });
  }
}

function renderJoints(snapshot) {
  const targets = snapshot?.q_target_deg || [];
  const actual = snapshot?.q_actual_deg || [];
  const errors = snapshot?.joint_error_deg || [];
  const rows = Array.from({ length: 6 }, (_, index) => {
    const target = targets[index];
    const measured = actual[index];
    const error = errors[index];
    return `<tr><td>J${index + 1}</td><td>${formatNumber(target)}°</td><td>${formatNumber(measured)}°</td><td>${formatNumber(error)}°</td></tr>`;
  });
  elements.jointTable.innerHTML = rows.join("");
}

function primaryGate() {
  return state.gates.find(gate => ["BLOCK", "UNKNOWN"].includes(gate.status))
    || state.gates.find(gate => gate.status === "WARN")
    || state.gates[0]
    || null;
}

function renderMachine() {
  const snapshot = state.snapshot;
  const phase = snapshot?.execution_phase || "UNKNOWN";
  const temperatures = snapshot?.temperatures_c || [];
  const maximum = temperatures.length ? Math.max(...temperatures) : null;
  const hardProblem = state.gates.some(gate => gate.category === "HARD" && ["BLOCK", "UNKNOWN"].includes(gate.status));
  const health = document.querySelector("#machine-health");
  health.textContent = !state.connected ? "OFFLINE" : hardProblem ? "BLOCKED" : "CORE ONLINE";
  health.className = !state.connected || hardProblem ? "bad" : "good";
  document.querySelector("#execution-state").textContent = phase;
  document.querySelector("#execution-state").className = ["FAILED", "STOPPING"].includes(phase) ? "bad" : "good";
  document.querySelector("#last-run-result").textContent = state.receipt?.state || "NO RUN";
  document.querySelector("#operation-availability").textContent = state.connected ? (hardProblem ? "GATED" : "AVAILABLE") : "OFFLINE";
  document.querySelector("#telemetry-age").textContent = snapshot?.telemetry_age_s == null ? "UNKNOWN" : `${formatNumber(snapshot.telemetry_age_s, 3)} s`;
  document.querySelector("#maximum-temperature").textContent = maximum == null ? "UNKNOWN" : `${formatNumber(maximum, 1)} °C`;
  document.querySelector("#controller-fault").textContent = snapshot?.controller_fault == null || snapshot?.controller_fault === 0 ? "NONE" : String(snapshot.controller_fault);
  document.querySelector("#controller-fault").className = snapshot?.controller_fault == null || snapshot?.controller_fault === 0 ? "good" : "bad";
  elements.runtimeState.textContent = phase;
  document.querySelector("#shared-actual").textContent = phase;
  renderJoints(snapshot);
}

function renderReferenceState() {
  const measuredPoseAvailable =
    state.connected && (state.snapshot?.q_actual_deg || []).length === 6;
  const source = document.querySelector("#robot-pose-source");
  const available = document.querySelector("#current-robot-pose");
  source.textContent = measuredPoseAvailable ? "MEASURED ROBOTSTATE" : "UNAVAILABLE";
  source.className = measuredPoseAvailable ? "good" : "bad";
  available.textContent = measuredPoseAvailable ? "AVAILABLE" : "NOT AVAILABLE";
  available.className = measuredPoseAvailable ? "good" : "warning";
}

function renderContext() {
  const operation = selectedOperation();
  if (!state.connected) {
    elements.contextContent.innerHTML = "<p>Backend unavailable. No operation can be sent.</p>";
    return;
  }
  if (state.mode === "OBSERVE") {
    elements.contextContent.innerHTML = "<p>Read-only view of the authoritative SystemSnapshot and GateResults.</p>";
    return;
  }
  if (!operation) {
    elements.contextContent.innerHTML = "<p>No prepared high-level operation is available.</p>";
    return;
  }
  const gate = primaryGate();
  const gateTone = gate?.status === "PASS" ? "planned-state" : toneForStatus(gate?.status);
  elements.contextContent.innerHTML = `<dl class="metrics"><dt>Operation</dt><dd>${operation.label}</dd><dt>Trajectory</dt><dd class="planned-state">${operation.trajectory_id || "SESSION / READY"}</dd><dt>Gate evidence</dt><dd class="${gateTone}">${gate ? `${gate.name}: ${gate.status}` : "ABSENT"}</dd><dt>Execution</dt><dd>${state.receipt?.state || "NOT STARTED"}</dd></dl>`;
}

function renderCausality() {
  const receipt = state.receipt;
  const evidence = receipt?.evidence;
  const operation = selectedOperation();
  elements.perceive.textContent = evidence ? `${evidence.perception}${evidence.perception_id ? ` · ${evidence.perception_id}` : ""}` : (state.connected ? "ROBOT STATE PRESENT" : "BACKEND OFFLINE");
  elements.intend.textContent = evidence ? `${evidence.intended}${evidence.intent_id ? ` · ${evidence.intent_id}` : ""}` : (operation?.label || "NO ACTIVE REQUEST");
  const gate = receipt?.gate_results?.find(item => ["BLOCK", "UNKNOWN"].includes(item.status)) || primaryGate();
  elements.gate.textContent = gate ? `${gate.category} · ${gate.name} · ${gate.status}` : "UNKNOWN";
  elements.gate.className = gate?.status === "PASS" ? "planned-state" : toneForStatus(gate?.status);
  elements.act.textContent = receipt ? `${receipt.state}${receipt.run_id ? ` · ${receipt.run_id}` : ""}` : "NO OPERATION RECEIPT";
  elements.worldStatus.textContent = state.connected ? "SCHEMATIC · CORE STATE" : "BACKEND OFFLINE";
}

function renderPipeline() {
  const receipt = state.receipt;
  const snapshot = state.snapshot;
  const evidence = receipt?.evidence;
  const nodes = [
    ["SENSE", evidence?.perception || "ABSENT", evidence?.perception === "PRESENT" ? "live" : "incomplete"],
    ["INTERPRET", evidence?.intended || "ABSENT", evidence?.intended === "PRESENT" ? "planned" : "incomplete"],
    ["MAP", evidence?.intent_id || "NOT INTEGRATED", evidence?.intent_id ? "planned" : "incomplete"],
    ["PLAN", snapshot?.validation_status || "UNKNOWN", snapshot?.validation_status === "PASS" ? "planned" : "incomplete"],
    ["GATE", receipt ? (receipt.accepted ? "ACCEPTED" : "REJECTED") : "NO DECISION", receipt ? (receipt.accepted ? "planned" : "failed") : "incomplete"],
    ["EXECUTE", evidence?.commanded || "ABSENT", evidence?.commanded === "PRESENT" ? "planned" : "incomplete"],
    ["MEASURE", evidence?.actual || (state.connected ? "PRESENT" : "UNKNOWN"), state.connected ? "actual" : "incomplete"]
  ];
  elements.pipeline.innerHTML = nodes.map(([name, status, className]) => `<div class="pipeline-node ${className}"><strong>${name}</strong><span>${status}</span></div>`).join("");
}

function renderObserve() {
  document.querySelector("#snapshot-api").textContent = state.connected ? "CONNECTED" : "OFFLINE";
  document.querySelector("#snapshot-api").className = state.connected ? "good" : "bad";
  document.querySelector("#observe-validation").textContent = state.snapshot?.validation_status || "UNKNOWN";
  document.querySelector("#observe-age").textContent = state.snapshot?.telemetry_age_s == null ? "UNKNOWN" : `${formatNumber(state.snapshot.telemetry_age_s, 3)} s`;
  document.querySelector("#observe-event").textContent = state.latestEvent?.kind || "ABSENT";
}

function renderMode() {
  const htmlMode = state.mode.toLowerCase().replace("_", "-");
  elements.workstation.dataset.mode = htmlMode;
  elements.modeValue.textContent = displayMode(state.mode);
  elements.contextMode.textContent = displayMode(state.mode);
  document.querySelectorAll("[data-mode-button]").forEach(button => {
    button.setAttribute("aria-pressed", String(button.dataset.modeButton === htmlMode));
  });
  document.querySelectorAll("[data-for-mode]").forEach(panel => panel.classList.toggle("active", panel.dataset.forMode === htmlMode));
}

function renderAll() {
  renderMode();
  renderOperations();
  renderMachine();
  renderReferenceState();
  renderContext();
  renderCausality();
  renderPipeline();
  renderObserve();
  elements.dataSource.textContent = state.connected ? state.backend.toUpperCase() : "BACKEND OFFLINE";
  elements.dataSource.className = state.connected ? "" : "mock";
}

async function refresh() {
  try {
    const payload = await requestJson("/api/workstation/status");
    state.connected = true;
    state.mode = payload.mode;
    state.operations = payload.operations || [];
    state.snapshot = payload.snapshot;
    state.gates = payload.gate_results || [];
    state.receipt = payload.latest_receipt;
    state.latestEvent = payload.latest_execution_event;
    state.backend = payload.backend;
    state.requestPending = Boolean(payload.active_request);
  } catch (_) {
    state.connected = false;
    state.snapshot = null;
    state.gates = [];
    state.requestPending = false;
  }
  renderAll();
}

async function changeMode(htmlMode) {
  const mode = htmlMode.replace("-", "_").toUpperCase();
  try {
    await requestJson("/api/workstation/mode", { method: "POST", body: JSON.stringify({ mode }) });
    await refresh();
  } catch (error) {
    showToast(`MODE NOT CHANGED · ${error.message}`, true);
  }
}

async function runSelected() {
  const operation = selectedOperation();
  if (!state.connected || !operation) {
    showToast(state.mode === "OBSERVE" ? "OBSERVE IS READ-ONLY" : "NO PREPARED OPERATION", true);
    return;
  }
  state.requestPending = true;
  renderAll();
  try {
    const receipt = await requestJson("/api/workstation/operations", {
      method: "POST",
      body: JSON.stringify({ operation_id: operation.operation_id, request_id: crypto.randomUUID() })
    });
    state.receipt = receipt;
    showToast(`${receipt.accepted ? "ACCEPTED" : "REJECTED"} · ${receipt.state}`, !receipt.accepted);
  } catch (error) {
    showToast(`REQUEST FAILED · ${error.message}`, true);
  } finally {
    state.requestPending = false;
    await refresh();
  }
}

document.addEventListener("click", event => {
  const modeButton = event.target.closest("[data-mode-button]");
  if (modeButton) {
    changeMode(modeButton.dataset.modeButton);
    return;
  }
  const workflowButton = event.target.closest("[data-workflow]");
  if (workflowButton) {
    state.workflow = workflowButton.dataset.workflow;
    document.querySelectorAll("[data-workflow]").forEach(button => button.setAttribute("aria-pressed", String(button === workflowButton)));
    if (state.workflow === "run") runSelected();
    else showToast(`${state.workflow.toUpperCase()} · LOCAL VIEW`);
    return;
  }
  const operationButton = event.target.closest("[data-operation]");
  if (operationButton) {
    state.selectedOperation = operationButton.dataset.operation;
    renderAll();
    showToast(`${selectedOperation()?.label || "OPERATION"} · SELECTED, NOT SENT`);
  }
});

document.querySelector("#global-stop").addEventListener("click", async () => {
  try {
    const receipt = await requestJson("/api/workstation/stop", {
      method: "POST",
      body: JSON.stringify({ request_id: `stop-${crypto.randomUUID()}` })
    });
    state.receipt = receipt;
    showToast(`${receipt.state} · ${receipt.reason || "CONFIRMED"}`, !receipt.accepted || receipt.state !== "STOPPED");
  } catch (error) {
    showToast(`STOP FAILED · ${error.message}`, true);
  } finally {
    await refresh();
  }
});

renderAll();
refresh();
window.setInterval(refresh, 1000);
