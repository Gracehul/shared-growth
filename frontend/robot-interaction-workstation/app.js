"use strict";

// UI-only prototype data. No network calls and no robot-control imports.
const mockState = {
  mode: "manual",
  workflow: "preview",
  selectedOperation: "ready",
  operations: [
    { id: "ready", label: "GO TO READY", description: "Move to verified setup pose", state: "READY TO RUN", detail: "validated path", act: "AWAITING OPERATOR", tone: "planned" },
    { id: "park", label: "PARK", description: "Controlled shutdown pose", state: "VALID WITH WARNING", detail: "review warning", act: "AWAITING CONFIRMATION", tone: "warning" },
    { id: "test", label: "TEST MOTION", description: "Prepared conservative motion", state: "READY TO RUN", detail: "Stage-2 validated", act: "NOT STARTED", tone: "planned" },
    { id: "branch", label: "DRAW TEST BRANCH", description: "Prepared branch trajectory", state: "BLOCKED — WORKSPACE", detail: "reason available", act: "NOT EXECUTED", tone: "blocked" },
    { id: "calibration", label: "CALIBRATE WORKSPACE", description: "Relate robot, tool, camera and surface", state: "NOT AVAILABLE", detail: "backend not implemented", act: "NOT EXECUTED", tone: "warning" }
  ],
  joints: [
    ["J1", "5.05°", "5.02°", "+0.03°"],
    ["J2", "51.24°", "51.18°", "+0.06°"],
    ["J3", "−69.28°", "−69.20°", "−0.08°"],
    ["J4", "−62.57°", "−62.61°", "+0.04°"],
    ["J5", "−2.62°", "−2.64°", "+0.02°"],
    ["J6", "18.20°", "18.18°", "+0.02°"]
  ],
  pipeline: [
    ["SENSE", "CAMERA CONNECTED", "live"],
    ["INTERPRET", "NOT INTEGRATED", "incomplete"],
    ["MAP", "NOT INTEGRATED", "incomplete"],
    ["PLAN", "NO ACTIVE TRAJECTORY", "planned"],
    ["GATE", "LAST: VALID", "planned"],
    ["EXECUTE", "LAST: FAILED", "failed"],
    ["MEASURE", "ROBOT READY · LIVE", "actual"]
  ]
};

const elements = {
  workstation: document.querySelector("#workstation"),
  modeValue: document.querySelector("#mode-value"),
  contextMode: document.querySelector("#context-mode"),
  contextContent: document.querySelector("#context-content"),
  operationList: document.querySelector("#operation-list"),
  jointTable: document.querySelector("#joint-table"),
  pipeline: document.querySelector("#pipeline"),
  intend: document.querySelector("#intend-value"),
  gate: document.querySelector("#gate-value"),
  act: document.querySelector("#act-value"),
  toast: document.querySelector("#toast"),
  worldStatus: document.querySelector("#world-status")
};

function showToast(message) {
  elements.toast.textContent = message;
  elements.toast.classList.add("visible");
  window.setTimeout(() => elements.toast.classList.remove("visible"), 1800);
}

function operationToneClass(tone) {
  if (tone === "blocked") return "blocked-state";
  if (tone === "warning") return "warning-state";
  return "";
}

function renderOperations() {
  elements.operationList.replaceChildren(...mockState.operations.map(operation => {
    const button = document.createElement("button");
    button.type = "button";
    button.className = `operation ${operationToneClass(operation.tone)}${operation.id === mockState.selectedOperation ? " selected" : ""}`;
    button.dataset.operation = operation.id;
    button.innerHTML = `<strong>${operation.label}</strong><span class="operation-state">${operation.state}</span><span>${operation.description}</span><span>${operation.detail}</span>`;
    return button;
  }));
}

function renderContext() {
  const operation = mockState.operations.find(item => item.id === mockState.selectedOperation);
  if (mockState.mode === "observe") {
    elements.contextContent.innerHTML = `<p>Read-only view of authoritative camera, robot, execution and log state.</p>`;
    return;
  }
  if (mockState.mode === "shared-growth") {
    elements.contextContent.innerHTML = `<p>Growth mapping, sensing calibration and timing remain unresolved. Future parameters stay explicitly unavailable.</p>`;
    return;
  }
  elements.contextContent.innerHTML = `<dl class="metrics"><dt>Operation</dt><dd>${operation.label}</dd><dt>Trajectory</dt><dd class="planned-state">PREPARED</dd><dt>Gate</dt><dd class="${operation.tone === "blocked" ? "bad" : operation.tone === "warning" ? "warning" : "planned-state"}">${operation.state}</dd><dt>Execution</dt><dd>${operation.act}</dd></dl>`;
}

function renderCausality() {
  const operation = mockState.operations.find(item => item.id === mockState.selectedOperation);
  if (mockState.mode === "shared-growth") {
    elements.intend.textContent = "NO GENERATED RESPONSE";
    elements.gate.textContent = "BLOCKED — SENSING NOT INTEGRATED";
    elements.gate.className = "bad";
    elements.act.textContent = "NO ROBOT ACTION";
    return;
  }
  if (mockState.mode === "observe") {
    elements.intend.textContent = "NO ACTIVE REQUEST";
    elements.gate.textContent = "OBSERVE ONLY";
    elements.gate.className = "";
    elements.act.textContent = "ROBOT READY";
    return;
  }
  elements.intend.textContent = operation.label;
  elements.gate.textContent = operation.state;
  elements.gate.className = operation.tone === "blocked" ? "bad" : operation.tone === "warning" ? "warning" : "planned-state";
  elements.act.textContent = operation.act;
}

function renderMode() {
  elements.workstation.dataset.mode = mockState.mode;
  elements.modeValue.textContent = mockState.mode.replace("-", " ").toUpperCase();
  elements.contextMode.textContent = elements.modeValue.textContent;
  document.querySelectorAll("[data-mode-button]").forEach(button => button.setAttribute("aria-pressed", String(button.dataset.modeButton === mockState.mode)));
  document.querySelectorAll("[data-for-mode]").forEach(panel => panel.classList.toggle("active", panel.dataset.forMode === mockState.mode));
  elements.worldStatus.textContent = mockState.mode === "observe" ? "MEASURED STATE" : mockState.mode === "shared-growth" ? "PERCEIVED / PLANNED / MEASURED" : "PREVIEW · NOT SENT";
  renderContext();
  renderCausality();
}

function renderStaticData() {
  elements.jointTable.innerHTML = mockState.joints.map(row => `<tr>${row.map(value => `<td>${value}</td>`).join("")}</tr>`).join("");
  elements.pipeline.innerHTML = mockState.pipeline.map(([name, status, className]) => `<div class="pipeline-node ${className}"><strong>${name}</strong><span>${status}</span></div>`).join("");
}

document.addEventListener("click", event => {
  const modeButton = event.target.closest("[data-mode-button]");
  if (modeButton) {
    mockState.mode = modeButton.dataset.modeButton;
    renderMode();
    return;
  }
  const workflowButton = event.target.closest("[data-workflow]");
  if (workflowButton) {
    mockState.workflow = workflowButton.dataset.workflow;
    document.querySelectorAll("[data-workflow]").forEach(button => button.setAttribute("aria-pressed", String(button === workflowButton)));
    showToast(`${mockState.workflow.toUpperCase()} · UI WORKFLOW ONLY`);
    return;
  }
  const operationButton = event.target.closest("[data-operation]");
  if (operationButton) {
    mockState.selectedOperation = operationButton.dataset.operation;
    renderOperations();
    renderContext();
    renderCausality();
    showToast(`${operationButton.textContent.trim().split(operationButton.querySelector(".operation-state").textContent)[0].trim()} · NOT SENT`);
  }
});

document.querySelector("#global-stop").addEventListener("click", () => {
  elements.act.textContent = "STOP UI INTENT · NOT SENT";
  showToast("STOP REQUEST · NO BACKEND CONNECTED");
});

renderOperations();
renderStaticData();
renderMode();
