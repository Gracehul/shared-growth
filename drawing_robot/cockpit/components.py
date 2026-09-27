"""Reusable read-only UI components for the Shared Growth application shell."""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk
from typing import Any, Callable

from .model import CockpitModel, Workspace
from .sources import CameraSnapshot, ExecutionSnapshot


BG = "#10151d"
PANEL = "#18212c"
PANEL_2 = "#1e2a37"
TEXT = "#edf3f8"
MUTED = "#90a0b2"
NAVY = "#4d86c6"
TEAL = "#42c3b4"
AMBER = "#e8ae48"
RED = "#e46363"
LINE = "#334355"


def semantic_color(status: str) -> str:
    value = status.upper()
    if value in {"FAILED", "FAULT", "INVALID", "ACTIVE", "OFFLINE", "INPUT ERROR"}:
        return RED
    if value in {"STOPPING", "NOT INTEGRATED", "NOT LOADED", "NO DATA", "NO RUN"}:
        return AMBER
    if value in {"READY", "RUNNING", "CONNECTED", "VALID", "LIVE", "COMPLETED", "NONE"}:
        return TEAL
    return NAVY


class MetricPanelMixin:
    def metric(self, label: str, row: int):
        self.ttk.Label(self, text=label, style="Muted.TLabel").grid(
            row=row, column=0, sticky="w", pady=4
        )
        value = self.ttk.Label(self, text="—", style="Value.TLabel")
        value.grid(row=row, column=1, sticky="e", padx=(10, 0), pady=4)
        return value


class InputSensingPanel(MetricPanelMixin, ttk.Frame):
    def __init__(self, parent) -> None:
        self.ttk = ttk
        super().__init__(parent, style="Panel.TFrame", padding=14)
        self.grid_columnconfigure(1, weight=1)
        ttk.Label(self, text="INPUT / SENSING", style="PanelTitle.TLabel").grid(
            row=0, column=0, columnspan=2, sticky="w", pady=(0, 10)
        )
        self.connected = self.metric("Camera", 1)
        self.resolution = self.metric("Resolution", 2)
        self.fps = self.metric("Effective FPS", 3)
        self.age = self.metric("Frame age", 4)
        self.failed = self.metric("Failed / dropped", 5)
        self.detail = ttk.Label(
            self, text="", style="Muted.TLabel", wraplength=220, justify="left"
        )
        self.detail.grid(row=6, column=0, columnspan=2, sticky="ew", pady=(10, 0))

    def update_snapshot(self, camera: CameraSnapshot) -> None:
        age = camera.frame_age_s()
        self.connected.configure(text="CONNECTED" if camera.connected else "OFFLINE")
        self.resolution.configure(
            text="—" if camera.resolution is None else f"{camera.resolution[0]} × {camera.resolution[1]}"
        )
        self.fps.configure(text="—" if camera.effective_fps is None else f"{camera.effective_fps:.1f}")
        self.age.configure(text="—" if age is None else f"{age * 1000:.0f} ms")
        self.failed.configure(text=f"{camera.failed_frames} / {camera.estimated_dropped_frames}")
        self.detail.configure(text=camera.detail or camera.device)


class ParameterInspector(ttk.Frame):
    """Structural M1 extension point; deliberately contains no invented parameters."""

    def __init__(self, parent) -> None:
        self.ttk = ttk
        super().__init__(parent, style="Panel.TFrame", padding=14)
        ttk.Label(self, text="PARAMETER INSPECTOR", style="PanelTitle.TLabel").pack(anchor="w")
        self.workspace = ttk.Label(self, text="RUN", style="Value.TLabel")
        self.workspace.pack(anchor="w", pady=(8, 4))
        self.message = ttk.Label(
            self, text="", style="Muted.TLabel", wraplength=220, justify="left"
        )
        self.message.pack(anchor="w", fill="x")

    def set_workspace(self, workspace: Workspace) -> None:
        messages = {
            Workspace.AUTHOR: "No editable authoring parameters exist in M1.",
            Workspace.PREVIEW: "Trajectory and ValidationResult are read-only inputs.",
            Workspace.RUN: "Resolved hardware and execution configuration remains owned by the existing runtime.",
            Workspace.REVIEW: "Recorded configuration is available only when an ExecutionLog is loaded.",
        }
        self.workspace.configure(text=workspace.value)
        self.message.configure(text=messages[workspace])


class MachineStatePanel(MetricPanelMixin, ttk.Frame):
    def __init__(self, parent) -> None:
        self.ttk = ttk
        super().__init__(parent, style="Panel.TFrame", padding=14)
        self.grid_columnconfigure(1, weight=1)
        ttk.Label(self, text="MACHINE / EXECUTION", style="PanelTitle.TLabel").grid(
            row=0, column=0, columnspan=4, sticky="w", pady=(0, 10)
        )
        self.state = self.metric("State", 1)
        self.result = self.metric("Run result", 2)
        self.age = self.metric("Telemetry age", 3)
        self.temperature = self.metric("Max temperature", 4)
        self.fault = self.metric("Fault", 5)
        ttk.Separator(self).grid(row=6, column=0, columnspan=4, sticky="ew", pady=10)
        for col, text in enumerate(("Joint", "Target", "Actual", "Error")):
            ttk.Label(self, text=text, style="Muted.TLabel").grid(
                row=7, column=col, sticky="e" if col else "w", padx=4
            )
        self.joints = []
        for index in range(6):
            values = []
            for col, initial in enumerate((f"J{index + 1}", "—", "—", "—")):
                label = ttk.Label(self, text=initial, style="Body.TLabel")
                label.grid(row=8 + index, column=col, sticky="e" if col else "w", padx=4, pady=3)
                values.append(label)
            self.joints.append(values)
        for col in range(1, 4):
            self.grid_columnconfigure(col, weight=1)
        self.detail = ttk.Label(
            self, text="", style="Muted.TLabel", wraplength=315, justify="left"
        )
        self.detail.grid(row=15, column=0, columnspan=4, sticky="ew", pady=(12, 0))

    def update_state(self, model: CockpitModel) -> None:
        state = model.robot.latest()
        execution = model.execution.latest()
        mode = model.display_mode(state)
        state_age = model.state_age_s(state)
        temperatures = () if state is None else state.temperatures_c
        controller_error = None if state is None else state.controller_error
        fault = None if state is None else state.fault
        self.state.configure(text=mode, foreground=semantic_color(mode))
        self.result.configure(text=execution.result.upper())
        self.age.configure(text="—" if state_age is None else f"{state_age:.2f} s")
        self.temperature.configure(text="—" if not temperatures else f"{max(temperatures):.0f} °C")
        fault_label = fault or (
            None if controller_error in (None, 0) else f"controller {controller_error}"
        )
        fault_status = "NONE" if fault_label is None else "ACTIVE"
        self.fault.configure(text=fault_status, foreground=semantic_color(fault_status))
        self.detail.configure(text=fault_label or execution.fault or "No reported fault")
        actual = () if state is None else state.angles_deg
        target = execution.target_angles_deg
        for index, labels in enumerate(self.joints):
            target_value = target[index] if len(target) == 6 else None
            actual_value = actual[index] if len(actual) == 6 else None
            error = None if target_value is None or actual_value is None else target_value - actual_value
            labels[1].configure(text="—" if target_value is None else f"{target_value:7.2f}°")
            labels[2].configure(text="—" if actual_value is None else f"{actual_value:7.2f}°")
            labels[3].configure(text="—" if error is None else f"{error:+7.2f}°")


class WorldView(ttk.Frame):
    """Container for camera and the existing Stage-2B renderer."""

    MODES = ("camera", "trajectory")

    def __init__(self, parent, model: CockpitModel) -> None:
        self.ttk = ttk
        self.model = model
        super().__init__(parent, style="Panel2.TFrame", padding=8)
        self.grid_rowconfigure(1, weight=1)
        self.grid_columnconfigure(0, weight=1)
        self.mode = tk.StringVar(value="camera")
        self._camera_photo = None
        self._trajectory_canvas = None
        self._trajectory_visualizer = None
        controls = ttk.Frame(self, style="Panel2.TFrame")
        controls.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        ttk.Label(controls, text="WORLD VIEW", background=PANEL_2, foreground=TEXT, font=("Segoe UI", 11, "bold")).pack(side="left")
        ttk.Button(controls, text="CAMERA", style="Cockpit.TButton", command=lambda: self.show("camera")).pack(side="left", padx=(14, 4))
        ttk.Button(controls, text="TRAJECTORY / ROBOT", style="Cockpit.TButton", command=lambda: self.show("trajectory")).pack(side="left")
        self.status = ttk.Label(controls, text="", background=PANEL_2, foreground=MUTED, font=("Consolas", 9))
        self.status.pack(side="right")
        self.viewport = ttk.Frame(self, style="Panel2.TFrame")
        self.viewport.grid(row=1, column=0, sticky="nsew")
        self.viewport.grid_rowconfigure(0, weight=1)
        self.viewport.grid_columnconfigure(0, weight=1)
        self.camera_view = tk.Canvas(
            self.viewport, bg="#090d12", highlightthickness=0,
            width=1, height=1,
        )
        self.camera_view.create_text(
            160, 120, text="CAMERA OFFLINE", fill=MUTED, font=("Consolas", 14)
        )
        self.trajectory_host = ttk.Frame(self.viewport, style="Panel2.TFrame")
        self.show("camera")

    def set_workspace(self, workspace: Workspace) -> None:
        if workspace in {Workspace.PREVIEW, Workspace.REVIEW} and self.model.trajectory.sample_count:
            self.show("trajectory")
        elif workspace is Workspace.RUN:
            self.show("camera")

    def show(self, mode: str) -> None:
        if mode not in self.MODES:
            raise ValueError(f"unsupported WorldView mode: {mode}")
        self.mode.set(mode)
        self.camera_view.grid_forget()
        self.trajectory_host.grid_forget()
        if mode == "camera":
            self.camera_view.grid(row=0, column=0, sticky="nsew")
            return
        self.trajectory_host.grid(row=0, column=0, sticky="nsew")
        if self._trajectory_canvas is not None:
            return
        source = self.model.trajectory
        if source.cartesian is None or source.joint is None or source.validation is None:
            text = source.error or "No Stage-2B trajectory bundle loaded"
            self.ttk.Label(
                self.trajectory_host, text=text, background=PANEL_2, foreground=MUTED
            ).pack(expand=True)
            return
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
        from ..stage2.visualization import MotionVisualizer

        visualizer = MotionVisualizer(source.cartesian, source.joint, source.validation)
        self._trajectory_visualizer = visualizer
        self._trajectory_canvas = FigureCanvasTkAgg(
            visualizer.figure, master=self.trajectory_host
        )
        self._trajectory_canvas.draw()
        self._trajectory_canvas.get_tk_widget().pack(fill="both", expand=True)

    def update_view(self, camera: CameraSnapshot, machine_mode: str) -> None:
        self.status.configure(text=f"{self.mode.get().upper()}  •  {machine_mode}")
        if camera.frame_bgr is None or self.mode.get() != "camera":
            return
        try:
            from PIL import Image, ImageTk

            image = Image.fromarray(camera.frame_bgr[:, :, ::-1])
            width = max(self.camera_view.winfo_width(), 320)
            height = max(self.camera_view.winfo_height(), 240)
            image.thumbnail((width, height), Image.Resampling.BILINEAR)
            self._camera_photo = ImageTk.PhotoImage(image)
            self.camera_view.delete("all")
            self.camera_view.create_image(
                width / 2, height / 2, image=self._camera_photo, anchor="center"
            )
        except Exception as exc:
            self.camera_view.delete("all")
            self.camera_view.create_text(
                max(self.camera_view.winfo_width(), 320) / 2,
                max(self.camera_view.winfo_height(), 240) / 2,
                text=f"Preview unavailable\n{exc}", fill=MUTED,
                font=("Consolas", 11), justify="center",
            )

    def close(self) -> None:
        if self._trajectory_visualizer is not None:
            import matplotlib.pyplot as plt
            plt.close(self._trajectory_visualizer.figure)
            self._trajectory_visualizer = None


class SignalTimeline(ttk.Frame):
    """Compact view over real ExecutionLog events; never fabricates signals."""

    def __init__(self, parent) -> None:
        super().__init__(parent, style="Panel.TFrame", padding=(12, 8))
        ttk.Label(self, text="TIME / SIGNALS", style="PanelTitle.TLabel").pack(anchor="w")
        self.canvas = tk.Canvas(self, height=70, bg=PANEL, highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)

    def update_events(self, execution: ExecutionSnapshot) -> None:
        canvas = self.canvas
        canvas.delete("all")
        events = execution.events
        width = max(canvas.winfo_width(), 800)
        if not events:
            canvas.create_text(
                12, 38, anchor="w",
                text="No timestamped execution events available",
                fill=MUTED, font=("Consolas", 9),
            )
            return
        start = events[0][0]
        end = events[-1][0]
        span = max(end - start, 1e-9)
        canvas.create_line(20, 38, width - 20, 38, fill=LINE, width=2)
        label_indices = {0, len(events) // 2, len(events) - 1}
        labeled_x: list[float] = []
        for index, (timestamp, kind) in enumerate(events):
            x = 20 + (timestamp - start) / span * (width - 40)
            color = semantic_color(kind.replace("RUN_", ""))
            canvas.create_line(x, 26, x, 50, fill=color, width=2)
            if index in label_indices and all(abs(x - value) >= 90 for value in labeled_x):
                anchor = "w" if index == 0 else "e" if index == len(events) - 1 else "center"
                canvas.create_text(
                    x, 61, text=kind.replace("COMMAND_", "CMD "), fill=MUTED,
                    font=("Consolas", 7), anchor=anchor,
                )
                labeled_x.append(x)


class PipelineView(ttk.Frame):
    def __init__(self, parent, model: CockpitModel) -> None:
        self.model = model
        super().__init__(parent, style="Panel.TFrame", padding=(12, 8))
        ttk.Label(self, text="SYSTEM PIPELINE", style="PanelTitle.TLabel").pack(anchor="w")
        self.canvas = tk.Canvas(self, height=118, bg=PANEL, highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)

    def refresh(self) -> None:
        canvas = self.canvas
        canvas.delete("all")
        nodes = self.model.pipeline_nodes()
        width = max(canvas.winfo_width(), 1000)
        margin, gap = 12, 14
        box_width = (width - 2 * margin - gap * (len(nodes) - 1)) / len(nodes)
        y0, y1 = 20, 99
        for index, node in enumerate(nodes):
            x0 = margin + index * (box_width + gap)
            x1 = x0 + box_width
            color = AMBER if node.kind == "placeholder" else MUTED if node.kind == "isolated" else semantic_color(node.status)
            canvas.create_rectangle(x0, y0, x1, y1, fill=PANEL_2, outline=color, width=2)
            canvas.create_text((x0 + x1) / 2, y0 + 18, text=node.name, fill=TEXT, font=("Segoe UI", 9, "bold"), width=box_width - 10)
            canvas.create_text((x0 + x1) / 2, y0 + 43, text=node.status, fill=color, font=("Consolas", 9, "bold"), width=box_width - 10)
            canvas.create_text((x0 + x1) / 2, y0 + 64, text=node.detail, fill=MUTED, font=("Segoe UI", 8), width=box_width - 10)
            if index:
                previous_right = x0 - gap
                dash = (4, 3) if index == 1 else None
                canvas.create_line(previous_right, (y0 + y1) / 2, x0, (y0 + y1) / 2, fill=MUTED if dash else NAVY, width=2, arrow="last", dash=dash)
