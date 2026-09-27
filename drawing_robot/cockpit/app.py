"""Stable M1 application shell over existing read-only Shared Growth sources."""

from __future__ import annotations

from pathlib import Path

import tkinter as tk
from tkinter import ttk

from .components import (
    AMBER,
    BG,
    MUTED,
    PANEL,
    PANEL_2,
    TEAL,
    TEXT,
    InputSensingPanel,
    MachineStatePanel,
    ParameterInspector,
    PipelineView,
    SignalTimeline,
    WorldView,
)
from .model import CockpitModel, Workspace


def _configure_windows_dpi() -> None:
    """Keep Tk and screenshot coordinates consistent on scaled Windows displays."""
    import sys
    if sys.platform != "win32":
        return
    try:
        import ctypes
        ctypes.windll.shcore.SetProcessDpiAwareness(1)
    except (AttributeError, OSError):
        pass


class ApplicationShell:
    """One shared shell whose workspace changes information emphasis only."""

    def __init__(
        self,
        model: CockpitModel,
        refresh_ms: int = 200,
        *,
        initial_workspace: Workspace = Workspace.RUN,
        screenshot_path: str | Path | None = None,
    ) -> None:
        self.model = model
        self.refresh_ms = max(100, int(refresh_ms))
        _configure_windows_dpi()
        self.root = tk.Tk()
        # Cap Tk's logical-pixel scaling so the instrument remains usable on
        # high-DPI lab displays instead of pushing the machine panel off-screen.
        current_scaling = float(self.root.tk.call("tk", "scaling"))
        self.root.tk.call("tk", "scaling", min(current_scaling, 1.25))
        self.root.title("Shared Growth Cockpit M1 — READ ONLY")
        width = min(1500, max(1120, self.root.winfo_screenwidth() - 40))
        height = min(960, max(760, self.root.winfo_screenheight() - 80))
        self.root.geometry(f"{width}x{height}+20+20")
        self.root.minsize(1120, 760)
        self.root.configure(bg=BG)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.workspace = tk.StringVar(value=initial_workspace.value)
        self.workspace_buttons: dict[Workspace, ttk.Button] = {}
        self._closed = False
        self._refresh_after_id: str | None = None
        self._build_style()
        self._build_layout()
        self.set_workspace(initial_workspace)
        self._refresh_after_id = self.root.after(50, self._refresh)
        if screenshot_path is not None:
            self.root.after(1800, lambda: self.capture_screenshot(screenshot_path, close=True))

    def _build_style(self) -> None:
        style = ttk.Style(self.root)
        style.theme_use("clam")
        style.configure("Cockpit.TFrame", background=BG)
        style.configure("Panel.TFrame", background=PANEL)
        style.configure("Panel2.TFrame", background=PANEL_2)
        style.configure("Title.TLabel", background=BG, foreground=TEXT, font=("Segoe UI", 18, "bold"))
        style.configure("PanelTitle.TLabel", background=PANEL, foreground=TEXT, font=("Segoe UI", 10, "bold"))
        style.configure("Body.TLabel", background=PANEL, foreground=TEXT, font=("Segoe UI", 9))
        style.configure("Muted.TLabel", background=PANEL, foreground=MUTED, font=("Segoe UI", 9))
        style.configure("Value.TLabel", background=PANEL, foreground=TEAL, font=("Consolas", 10, "bold"))
        style.configure("Cockpit.TButton", background=PANEL_2, foreground=TEXT, padding=(10, 6), borderwidth=0)
        style.configure("Workspace.TButton", background=BG, foreground=MUTED, padding=(12, 7), borderwidth=0, font=("Segoe UI", 9, "bold"))
        style.configure("WorkspaceActive.TButton", background=PANEL_2, foreground=TEAL, padding=(12, 7), borderwidth=1, font=("Segoe UI", 9, "bold"))

    def _build_layout(self) -> None:
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_rowconfigure(1, weight=1)
        self.root.grid_rowconfigure(2, weight=0, minsize=100)
        self.root.grid_rowconfigure(3, weight=0, minsize=150)
        self._build_header()

        main = ttk.Frame(self.root, style="Cockpit.TFrame")
        main.grid(row=1, column=0, sticky="nsew", padx=12, pady=(0, 7))
        main.grid_rowconfigure(0, weight=1)
        main.grid_columnconfigure(0, weight=0, minsize=255)
        main.grid_columnconfigure(1, weight=1)
        main.grid_columnconfigure(2, weight=0, minsize=355)

        left = ttk.Frame(main, style="Cockpit.TFrame")
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        left.grid_columnconfigure(0, weight=1)
        left.grid_rowconfigure(0, weight=1)
        self.input_panel = InputSensingPanel(left)
        self.input_panel.grid(row=0, column=0, sticky="nsew")
        self.parameter_inspector = ParameterInspector(left)
        self.parameter_inspector.grid(row=1, column=0, sticky="ew", pady=(7, 0))

        self.world_view = WorldView(main, self.model)
        self.world_view.grid(row=0, column=1, sticky="nsew", padx=6)
        self.machine_panel = MachineStatePanel(main)
        self.machine_panel.grid(row=0, column=2, sticky="nsew", padx=(6, 0))

        self.timeline = SignalTimeline(self.root)
        self.timeline.grid(row=2, column=0, sticky="nsew", padx=12, pady=(0, 7))
        self.pipeline = PipelineView(self.root, self.model)
        self.pipeline.grid(row=3, column=0, sticky="nsew", padx=12, pady=(0, 12))

    def _build_header(self) -> None:
        header = ttk.Frame(self.root, style="Cockpit.TFrame", padding=(16, 10))
        header.grid(row=0, column=0, sticky="ew")
        ttk.Label(header, text="SHARED GROWTH", style="Title.TLabel").pack(side="left")
        nav = ttk.Frame(header, style="Cockpit.TFrame")
        nav.pack(side="left", padx=(36, 0))
        for workspace in Workspace:
            button = ttk.Button(
                nav, text=workspace.value, style="Workspace.TButton",
                command=lambda value=workspace: self.set_workspace(value),
            )
            button.pack(side="left", padx=2)
            self.workspace_buttons[workspace] = button
        ttk.Label(
            header, text="M1  •  OBSERVER  •  READ ONLY",
            background=BG, foreground=AMBER, font=("Consolas", 10, "bold"),
        ).pack(side="right")

    def set_workspace(self, workspace: Workspace) -> None:
        workspace = Workspace(workspace)
        self.workspace.set(workspace.value)
        for item, button in self.workspace_buttons.items():
            button.configure(style="WorkspaceActive.TButton" if item is workspace else "Workspace.TButton")
        self.parameter_inspector.set_workspace(workspace)
        self.world_view.set_workspace(workspace)

    def _refresh(self) -> None:
        if self._closed:
            return
        camera = self.model.camera.latest()
        state = self.model.robot.latest()
        execution = self.model.execution.latest()
        machine_mode = self.model.display_mode(state)
        self.input_panel.update_snapshot(camera)
        self.machine_panel.update_state(self.model)
        self.world_view.update_view(camera, machine_mode)
        self.timeline.update_events(execution)
        self.pipeline.refresh()
        self._refresh_after_id = self.root.after(self.refresh_ms, self._refresh)

    def capture_screenshot(self, path: str | Path, *, close: bool = False) -> Path:
        from PIL import ImageGrab

        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        self.root.deiconify()
        self.root.lift()
        self.root.attributes("-topmost", True)
        self.root.focus_force()
        self.root.update_idletasks()
        self.root.update()
        x, y = self.root.winfo_rootx(), self.root.winfo_rooty()
        width, height = self.root.winfo_width(), self.root.winfo_height()
        ImageGrab.grab(bbox=(x, y, x + width, y + height)).save(destination)
        self.root.attributes("-topmost", False)
        if close:
            self.root.after(50, self.close)
        return destination

    def run(self) -> None:
        self.root.mainloop()

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        if self._refresh_after_id is not None:
            try:
                self.root.after_cancel(self._refresh_after_id)
            except tk.TclError:
                pass
            self._refresh_after_id = None
        self.world_view.close()
        self.root.quit()
        self.model.close()
        self.root.destroy()


# Preserve the v0.1 import name while making the shell role explicit.
CockpitApp = ApplicationShell
