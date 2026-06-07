import os
import subprocess
import sys
import threading
import tkinter as tk
from tkinter import ttk
from datetime import datetime

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

MAIN_PY = os.path.join(os.path.dirname(__file__), "main.py")
PLOT_TMP = os.path.join(os.path.dirname(__file__), "_gui_tmp_plot.png")
MJPYTHON = os.path.join(os.path.dirname(sys.executable), "mjpython")


class PIDTunerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("UR5e PID Tuner")
        self.root.resizable(False, False)

        self._last_plot_path = None
        self._build_controls()
        self._build_plot()

    def _build_controls(self):
        ctrl = ttk.Frame(self.root, padding=12)
        ctrl.grid(row=0, column=0, sticky="ns")

        # Joint selector
        ttk.Label(ctrl, text="Joint").grid(row=0, column=0, columnspan=2, sticky="w")
        self.joint_var = tk.IntVar(value=6)
        for j in range(1, 7):
            ttk.Radiobutton(ctrl, text=str(j), variable=self.joint_var, value=j).grid(
                row=1, column=j - 1, padx=2
            )

        ttk.Separator(ctrl, orient="horizontal").grid(
            row=2, column=0, columnspan=6, sticky="ew", pady=8
        )

        # PID gain rows
        self.gains = {}
        for row_idx, (name, default, lo, hi) in enumerate(
            [("Kp", 100.0, 0.0, 500.0),
             ("Ki", 100.0, 0.0, 500.0),
             ("Kd", 100.0, 0.0, 500.0)],
            start=3,
        ):
            ttk.Label(ctrl, text=name, width=4).grid(row=row_idx, column=0, sticky="w")

            var = tk.DoubleVar(value=default)
            self.gains[name] = var

            slider = ttk.Scale(
                ctrl, from_=lo, to=hi, variable=var, orient="horizontal", length=200,
                command=lambda val, v=var: v.set(round(float(val), 2)),
            )
            slider.grid(row=row_idx, column=1, columnspan=4, padx=4)

            entry = ttk.Entry(ctrl, textvariable=var, width=8)
            entry.grid(row=row_idx, column=5, padx=4)
            entry.bind("<Return>", lambda e, v=var, lo=lo, hi=hi: self._clamp(v, lo, hi))

        ttk.Separator(ctrl, orient="horizontal").grid(
            row=6, column=0, columnspan=6, sticky="ew", pady=8
        )

        btn_frame = ttk.Frame(ctrl)
        btn_frame.grid(row=7, column=0, columnspan=6)

        self.run_btn = ttk.Button(btn_frame, text="Run", command=self._on_run)
        self.run_btn.pack(side="left", padx=6)

        self.save_btn = ttk.Button(btn_frame, text="Save", command=self._on_save, state="disabled")
        self.save_btn.pack(side="left", padx=6)

        self.status_var = tk.StringVar(value="Ready")
        ttk.Label(ctrl, textvariable=self.status_var, foreground="gray").grid(
            row=8, column=0, columnspan=6, pady=(6, 0)
        )

    def _build_plot(self):
        plot_frame = ttk.Frame(self.root, padding=(0, 12, 12, 12))
        plot_frame.grid(row=0, column=1, sticky="nsew")

        self.fig, self.ax = plt.subplots(figsize=(6, 4))
        self.ax.text(0.5, 0.5, "Press Run to simulate",
                     ha="center", va="center", transform=self.ax.transAxes,
                     color="gray", fontsize=11)
        self.ax.set_axis_off()

        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    def _clamp(self, var, lo, hi):
        try:
            var.set(max(lo, min(hi, float(var.get()))))
        except (tk.TclError, ValueError):
            pass

    def _on_run(self):
        # Kill any previous simulation process
        if hasattr(self, "_sim_proc") and self._sim_proc and self._sim_proc.poll() is None:
            self._sim_proc.terminate()

        self.run_btn.configure(state="disabled")
        self.save_btn.configure(state="disabled")
        self.status_var.set("Simulation running…")

        joint = self.joint_var.get()
        kp = self.gains["Kp"].get()
        ki = self.gains["Ki"].get()
        kd = self.gains["Kd"].get()

        if os.path.exists(PLOT_TMP):
            os.remove(PLOT_TMP)

        cmd = [
            MJPYTHON, MAIN_PY,
            "--joint", str(joint),
            "--kp", str(kp),
            "--ki", str(ki),
            "--kd", str(kd),
            "--out", PLOT_TMP,
        ]
        self._sim_proc = subprocess.Popen(cmd)

        def watch():
            import time
            while True:
                # Process died before saving the plot
                if self._sim_proc.poll() is not None:
                    if not os.path.exists(PLOT_TMP):
                        self.root.after(0, lambda: self._on_error("Simulation exited before saving plot"))
                    else:
                        self.root.after(0, self._show_plot)
                    return
                # Plot file appeared — simulation still running
                if os.path.exists(PLOT_TMP):
                    self.root.after(0, self._show_plot)
                    return
                time.sleep(0.2)

        threading.Thread(target=watch, daemon=True).start()

    def _show_plot(self):
        if not os.path.exists(PLOT_TMP):
            self._on_error("Plot file not found — did the simulation finish?")
            return

        self.ax.clear()
        self.ax.set_axis_on()
        img = mpimg.imread(PLOT_TMP)
        self.ax.imshow(img)
        self.ax.set_axis_off()
        self.fig.tight_layout(pad=0)
        self.canvas.draw()

        self._last_plot_path = PLOT_TMP
        self.run_btn.configure(state="normal")
        self.save_btn.configure(state="normal")
        self.status_var.set("Done.")

    def _on_error(self, msg):
        self.status_var.set(f"Error: {msg}")
        self.run_btn.configure(state="normal")

    def _on_save(self):
        if not self._last_plot_path or not os.path.exists(self._last_plot_path):
            return
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        joint = self.joint_var.get()
        dest = os.path.join(
            os.path.dirname(__file__),
            f"pid_j{joint}_{timestamp}.png",
        )
        import shutil
        shutil.copy2(self._last_plot_path, dest)
        self.status_var.set(f"Saved → {os.path.basename(dest)}")


if __name__ == "__main__":
    root = tk.Tk()
    app = PIDTunerApp(root)
    root.mainloop()
