import csv
import os
import shutil
import subprocess
import sys
import threading
import tkinter as tk
from tkinter import ttk, filedialog
from datetime import datetime

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
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
        self._sim_proc = None
        self._build_ui()

    def _build_ui(self):
        left = ttk.Frame(self.root, padding=12)
        left.grid(row=0, column=0, sticky="ns")

        # Header row
        for col, text in enumerate(["Joint", "Kp", "Ki", "Kd", "Active"]):
            ttk.Label(left, text=text, width=10, anchor="center",
                      font=("", 10, "bold")).grid(row=0, column=col, padx=4, pady=4)

        ttk.Separator(left, orient="horizontal").grid(
            row=1, column=0, columnspan=5, sticky="ew", pady=2
        )

        self.joint_rows = {}
        for j in range(1, 7):
            row = j + 1
            ttk.Label(left, text=str(j), anchor="e", width=6).grid(
                row=row, column=0, padx=4, pady=4
            )

            kp_var = tk.DoubleVar(value=0.0)
            ki_var = tk.DoubleVar(value=0.0)
            kd_var = tk.DoubleVar(value=0.0)
            active_var = tk.BooleanVar(value=False)

            ttk.Entry(left, textvariable=kp_var, width=10, justify="right").grid(
                row=row, column=1, padx=4, pady=4
            )
            ttk.Entry(left, textvariable=ki_var, width=10, justify="right").grid(
                row=row, column=2, padx=4, pady=4
            )
            ttk.Entry(left, textvariable=kd_var, width=10, justify="right").grid(
                row=row, column=3, padx=4, pady=4
            )
            ttk.Checkbutton(left, variable=active_var).grid(
                row=row, column=4, padx=4, pady=4
            )

            self.joint_rows[j] = {"kp": kp_var, "ki": ki_var, "kd": kd_var, "active": active_var}

        ttk.Separator(left, orient="horizontal").grid(
            row=8, column=0, columnspan=5, sticky="ew", pady=8
        )

        btn_frame = ttk.Frame(left)
        btn_frame.grid(row=9, column=0, columnspan=5)

        self.run_btn = ttk.Button(btn_frame, text="Run", command=self._on_run)
        self.run_btn.pack(side="left", padx=8)

        ttk.Button(btn_frame, text="Export Data", command=self._on_export).pack(
            side="left", padx=8
        )

        self.save_btn = ttk.Button(btn_frame, text="Save Graph", command=self._on_save, state="disabled")
        self.save_btn.pack(side="left", padx=8)

        self.status_var = tk.StringVar(value="Ready")
        ttk.Label(left, textvariable=self.status_var, foreground="gray").grid(
            row=10, column=0, columnspan=5, pady=(6, 0)
        )

        # Plot panel
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

    def _active_joints(self):
        return [j for j in range(1, 7) if self.joint_rows[j]["active"].get()]

    def _on_run(self):
        active = self._active_joints()
        if not active:
            self.status_var.set("No joints selected.")
            return

        if self._sim_proc and self._sim_proc.poll() is None:
            self._sim_proc.terminate()

        self.run_btn.configure(state="disabled")
        self.save_btn.configure(state="disabled")
        self.status_var.set("Simulation running…")

        if os.path.exists(PLOT_TMP):
            os.remove(PLOT_TMP)

        configs = []
        for j in active:
            r = self.joint_rows[j]
            configs.append(f"{j}:{r['kp'].get()}:{r['ki'].get()}:{r['kd'].get()}")

        cmd = [MJPYTHON, MAIN_PY, "--joints", ",".join(configs), "--out", PLOT_TMP]
        self._sim_proc = subprocess.Popen(cmd)

        def watch():
            import time
            while True:
                if self._sim_proc.poll() is not None:
                    if not os.path.exists(PLOT_TMP):
                        self.root.after(0, lambda: self._on_error("Simulation exited before saving plot"))
                    else:
                        self.root.after(0, self._show_plot)
                    return
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

    def _on_export(self):
        path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv")],
            initialfile=f"pid_gains_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
        )
        if not path:
            return
        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["Joint", "Kp", "Ki", "Kd"])
            for j in range(1, 7):
                r = self.joint_rows[j]
                writer.writerow([j, r["kp"].get(), r["ki"].get(), r["kd"].get()])
        self.status_var.set(f"Exported → {os.path.basename(path)}")

    def _on_save(self):
        if not self._last_plot_path or not os.path.exists(self._last_plot_path):
            return
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        dest = os.path.join(os.path.dirname(__file__), f"pid_plot_{timestamp}.png")
        shutil.copy2(self._last_plot_path, dest)
        self.status_var.set(f"Saved → {os.path.basename(dest)}")


if __name__ == "__main__":
    root = tk.Tk()
    app = PIDTunerApp(root)
    root.mainloop()
