import time
import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import serial
import device_model

# adopted from my set_baudrate.py for GUI
BAUD_CODES = {
    4800: "50 06 00 04 00 01 04 4A",
    9600: "50 06 00 04 00 02 44 4B",
    19200: "50 06 00 04 00 03 85 8B",
    38400: "50 06 00 04 00 04 C4 49",
    57600: "50 06 00 04 00 05 05 89",
    115200: "50 06 00 04 00 06 45 88",
    230400: "50 06 00 04 00 07 84 48",
}

# these save & unlock command hex are from WTVB01-485 User's Guide
UNLOCK_CMD = bytes.fromhex("50 06 00 69 B5 88 22 A1")
SAVE_CMD = bytes.fromhex("50 06 00 00 00 00 84 44")


def change_sensor_baudrate(port: str, current_baud: int, target_baud: int):
    """
    changes baud rate sensor WTVB01-485 dan save it to memory flash.
    """
    if target_baud not in BAUD_CODES:
        raise ValueError(f"Baud rate {target_baud} is not supported by the sensor.")

    with serial.Serial(port, current_baud, timeout=0.5) as ser:
        # unlock register (valid for 10s)
        ser.write(UNLOCK_CMD)
        time.sleep(0.1)

        # send the code of the target baud rate to register 0x04
        ser.write(bytes.fromhex(BAUD_CODES[target_baud]))
        time.sleep(0.1)

    # unlock it again with new baud rate then send command SAVE (register 0x00) so that the setting is saved permanently
    with serial.Serial(port, target_baud, timeout=0.5) as ser_new:
        ser_new.write(UNLOCK_CMD)
        time.sleep(0.1)
        ser_new.write(SAVE_CMD)
        time.sleep(0.1)


class FourRunBalancingApp:
    def __init__(self, root):
        self.root = root
        self.root.title("WTVB01-485 Four-Run Rotor Balancing Analyzer (Lecture Aid)")
        self.root.geometry("1320x820")

        self.dev = None
        self.is_connected = False
        self.is_capturing = False

        # store captured results for Runs 1, 2, 3, 4
        # each entry holds {"X": (amp, freq), "Y": (amp, freq), "Z": (amp, freq), "fs": float}
        self.run_results = [None, None, None, None]
        self.trial_angles = (0.0, 120.0, 240.0)

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # GUI layout (Tkinter Frames, Inputs, Tables, and Embedded Matplotlib)
    def _build_ui(self):
        # main layout: left control panel (fixed width) + right visualization panel
        left_panel = ttk.Frame(self.root, padding=10)
        left_panel.pack(side=tk.LEFT, fill=tk.Y)

        right_panel = ttk.Frame(self.root, padding=10)
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        # section sensor connection
        conn_frame = ttk.LabelFrame(left_panel, text="1. Sensor Connection", padding=8)
        conn_frame.pack(fill=tk.X, pady=(0, 8))

        ttk.Label(conn_frame, text="COM Port:").grid(row=0, column=0, sticky="w", pady=2)
        self.port_var = tk.StringVar(value="COM7")
        ttk.Entry(conn_frame, textvariable=self.port_var, width=9).grid(row=0, column=1, sticky="w", padx=4)

        ttk.Label(conn_frame, text="Target Baud Rate:").grid(row=0, column=2, sticky="w", pady=2)
        self.baud_var = tk.StringVar(value="115200")
        ttk.Combobox(
            conn_frame,
            textvariable=self.baud_var,
            values=["4800", "9600", "19200", "38400", "57600", "115200", "230400"],
            width=9,
            state="readonly",
        ).grid(row=0, column=3, sticky="w", padx=4)

        ttk.Label(conn_frame, text="Initial Baud Rate:").grid(row=1, column=0, sticky="w", pady=3)
        self.init_baud_var = tk.StringVar(value="9600")
        ttk.Combobox(
            conn_frame,
            textvariable=self.init_baud_var,
            values=["9600", "115200", "4800", "19200", "38400", "57600", "230400"],
            width=8,
            state="readonly",
        ).grid(row=1, column=1, sticky="w", padx=4)

        self.set_baud_btn = ttk.Button(
            conn_frame, text="Set & Save Baud Rate", command=self._handle_set_baudrate
        )
        self.set_baud_btn.grid(row=1, column=2, columnspan=2, sticky="ew", padx=4, pady=3)

        self.conn_btn = ttk.Button(conn_frame, text="Connect", command=self._toggle_connection)
        self.conn_btn.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(4, 2), padx=(0, 4))

        self.status_var = tk.StringVar(value="Status: Disconnected")
        ttk.Label(conn_frame, textvariable=self.status_var, foreground="gray").grid(
            row=2, column=2, columnspan=2, sticky="w", pady=(4, 2)
        )

        self.live_readout_var = tk.StringVar(value="Live Accel: AX=---  AY=---  AZ=---")
        ttk.Label(conn_frame, textvariable=self.live_readout_var, font=("Consolas", 9)).grid(
            row=3, column=0, columnspan=4, sticky="w", pady=(4, 0)
        )

        # section Rotor & FFT Parameters
        param_frame = ttk.LabelFrame(left_panel, text="2. Balancing & FFT Settings", padding=8)
        param_frame.pack(fill=tk.X, pady=(0, 8))

        self.m_trial_var = tk.StringVar(value="10.0")
        self.r_trial_var = tk.StringVar(value="50.0")
        self.r_corr_var = tk.StringVar(value="50.0")
        self.duration_var = tk.StringVar(value="4.0")
        self.fmin_var = tk.StringVar(value="2.0")
        self.fmax_var = tk.StringVar(value="60.0")
        self.axis_var = tk.StringVar(value="Z")  # Z is convenient for desk/PC top testing; X/Y for rotor radial

        fields = [
            ("Trial Mass mt (g):", self.m_trial_var, "Trial Radius rt (mm):", self.r_trial_var),
            ("Corr. Radius rc (mm):", self.r_corr_var, "Capture Time (s):", self.duration_var),
            ("Min Freq (Hz):", self.fmin_var, "Max Freq (Hz):", self.fmax_var),
        ]
        for r_idx, (l1, v1, l2, v2) in enumerate(fields):
            ttk.Label(param_frame, text=l1).grid(row=r_idx, column=0, sticky="w", pady=2)
            ttk.Entry(param_frame, textvariable=v1, width=8).grid(row=r_idx, column=1, sticky="w", padx=4)
            ttk.Label(param_frame, text=l2).grid(row=r_idx, column=2, sticky="w", pady=2)
            ttk.Entry(param_frame, textvariable=v2, width=8).grid(row=r_idx, column=3, sticky="w", padx=4)

        ttk.Label(param_frame, text="Balancing Axis:").grid(row=3, column=0, sticky="w", pady=(4, 0))
        axis_box = ttk.Combobox(
            param_frame, textvariable=self.axis_var, values=["X", "Y", "Z"], width=6, state="readonly"
        )
        axis_box.grid(row=3, column=1, sticky="w", padx=4, pady=(4, 0))
        axis_box.bind("<<ComboboxSelected>>", lambda e: self._recalculate_if_ready())

        ttk.Label(param_frame, text="(Use Z on desk/PC, X/Y on rotor)", foreground="gray").grid(
            row=3, column=2, columnspan=2, sticky="w", pady=(4, 0)
        )

        # section Four-Run capture controls
        run_frame = ttk.LabelFrame(left_panel, text="3. Four-Run Data Acquisition", padding=8)
        run_frame.pack(fill=tk.X, pady=(0, 8))

        run_labels = [
            "Capture Run 1 (Initial - No Trial Mass)",
            "Capture Run 2 (Trial Mass @ 0°)",
            "Capture Run 3 (Trial Mass @ 120°)",
            "Capture Run 4 (Trial Mass @ 240°)",
        ]
        self.run_btns = []
        for idx, text in enumerate(run_labels):
            btn = ttk.Button(run_frame, text=text, command=lambda i=idx: self._start_capture(i))
            btn.pack(fill=tk.X, pady=2)
            self.run_btns.append(btn)

        btn_row = ttk.Frame(run_frame)
        btn_row.pack(fill=tk.X, pady=(4, 0))
        ttk.Button(btn_row, text="Calculate Four-Run", command=self._recalculate_if_ready).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 4)
        )
        ttk.Button(btn_row, text="Reset Runs", command=self._reset_runs).pack(
            side=tk.RIGHT, fill=tk.X, expand=True
        )

        # summary table of captured peak accelerations
        cols = ("run", "x_max", "y_max", "z_max", "freq")
        self.tree = ttk.Treeview(run_frame, columns=cols, show="headings", height=4)
        self.tree.heading("run", text="Run")
        self.tree.heading("x_max", text="X Max (g)")
        self.tree.heading("y_max", text="Y Max (g)")
        self.tree.heading("z_max", text="Z Max (g)")
        self.tree.heading("freq", text="Peak Hz")
        for c, w in zip(cols, [65, 78, 78, 78, 65]):
            self.tree.column(c, width=w, anchor="center")
        self.tree.pack(fill=tk.X, pady=(6, 0))
        self._refresh_table()

        # section Four-Run calculation outputs
        out_frame = ttk.LabelFrame(left_panel, text="4. Balancing Analysis Results", padding=8)
        out_frame.pack(fill=tk.BOTH, expand=True)

        self.result_vars = {
            "unbal_loc": tk.StringVar(value="---"),
            "init_unbal": tk.StringVar(value="---"),
            "res_unbal": tk.StringVar(value="---"),
            "corr_pos": tk.StringVar(value="---"),
            "corr_mass": tk.StringVar(value="---"),
        }

        result_rows = [
            ("Unbalance Location:", "unbal_loc"),
            ("Initial Unbalance:", "init_unbal"),
            ("Residual Unbalance (Est.):", "res_unbal"),
            ("Correction Mass Position:", "corr_pos"),
            ("Correction Mass & Weight:", "corr_mass"),
        ]
        for r_idx, (label_text, key) in enumerate(result_rows):
            ttk.Label(out_frame, text=label_text, font=("Segoe UI", 9, "bold")).grid(
                row=r_idx, column=0, sticky="w", pady=3
            )
            ttk.Label(out_frame, textvariable=self.result_vars[key], font=("Consolas", 9, "bold"),
                      foreground="#004a99").grid(
                row=r_idx, column=1, sticky="w", padx=6, pady=3
            )

        # right panel: embedded Matplotlib figure (3 FFT plots + 1 Polar plot) ---
        self.fig = Figure(figsize=(8.5, 7.5), dpi=100)
        gs = self.fig.add_gridspec(3, 2, width_ratios=[1.4, 1.0], hspace=0.42, wspace=0.28)

        self.ax_fft_x = self.fig.add_subplot(gs[0, 0])
        self.ax_fft_y = self.fig.add_subplot(gs[1, 0])
        self.ax_fft_z = self.fig.add_subplot(gs[2, 0])
        self.ax_polar = self.fig.add_subplot(gs[:, 1], projection="polar")

        self._init_empty_plots()

        self.canvas = FigureCanvasTkAgg(self.fig, master=right_panel)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def _init_empty_plots(self):
        for ax, name in zip([self.ax_fft_x, self.ax_fft_y, self.ax_fft_z], ["X-Axis", "Y-Axis", "Z-Axis"]):
            ax.clear()
            ax.set_title(f"{name} FFT Spectrum", fontsize=9.5)
            ax.set_ylabel("Accel (g)", fontsize=8.5)
            ax.grid(True, linestyle="--", alpha=0.5)
        self.ax_fft_z.set_xlabel("Frequency (Hz)", fontsize=8.5)

        self.ax_polar.clear()
        self.ax_polar.set_title("Four-Run Vector Diagram\n(Awaiting 4 Runs)", fontsize=9.5, pad=14)

    # serial connection & non-blocking live readout loop
    def _handle_set_baudrate(self):
        """
        change baud rate of the sensor (i.e 9600 -> 115200) directly from GUI
        """
        if self.is_connected:
            messagebox.showwarning(
                "Sensor is still connected",
                "Click 'Disconnect' first before changing the baud rate sensor.",
            )
            return

        port = self.port_var.get().strip()
        current_baud = int(self.init_baud_var.get())
        target_baud = int(self.baud_var.get())

        try:
            self.status_var.set(f"Switching {current_baud} -> {target_baud}...")
            self.root.update_idletasks()
            change_sensor_baudrate(port, current_baud, target_baud)
            self.status_var.set(f"Baud rate saved: {target_baud} bps")
            messagebox.showinfo(
                "Baud Rate changed sucessfully",
                f"Sensor at {port} sucessfully changed from {current_baud} to {target_baud} bps dan saved to flash.\n\nYou may click 'Connect'.",
            )
        except Exception as e:
            self.status_var.set("Failed to change baud rate")
            messagebox.showerror("Baud Rate Error", f"Failed to change baud rate on {port}:\n{e}")

    def _toggle_connection(self):
        if not self.is_connected:
            port = self.port_var.get().strip()
            baud = int(self.baud_var.get())
            self.dev = device_model.DeviceModel("WTVB01-485", port, baud, 0x50)
            self.dev.openDevice()
            if self.dev.isOpen:
                self.dev.startLoopRead()
                self.is_connected = True
                self.conn_btn.config(text="Disconnect")
                self.set_baud_btn.config(state="disabled")
                self.status_var.set(f"Connected ({port})")
                self._poll_live_ui()
            else:
                messagebox.showerror("Connection Error", f"Could not open {port}. Check USB cable or close other apps.")
        else:
            self.is_connected = False
            if self.dev:
                self.dev.stopLoopRead()
                self.dev.closeDevice()
            self.conn_btn.config(text="Connect")
            self.set_baud_btn.config(state="normal")
            self.status_var.set("Disconnected")
            self.live_readout_var.set("Live Accel: AX=---  AY=---  AZ=---")

    def _poll_live_ui(self):
        """Runs every 150 ms via Tkinter's event loop without freezing the GUI."""
        if not self.is_connected or self.dev is None:
            return
        ax = self.dev.get("52")
        ay = self.dev.get("53")
        az = self.dev.get("54")
        if ax is not None and ay is not None and az is not None:
            self.live_readout_var.set(f"Live Accel: AX={ax:+.4f}g  AY={ay:+.4f}g  AZ={az:+.4f}g")
        self.root.after(150, self._poll_live_ui)

    # run capture & fft computation
    def _start_capture(self, run_idx):
        if not self.is_connected:
            messagebox.showwarning("Not Connected", "Please connect to the WTVB01-485 sensor first.")
            return
        if self.is_capturing:
            return

        try:
            duration = float(self.duration_var.get())
            if duration <= 0.2:
                raise ValueError
        except ValueError:
            messagebox.showerror("Invalid Input", "Capture Time must be > 0.2 seconds.")
            return

        self.is_capturing = True
        for b in self.run_btns:
            b.config(state="disabled")

        self.dev.clear_accel_buffer()
        self.status_var.set(f"Recording Run {run_idx + 1} ({duration:.1f}s)...")

        # schedule finish after `duration` seconds without freezing the GUI window
        self.root.after(int(duration * 1000), lambda: self._finish_capture(run_idx))

    def _finish_capture(self, run_idx):
        samples = self.dev.get_accel_snapshot()
        self.is_capturing = False
        for b in self.run_btns:
            b.config(state="normal")

        if len(samples) < 20:
            self.status_var.set("Capture failed (too few samples)")
            messagebox.showerror("Capture Error", f"Only {len(samples)} samples received. Check wiring/baud rate.")
            return

        fft_res = self._compute_fft(samples)
        self.run_results[run_idx] = fft_res
        self.status_var.set(f"Run {run_idx + 1} done (fs={fft_res['fs']:.1f} Hz, N={fft_res['N']})")

        self._refresh_table()
        self._plot_fft(fft_res, f"Run {run_idx + 1}")
        self._recalculate_if_ready()

    def _compute_fft(self, samples):
        data = np.array(samples)  # columns: [t, ax, ay, az]
        t = data[:, 0]
        fs = 1.0 / np.mean(np.diff(t))
        N = len(t)

        window = np.hanning(N)
        coherent_gain = np.sum(window) / N
        freqs = np.fft.rfftfreq(N, d=1.0 / fs)

        try:
            f_min = float(self.fmin_var.get())
            f_max = float(self.fmax_var.get())
        except ValueError:
            f_min, f_max = 2.0, fs / 2.0

        mask = (freqs >= max(0.5, f_min)) & (freqs <= min(f_max, fs / 2.0))
        if not np.any(mask):
            mask = freqs >= 0.5

        out = {"fs": fs, "N": N, "freqs": freqs, "axes": {}}
        for col_idx, axis_name in enumerate(["X", "Y", "Z"], start=1):
            sig = data[:, col_idx]
            sig_ac = sig - np.mean(sig)  # Remove static 1g gravity offset
            spectrum = np.abs(np.fft.rfft(sig_ac * window)) / (N * coherent_gain)
            if N > 2:
                spectrum[1:-1] *= 2.0

            valid_f = freqs[mask]
            valid_a = spectrum[mask]
            pk = int(np.argmax(valid_a))

            out["axes"][axis_name] = {
                "spectrum": spectrum,
                "max_g": float(valid_a[pk]),
                "freq_hz": float(valid_f[pk]),
            }
        return out

    def _plot_fft(self, fft_res, title_prefix):
        freqs = fft_res["freqs"]
        plot_map = [
            (self.ax_fft_x, "X", "#1f77b4"),
            (self.ax_fft_y, "Y", "#2ca02c"),
            (self.ax_fft_z, "Z", "#d62728"),
        ]
        for ax, axis_name, color in plot_map:
            ax.clear()
            info = fft_res["axes"][axis_name]
            ax.plot(freqs, info["spectrum"], color=color, lw=1.2)
            ax.plot(
                info["freq_hz"],
                info["max_g"],
                "ko",
                markersize=5,
                label=f"Max: {info['max_g']:.5f} g @ {info['freq_hz']:.2f} Hz ({info['freq_hz'] * 60:.0f} RPM)",
            )
            ax.set_title(f"{title_prefix} — {axis_name}-Axis FFT (fs = {fft_res['fs']:.1f} Hz)", fontsize=9.5)
            ax.set_ylabel("Accel (g)", fontsize=8.5)
            ax.grid(True, linestyle="--", alpha=0.5)
            ax.legend(loc="upper right", fontsize=8)

        self.ax_fft_z.set_xlabel("Frequency (Hz)", fontsize=8.5)
        self.canvas.draw()

    def _refresh_table(self):
        for item in self.tree.get_children():
            self.tree.delete(item)
        names = ["Run 1 (0)", "Run 2 (0°)", "Run 3 (120°)", "Run 4 (240°)"]
        axis_sel = self.axis_var.get()
        for idx, r in enumerate(self.run_results):
            if r is None:
                self.tree.insert("", "end", values=(names[idx], "---", "---", "---", "---"))
            else:
                x_g = r["axes"]["X"]["max_g"]
                y_g = r["axes"]["Y"]["max_g"]
                z_g = r["axes"]["Z"]["max_g"]
                hz = r["axes"][axis_sel]["freq_hz"]
                self.tree.insert(
                    "", "end", values=(names[idx], f"{x_g:.5f}", f"{y_g:.5f}", f"{z_g:.5f}", f"{hz:.1f}")
                )

    # Four-Run balancing math & polar vector visualization
    def _recalculate_if_ready(self):
        self._refresh_table()
        if any(r is None for r in self.run_results):
            return

        try:
            m_trial = float(self.m_trial_var.get())
            r_trial = float(self.r_trial_var.get())
            r_corr = float(self.r_corr_var.get())
        except ValueError:
            messagebox.showerror("Invalid Parameters", "Check Trial Mass, Trial Radius, and Correction Radius.")
            return

        axis = self.axis_var.get()
        V0, V1, V2, V3 = [r["axes"][axis]["max_g"] for r in self.run_results]

        # a1, a2, a3 = np.deg2rad(self.trial_angles)
        # A = np.array([
        #     [1.0, np.cos(a1), np.sin(a1)],
        #     [1.0, np.cos(a2), np.sin(a2)],
        #     [1.0, np.cos(a3), np.sin(a3)],
        # ])
        # b = np.array([V1 ** 2 - V0 ** 2, V2 ** 2 - V0 ** 2, V3 ** 2 - V0 ** 2])
        # x1, x2, x3 = np.linalg.solve(A, b)

        angles_rad = np.deg2rad(self.trial_angles)
        A = np.column_stack([np.ones_like(angles_rad), np.cos(angles_rad), np.sin(angles_rad)])
        b = np.array([V1 ** 2 - V0 ** 2, V2 ** 2 - V0 ** 2, V3 ** 2 - V0 ** 2])
        x1, x2, x3 = np.linalg.lstsq(A, b, rcond=None)[0]

        E_circle = np.sqrt(max(x1, 0.0))
        E_vec = np.hypot(x2, x3) / (2.0 * V0) if V0 > 0 else E_circle
        E = E_vec if E_vec > 0 else E_circle

        if E <= 1e-12:
            messagebox.showwarning(
                "Zero Change Detected",
                "All trial runs produced identical vibration amplitudes (E = 0). Try tapping/vibrating differently per run.",
            )
            return

        # Unbalance Location (heavy spot angle)
        theta_u_deg = np.rad2deg(np.arctan2(x3, x2)) % 360.0

        # initial & estimated Residual Unbalance
        U_trial = m_trial * r_trial  # g*mm
        sensitivity = E / U_trial  # g_accel / (g*mm)
        U_initial = U_trial * (V0 / E)
        res_vib_est = abs(E_vec - E_circle)
        U_residual_est = res_vib_est / sensitivity

        # position & ocation of correction mass (180° opposite heavy spot)
        theta_c_deg = (theta_u_deg + 180.0) % 360.0

        # mass & weight of correction mass
        m_corr_g = U_initial / r_corr
        weight_N = (m_corr_g / 1000.0) * 9.80665
        weight_gf = m_corr_g

        self.result_vars["unbal_loc"].set(f"{theta_u_deg:.2f}° (from 0° ref)")
        self.result_vars["init_unbal"].set(f"{U_initial:.2f} g·mm")
        self.result_vars["res_unbal"].set(f"{U_residual_est:.2f} g·mm (~{res_vib_est:.5f} g)")
        self.result_vars["corr_pos"].set(f"{theta_c_deg:.2f}° @ R = {r_corr:.1f} mm")
        self.result_vars["corr_mass"].set(f"{m_corr_g:.3f} g ({weight_N:.5f} N / {weight_gf:.3f} gf)")

        self._plot_polar(V0, V1, V2, V3, E, theta_u_deg, theta_c_deg, m_corr_g, axis)

    def _plot_polar(self, V0, V1, V2, V3, E, theta_u_deg, theta_c_deg, m_corr_g, axis):
        self.ax_polar.clear()
        self.ax_polar.set_theta_zero_location("E")
        self.ax_polar.set_theta_direction(1)

        phi = np.linspace(0, 2 * np.pi, 360)
        self.ax_polar.plot(phi, np.full_like(phi, V0), "k--", lw=1.1, label=f"Base V0={V0:.4f}g")

        colors = ["#1f77b4", "#2ca02c", "#ff7f0e"]
        for idx, (ang, Vi, col) in enumerate(zip(self.trial_angles, [V1, V2, V3], colors), start=1):
            cx = -V0 * np.cos(np.deg2rad(ang))
            cy = -V0 * np.sin(np.deg2rad(ang))
            xs = cx + Vi * np.cos(phi)
            ys = cy + Vi * np.sin(phi)
            self.ax_polar.plot(np.arctan2(ys, xs), np.hypot(xs, ys), color=col, lw=1.1,
                               label=f"Run {idx + 1} ({ang:.0f}°)")

        tu = np.deg2rad(theta_u_deg)
        tc = np.deg2rad(theta_c_deg)
        self.ax_polar.annotate("", xy=(tu, E), xytext=(0, 0), arrowprops=dict(arrowstyle="->", color="red", lw=2.2))
        self.ax_polar.plot(tu, E, "ro", label=f"Heavy Spot: {theta_u_deg:.1f}°")

        self.ax_polar.annotate("", xy=(tc, E), xytext=(0, 0), arrowprops=dict(arrowstyle="->", color="green", lw=2.2))
        self.ax_polar.plot(tc, E, "go", label=f"Correction: {m_corr_g:.2f}g @ {theta_c_deg:.1f}°")

        self.ax_polar.set_title(f"Four-Run Polar Solution ({axis}-Axis)", fontsize=9.5, pad=16)
        self.ax_polar.legend(loc="lower center", bbox_to_anchor=(0.5, -0.32), ncol=2, fontsize=7.5)
        self.canvas.draw()

    def _reset_runs(self):
        self.run_results = [None, None, None, None]
        for k in self.result_vars:
            self.result_vars[k].set("---")
        self._refresh_table()
        self._init_empty_plots()
        self.canvas.draw()
        self.status_var.set("All runs cleared.")

    def _on_close(self):
        self.is_connected = False
        if self.dev is not None:
            self.dev.stopLoopRead()
            self.dev.closeDevice()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = FourRunBalancingApp(root)
    root.mainloop()
