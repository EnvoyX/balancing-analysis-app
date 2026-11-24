import time
import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import serial
import device_model

# adopted from set_baudrate.py for GUI baud rate switching
BAUD_CODES = {
    4800: "50 06 00 04 00 01 04 4A",
    9600: "50 06 00 04 00 02 44 4B",
    19200: "50 06 00 04 00 03 85 8B",
    38400: "50 06 00 04 00 04 C4 49",
    57600: "50 06 00 04 00 05 05 89",
    115200: "50 06 00 04 00 06 45 88",
    230400: "50 06 00 04 00 07 84 48",
}
CODE_TO_BAUD = {
    1: 4800,
    2: 9600,
    3: 19200,
    4: 38400,
    5: 57600,
    6: 115200,
    7: 230400,
}

# hex commands to unlock and save registers from WTVB01-485 User's Guide
UNLOCK_CMD = bytes.fromhex("50 06 00 69 B5 88 22 A1")
SAVE_CMD = bytes.fromhex("50 06 00 00 00 00 84 44")
# modbus RTU command to read 1 holding register at address 0x0004 (BAUD) for slave ID 0x50
READ_BAUD_CMD = bytes.fromhex("50 03 00 04 00 01 C8 4A")


def _modbus_crc16(data: bytes) -> bytes:
    """
    compute standard Modbus RTU CRC-16 (little-endian).
    """
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc.to_bytes(2, byteorder="little")


def detect_sensor_baudrate(port: str) -> tuple[int, int]:
    """
    scan supported baud rates and read register 0x04 to get the stored sensor baud rate.
    returns (active_serial_baud, stored_register_baud).
    """
    # prioritize the most common baud rates first for fast detection
    scan_order = [115200, 9600, 38400, 57600, 19200, 4800, 230400]

    for baud in scan_order:
        with serial.Serial(port, baud, timeout=0.12) as ser:
            ser.reset_input_buffer()
            ser.write(READ_BAUD_CMD)
            time.sleep(0.05)
            resp = ser.read(64)

        # look for valid 7-byte Modbus response: [0x50, 0x03, 0x02, high_byte, low_byte, crc_lo, crc_hi]
        for i in range(len(resp) - 6):
            frame = resp[i: i + 7]
            if frame[0] == 0x50 and frame[1] == 0x03 and frame[2] == 0x02:
                if _modbus_crc16(frame[:5]) == frame[5:7]:
                    reg_code = (frame[3] << 8) | frame[4]
                    stored_baud = CODE_TO_BAUD.get(reg_code, baud)
                    return baud, stored_baud

    raise RuntimeError(
        f"No valid Modbus response received from sensor on {port}."
    )


def change_sensor_baudrate(port: str, current_baud: int, target_baud: int):
    """
    change WTVB01-485 sensor baud rate and persist it to flash memory.
    """
    if target_baud not in BAUD_CODES:
        raise ValueError(f"Baud rate {target_baud} is not supported by the sensor.")

    with serial.Serial(port, current_baud, timeout=0.5) as ser:
        # unlock configuration register (valid for 10s)
        ser.write(UNLOCK_CMD)
        time.sleep(0.1)

        # write target baud rate code to register 0x04
        ser.write(bytes.fromhex(BAUD_CODES[target_baud]))
        time.sleep(0.1)

    # re-open at the new baud rate, unlock, and send SAVE command (register 0x00)
    with serial.Serial(port, target_baud, timeout=0.5) as ser_new:
        ser_new.write(UNLOCK_CMD)
        time.sleep(0.1)
        ser_new.write(SAVE_CMD)
        time.sleep(0.1)


class FourRunBalancingApp:
    def __init__(self, root):
        self.root = root
        self.root.title("WTVB01-485 Four-Run Rotor Balancing Analyzer")
        self.root.geometry("1360x820")

        self.dev = None
        self.is_connected = False
        self.is_capturing = False

        # store captured results: indices 0..3 = Runs 1..4, index 4 = Optional Trim Run
        # each entry holds {"X": {"spectrum", "max_g", "freq_hz"}, ..., "fs": float, "N": int}
        self.run_results = [None, None, None, None, None]

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # build main Tkinter GUI layout (left controls + right Matplotlib plots)
    def _build_ui(self):
        # outer container for the scrollable left sidebar
        left_container = ttk.Frame(self.root)
        left_container.pack(side=tk.LEFT, fill=tk.Y)

        self.left_canvas = tk.Canvas(left_container, highlightthickness=0, borderwidth=0)
        left_scrollbar = ttk.Scrollbar(
            left_container, orient=tk.VERTICAL, command=self.left_canvas.yview
        )
        self.left_canvas.configure(yscrollcommand=left_scrollbar.set)

        left_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.left_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # inner frame placed inside the canvas to hold all control sections
        left_panel = ttk.Frame(self.left_canvas, padding=10)
        self.left_canvas_window = self.left_canvas.create_window(
            (0, 0), window=left_panel, anchor="nw"
        )

        # automatically update scroll region and match canvas width to inner panel width
        def _on_left_panel_configure(event):
            self.left_canvas.configure(scrollregion=self.left_canvas.bbox("all"))
            req_width = left_panel.winfo_reqwidth()
            if self.left_canvas.winfo_width() != req_width:
                self.left_canvas.configure(width=req_width)

        left_panel.bind("<Configure>", _on_left_panel_configure)

        # enable mouse wheel scrolling only when the cursor hovers over the left panel
        def _on_mousewheel(event):
            if event.num == 4:
                self.left_canvas.yview_scroll(-1, "units")
            elif event.num == 5:
                self.left_canvas.yview_scroll(1, "units")
            elif event.delta:
                self.left_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

        def _bind_wheel(_event):
            self.left_canvas.bind_all("<MouseWheel>", _on_mousewheel)
            self.left_canvas.bind_all("<Button-4>", _on_mousewheel)
            self.left_canvas.bind_all("<Button-5>", _on_mousewheel)

        def _unbind_wheel(_event):
            self.left_canvas.unbind_all("<MouseWheel>")
            self.left_canvas.unbind_all("<Button-4>")
            self.left_canvas.unbind_all("<Button-5>")

        left_container.bind("<Enter>", _bind_wheel)
        left_container.bind("<Leave>", _unbind_wheel)

        right_panel = ttk.Frame(self.root, padding=10)
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        # sensor connection Section
        conn_frame = ttk.LabelFrame(left_panel, text="1. Sensor Connection", padding=8)
        conn_frame.pack(fill=tk.X, pady=(0, 6))

        ttk.Label(conn_frame, text="COM Port:").grid(row=0, column=0, sticky="w", pady=2)
        self.port_var = tk.StringVar(value="COM7")
        ttk.Entry(conn_frame, textvariable=self.port_var, width=9).grid(row=0, column=1, sticky="w", padx=4)

        ttk.Label(conn_frame, text="Target Baud:").grid(row=0, column=2, sticky="w", pady=2)
        self.baud_var = tk.StringVar(value="115200")
        ttk.Combobox(
            conn_frame,
            textvariable=self.baud_var,
            values=["4800", "9600", "19200", "38400", "57600", "115200", "230400"],
            width=9,
            state="readonly",
        ).grid(row=0, column=3, sticky="w", padx=4)

        ttk.Label(conn_frame, text="Initial Baud:").grid(row=1, column=0, sticky="w", pady=2)
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
        self.set_baud_btn.grid(row=1, column=2, columnspan=2, sticky="ew", padx=4, pady=2)

        self.conn_btn = ttk.Button(conn_frame, text="Connect", command=self._toggle_connection)
        self.conn_btn.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(3, 2), padx=(0, 4))

        self.status_var = tk.StringVar(value="Status: Disconnected")
        ttk.Label(conn_frame, textvariable=self.status_var, foreground="gray").grid(
            row=2, column=2, columnspan=2, sticky="w", pady=(3, 2)
        )

        self.detect_baud_btn = ttk.Button(
            conn_frame,
            text="Detect Sensor Baud",
            command=self._handle_detect_baudrate,
        )
        self.detect_baud_btn.grid(
            row=3, column=0, columnspan=2, sticky="ew", pady=(3, 0), padx=(0, 4)
        )

        self.detected_baud_var = tk.StringVar(value="Sensor Baud: Unknown")
        ttk.Label(
            conn_frame,
            textvariable=self.detected_baud_var,
            font=("Segoe UI", 9, "bold"),
            foreground="#004a99",
        ).grid(row=3, column=2, columnspan=2, sticky="w", pady=(3, 0))

        self.live_readout_var = tk.StringVar(
            value="Live Accel: AX=---  AY=---  AZ=---"
        )
        ttk.Label(
            conn_frame, textvariable=self.live_readout_var, font=("Consolas", 9)
        ).grid(row=4, column=0, columnspan=4, sticky="w", pady=(3, 0))

        # balancing, custom angles, ISO 1940-1 & FFT parameters Section
        param_frame = ttk.LabelFrame(left_panel, text="2. Balancing, ISO 1940-1 & FFT Settings", padding=8)
        param_frame.pack(fill=tk.X, pady=(0, 6))

        self.m_trial_var = tk.StringVar(value="10.0")
        self.r_trial_var = tk.StringVar(value="50.0")
        self.r_corr_var = tk.StringVar(value="50.0")
        self.m_rotor_var = tk.StringVar(value="2.5")
        self.iso_grade_var = tk.StringVar(value="6.3")
        self.duration_var = tk.StringVar(value="4.0")
        self.fmin_var = tk.StringVar(value="2.0")
        self.fmax_var = tk.StringVar(value="60.0")
        self.axis_var = tk.StringVar(value="Z")  # Z is convenient for desk testing; X/Y for rotor radial

        # custom trial mass placement angles (degrees)
        self.angle1_var = tk.StringVar(value="0.0")
        self.angle2_var = tk.StringVar(value="120.0")
        self.angle3_var = tk.StringVar(value="240.0")

        fields = [
            ("Trial Mass mt (g):", self.m_trial_var, "Trial Radius rt (mm):", self.r_trial_var),
            ("Corr. Radius rc (mm):", self.r_corr_var, "Rotor Mass M (kg):", self.m_rotor_var),
            ("Capture Time (s):", self.duration_var, "Min Freq (Hz):", self.fmin_var),
        ]
        for r_idx, (l1, v1, l2, v2) in enumerate(fields):
            ttk.Label(param_frame, text=l1).grid(row=r_idx, column=0, sticky="w", pady=2)
            ttk.Entry(param_frame, textvariable=v1, width=8).grid(row=r_idx, column=1, sticky="w", padx=4)
            ttk.Label(param_frame, text=l2).grid(row=r_idx, column=2, sticky="w", pady=2)
            ttk.Entry(param_frame, textvariable=v2, width=8).grid(row=r_idx, column=3, sticky="w", padx=4)

        # max freq and balancing axis row
        ttk.Label(param_frame, text="Max Freq (Hz):").grid(row=3, column=0, sticky="w", pady=2)
        ttk.Entry(param_frame, textvariable=self.fmax_var, width=8).grid(row=3, column=1, sticky="w", padx=4)

        ttk.Label(param_frame, text="Balancing Axis:").grid(row=3, column=2, sticky="w", pady=2)
        axis_box = ttk.Combobox(
            param_frame, textvariable=self.axis_var, values=["X", "Y", "Z"], width=6, state="readonly"
        )
        axis_box.grid(row=3, column=3, sticky="w", padx=4)
        axis_box.bind("<<ComboboxSelected>>", lambda e: self._recalculate_if_ready())

        # ISO 1940-1 balance quality Grade (G) selector (editable for custom G values)
        ttk.Label(param_frame, text="ISO Grade G (mm/s):").grid(row=4, column=0, sticky="w", pady=2)
        iso_box = ttk.Combobox(
            param_frame,
            textvariable=self.iso_grade_var,
            values=["0.4", "1.0", "2.5", "6.3", "16.0", "40.0"],
            width=7,
            state="normal",
        )
        iso_box.grid(row=4, column=1, sticky="w", padx=4)
        iso_box.bind("<<ComboboxSelected>>", lambda e: self._recalculate_if_ready())
        iso_box.bind("<Return>", lambda e: self._recalculate_if_ready())

        ttk.Label(param_frame, text="(Standard or custom G)", foreground="gray").grid(
            row=4, column=2, columnspan=2, sticky="w", pady=2
        )

        # custom trial angles sub-row
        angle_frame = ttk.Frame(param_frame)
        angle_frame.grid(row=5, column=0, columnspan=4, sticky="ew", pady=(4, 0))

        ttk.Label(angle_frame, text="Trial Angles (°):").pack(side=tk.LEFT)
        for var in (self.angle1_var, self.angle2_var, self.angle3_var):
            entry = ttk.Entry(angle_frame, textvariable=var, width=6)
            entry.pack(side=tk.LEFT, padx=3)
            entry.bind("<FocusOut>", lambda e: self._on_angles_changed())
            entry.bind("<Return>", lambda e: self._on_angles_changed())

        ttk.Button(
            angle_frame, text="0/120/240", width=9, command=lambda: self._set_angle_preset(0.0, 120.0, 240.0)
        ).pack(side=tk.LEFT, padx=(4, 2))
        ttk.Button(
            angle_frame, text="0/90/180", width=8, command=lambda: self._set_angle_preset(0.0, 90.0, 180.0)
        ).pack(side=tk.LEFT)

        # Four-Run & verification data acquisition section
        run_frame = ttk.LabelFrame(left_panel, text="3. Four-Run Data Acquisition", padding=8)
        run_frame.pack(fill=tk.X, pady=(0, 6))

        self.run_btns = []
        for idx in range(5):
            btn = ttk.Button(run_frame, text="", command=lambda i=idx: self._start_capture(i))
            btn.pack(fill=tk.X, pady=1.5)
            self.run_btns.append(btn)
        self._update_run_button_labels()

        btn_row = ttk.Frame(run_frame)
        btn_row.pack(fill=tk.X, pady=(4, 0))
        ttk.Button(btn_row, text="Calculate & Check ISO", command=self._recalculate_if_ready).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 4)
        )
        ttk.Button(btn_row, text="Reset Runs", command=self._reset_runs).pack(
            side=tk.RIGHT, fill=tk.X, expand=True
        )

        # summary table of captured peak accelerations
        cols = ("run", "x_max", "y_max", "z_max", "freq")
        self.tree = ttk.Treeview(run_frame, columns=cols, show="headings", height=5)
        self.tree.heading("run", text="Run")
        self.tree.heading("x_max", text="X Max (g)")
        self.tree.heading("y_max", text="Y Max (g)")
        self.tree.heading("z_max", text="Z Max (g)")
        self.tree.heading("freq", text="Peak Hz")
        for c, w in zip(cols, [78, 74, 74, 74, 62]):
            self.tree.column(c, width=w, anchor="center")
        self.tree.pack(fill=tk.X, pady=(5, 0))
        self._refresh_table()

        # balancing analysis & ISO 1940-1 status section
        out_frame = ttk.LabelFrame(left_panel, text="4. Balancing Results & ISO 1940-1", padding=8)
        out_frame.pack(fill=tk.X, pady=(0, 8))

        self.result_vars = {
            "unbal_loc": tk.StringVar(value="---"),
            "init_unbal": tk.StringVar(value="---"),
            "res_unbal": tk.StringVar(value="---"),
            "corr_pos": tk.StringVar(value="---"),
            "corr_mass": tk.StringVar(value="---"),
            "iso_limit": tk.StringVar(value="---"),
        }

        result_rows = [
            ("Unbalance Location:", "unbal_loc"),
            ("Initial Unbalance:", "init_unbal"),
            ("Residual Unbalance:", "res_unbal"),
            ("Correction Position:", "corr_pos"),
            ("Correction Mass:", "corr_mass"),
            ("ISO 1940-1 Limit (Uper):", "iso_limit"),
        ]
        for r_idx, (label_text, key) in enumerate(result_rows):
            ttk.Label(out_frame, text=label_text, font=("Segoe UI", 9, "bold")).grid(
                row=r_idx, column=0, sticky="w", pady=2
            )
            ttk.Label(
                out_frame,
                textvariable=self.result_vars[key],
                font=("Consolas", 9, "bold"),
                foreground="#004a99",
            ).grid(row=r_idx, column=1, sticky="w", padx=6, pady=2)

        # ISO 1940-1 Pass/Fail status banner
        ttk.Label(out_frame, text="ISO 1940-1 Status:", font=("Segoe UI", 9, "bold")).grid(
            row=len(result_rows), column=0, sticky="w", pady=(4, 2)
        )
        self.iso_status_label = tk.Label(
            out_frame,
            text="AWAITING DATA",
            font=("Segoe UI", 9, "bold"),
            bg="#e0e0e0",
            fg="#333333",
            padx=8,
            pady=3,
            relief=tk.GROOVE,
        )
        self.iso_status_label.grid(row=len(result_rows), column=1, sticky="ew", padx=6, pady=(4, 2))

        # right panel: embedded Matplotlib figure (3 FFT plots + 1 Polar plot)
        self.fig = Figure(figsize=(8.6, 7.8), dpi=100)
        gs = self.fig.add_gridspec(3, 2, width_ratios=[1.4, 1.0], hspace=0.42, wspace=0.28)

        self.ax_fft_x = self.fig.add_subplot(gs[0, 0])
        self.ax_fft_y = self.fig.add_subplot(gs[1, 0])
        self.ax_fft_z = self.fig.add_subplot(gs[2, 0])
        self.ax_polar = self.fig.add_subplot(gs[:, 1], projection="polar")

        self._init_empty_plots()

        self.canvas = FigureCanvasTkAgg(self.fig, master=right_panel)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def _get_trial_angles(self) -> tuple[float, float, float]:
        """
        parse the three custom trial mass angles from GUI inputs.
        """
        a1 = float(self.angle1_var.get())
        a2 = float(self.angle2_var.get())
        a3 = float(self.angle3_var.get())
        return (a1, a2, a3)

    def _set_angle_preset(self, a1: float, a2: float, a3: float):
        """
        apply preset trial mass angles and refresh labels/calculations.
        """
        self.angle1_var.set(f"{a1:.1f}")
        self.angle2_var.set(f"{a2:.1f}")
        self.angle3_var.set(f"{a3:.1f}")
        self._on_angles_changed()

    def _on_angles_changed(self):
        """
        update button labels, table rows, and recalculate when custom angles change.
        """
        self._update_run_button_labels()
        self._refresh_table()
        self._recalculate_if_ready()

    def _update_run_button_labels(self):
        """
        synchronize capture button text with the current custom trial angles.
        """
        try:
            a1, a2, a3 = self._get_trial_angles()
            angle_strs = [f"{a1:g}°", f"{a2:g}°", f"{a3:g}°"]
        except ValueError:
            angle_strs = ["Angle 1", "Angle 2", "Angle 3"]

        labels = [
            "Capture Run 1 (Initial - No Trial Mass)",
            f"Capture Run 2 (Trial Mass @ {angle_strs[0]})",
            f"Capture Run 3 (Trial Mass @ {angle_strs[1]})",
            f"Capture Run 4 (Trial Mass @ {angle_strs[2]})",
            "Capture Trim Run (Optional - With Correction Mass)",
        ]
        for btn, text in zip(self.run_btns, labels):
            btn.config(text=text)

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
        change sensor baud rate (e.g., 9600 -> 115200) directly from the GUI.
        """
        if self.is_connected:
            messagebox.showwarning(
                "Sensor Still Connected",
                "Click 'Disconnect' first before changing the sensor baud rate.",
            )
            return

        port = self.port_var.get().strip()
        current_baud = int(self.init_baud_var.get())
        target_baud = int(self.baud_var.get())

        try:
            self.status_var.set(f"Switching {current_baud} -> {target_baud}...")
            self.root.update_idletasks()
            change_sensor_baudrate(port, current_baud, target_baud)
            self.init_baud_var.set(str(target_baud))
            self.detected_baud_var.set(f"Sensor Baud: {target_baud} bps")
            self.status_var.set(f"Baud rate saved: {target_baud} bps")
            messagebox.showinfo(
                "Baud Rate Changed",
                f"Sensor on {port} changed from {current_baud} to {target_baud} bps and saved to flash.\n\nYou may now click 'Connect'.",
            )
        except Exception as e:
            self.status_var.set("Failed to change baud rate")
            messagebox.showerror("Baud Rate Error", f"Failed to change baud rate on {port}:\n{e}")

    def _handle_detect_baudrate(self):
        """
        scan COM port to identify and display the baud rate saved in sensor register 0x04.
        """
        if self.is_connected:
            messagebox.showwarning(
                "Sensor Still Connected",
                "Click 'Disconnect' first before scanning the sensor baud rate.",
            )
            return

        port = self.port_var.get().strip()
        try:
            self.status_var.set(f"Scanning {port}...")
            self.root.update_idletasks()
            active_baud, stored_baud = detect_sensor_baudrate(port)

            # Sync GUI dropdowns and display label with the detected sensor baud rate
            self.init_baud_var.set(str(active_baud))
            self.baud_var.set(str(stored_baud))
            self.detected_baud_var.set(f"Sensor Baud: {stored_baud} bps")
            self.status_var.set(f"Detected {stored_baud} bps on {port}")
        except Exception as e:
            self.detected_baud_var.set("Sensor Baud: Not Found")
            self.status_var.set("Baud detection failed")
            messagebox.showerror("Detection Error", str(e))

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
                self.detect_baud_btn.config(state="disabled")
                self.detected_baud_var.set(f"Sensor Baud: {baud} bps")
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
            self.detect_baud_btn.config(state="normal")
            self.status_var.set("Disconnected")
            self.live_readout_var.set("Live Accel: AX=---  AY=---  AZ=---")

    def _poll_live_ui(self):
        """
        poll sensor registers every 150 ms via Tkinter's event loop without blocking the GUI.
        """
        if not self.is_connected or self.dev is None:
            return
        ax = self.dev.get("52")
        ay = self.dev.get("53")
        az = self.dev.get("54")
        if ax is not None and ay is not None and az is not None:
            self.live_readout_var.set(f"Live Accel: AX={ax:+.4f}g  AY={ay:+.4f}g  AZ={az:+.4f}g")
        self.root.after(150, self._poll_live_ui)

    # data capture & FFT computation
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
        run_label = "Trim Run" if run_idx == 4 else f"Run {run_idx + 1}"
        self.status_var.set(f"Recording {run_label} ({duration:.1f}s)...")

        # schedule capture completion after `duration` seconds without freezing the UI
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
        run_label = "Trim Run" if run_idx == 4 else f"Run {run_idx + 1}"
        self.status_var.set(f"{run_label} done (fs={fft_res['fs']:.1f} Hz, N={fft_res['N']})")

        self._refresh_table()
        self._plot_fft(fft_res, run_label)
        self._recalculate_if_ready()

    def _compute_fft(self, samples):
        data = np.array(samples)  # Columns: [t, ax, ay, az]
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
            sig_ac = sig - np.mean(sig)  # Remove static 1g DC gravity offset
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

        try:
            a1, a2, a3 = self._get_trial_angles()
            names = ["Run 1 (Init)", f"Run 2 ({a1:g}°)", f"Run 3 ({a2:g}°)", f"Run 4 ({a3:g}°)", "Trim Run"]
        except ValueError:
            names = ["Run 1 (Init)", "Run 2", "Run 3", "Run 4", "Trim Run"]

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

    # Four-Run balancing math, ISO 1940-1 evaluation & polar diagram
    def _recalculate_if_ready(self):
        self._refresh_table()
        # require at least the first 4 runs (Initial + 3 Trial Runs)
        if any(r is None for r in self.run_results[:4]):
            return

        try:
            m_trial = float(self.m_trial_var.get())
            r_trial = float(self.r_trial_var.get())
            r_corr = float(self.r_corr_var.get())
            m_rotor = float(self.m_rotor_var.get())
            g_grade = float(self.iso_grade_var.get())
            trial_angles = self._get_trial_angles()
            if m_trial <= 0 or r_trial <= 0 or r_corr <= 0 or m_rotor <= 0 or g_grade <= 0:
                raise ValueError("Physical parameters and ISO Grade G must be positive.")
        except ValueError as err:
            messagebox.showerror("Invalid Parameters", f"Check balancing, angle, or ISO parameters:\n{err}")
            return

        axis = self.axis_var.get()
        V0, V1, V2, V3 = [r["axes"][axis]["max_g"] for r in self.run_results[:4]]

        # build 3x3 linear system for custom trial angles:
        # V_i^2 - V_0^2 = E^2 + (2*V0*E*cos(theta_u))*cos(theta_i) + (2*V0*E*sin(theta_u))*sin(theta_i)
        angles_rad = np.deg2rad(trial_angles)
        A = np.column_stack([np.ones_like(angles_rad), np.cos(angles_rad), np.sin(angles_rad)])

        # reject duplicate or degenerate trial angles where det(A) == 0
        if abs(np.linalg.det(A)) < 1e-8:
            messagebox.showerror(
                "Invalid Trial Angles",
                "The three trial mass angles must be distinct (non-collinear) to solve the Four-Run system.",
            )
            return

        b = np.array([V1 ** 2 - V0 ** 2, V2 ** 2 - V0 ** 2, V3 ** 2 - V0 ** 2])
        x1, x2, x3 = np.linalg.solve(A, b)

        E_circle = np.sqrt(max(x1, 0.0))
        E_vec = np.hypot(x2, x3) / (2.0 * V0) if V0 > 0 else E_circle
        E = E_vec if E_vec > 0 else E_circle

        if E <= 1e-12:
            messagebox.showwarning(
                "Zero Change Detected",
                "All trial runs produced identical vibration amplitudes (E = 0).",
            )
            return

        # heavy spot angular location (degrees)
        theta_u_deg = np.rad2deg(np.arctan2(x3, x2)) % 360.0

        # initial unbalance and system influence sensitivity
        U_trial = m_trial * r_trial  # g*mm
        sensitivity = E / U_trial  # g_accel / (g*mm)
        U_initial = U_trial * (V0 / E)

        # use measured Trim Run (Run 5) if available; otherwise use Four-Run consistency residual
        trim_run = self.run_results[4]
        if trim_run is not None:
            V_res = trim_run["axes"][axis]["max_g"]
            U_residual = V_res / sensitivity
            res_source = "Trim"
        else:
            V_res = abs(E_vec - E_circle)
            U_residual = V_res / sensitivity
            res_source = "Est."

        # correction mass position (180 degrees opposite the heavy spot) and mass
        theta_c_deg = (theta_u_deg + 180.0) % 360.0
        m_corr_g = U_initial / r_corr
        weight_N = (m_corr_g / 1000.0) * 9.80665
        weight_gf = m_corr_g

        # ISO 1940-1 permissible residual unbalance calculation
        # average dominant 1X frequency across the captured runs
        freqs_1x = [r["axes"][axis]["freq_hz"] for r in self.run_results[:4]]
        mean_freq_hz = float(np.mean(freqs_1x))
        rpm = mean_freq_hz * 60.0
        omega = 2.0 * np.pi * mean_freq_hz  # rad/s

        # e_per (g*mm/kg or um) = (G * 1000) / omega; U_per (g*mm) = e_per * M_rotor (kg)
        e_per = (g_grade * 1000.0) / omega if omega > 0 else 0.0
        U_per = e_per * m_rotor
        m_per = U_per / r_corr
        V_per = U_per * sensitivity  # Equivalent permissible vibration amplitude (g)

        is_passed = U_residual <= U_per

        # update numerical result fields
        self.result_vars["unbal_loc"].set(f"{theta_u_deg:.2f}° (from 0° ref)")
        self.result_vars["init_unbal"].set(f"{U_initial:.2f} g·mm ({V0:.5f} g)")
        self.result_vars["res_unbal"].set(f"{U_residual:.2f} g·mm ({res_source}: {V_res:.5f} g)")
        self.result_vars["corr_pos"].set(f"{theta_c_deg:.2f}° @ R = {r_corr:.1f} mm")
        self.result_vars["corr_mass"].set(f"{m_corr_g:.3f} g ({weight_N:.5f} N / {weight_gf:.3f} gf)")
        self.result_vars["iso_limit"].set(f"{U_per:.2f} g·mm ({m_per:.3f} g @ {rpm:.0f} RPM)")

        # update ISO 1940-1 Pass/Fail status badge
        if is_passed:
            self.iso_status_label.config(
                text=f"PASS — ISO 1940-1 G{g_grade:g} ({U_residual:.1f} ≤ {U_per:.1f} g·mm)",
                bg="#1b7a3d",
                fg="white",
            )
        else:
            self.iso_status_label.config(
                text=f"FAIL — ISO 1940-1 G{g_grade:g} ({U_residual:.1f} > {U_per:.1f} g·mm)",
                bg="#b82525",
                fg="white",
            )

        self._plot_polar(V0, [V1, V2, V3], trial_angles, E, theta_u_deg, theta_c_deg, m_corr_g, V_per, g_grade, axis)

    def _plot_polar(self, V0, v_trials, trial_angles, E, theta_u_deg, theta_c_deg, m_corr_g, V_per, g_grade, axis):
        self.ax_polar.clear()
        self.ax_polar.set_theta_zero_location("E")
        self.ax_polar.set_theta_direction(1)

        phi = np.linspace(0, 2 * np.pi, 360)
        self.ax_polar.plot(phi, np.full_like(phi, V0), "k--", lw=1.1, label=f"Base V0={V0:.4f}g")

        # plot ISO 1940-1 permissible vibration tolerance boundary circle
        if V_per > 0:
            self.ax_polar.plot(
                phi, np.full_like(phi, V_per), color="#9b59b6", linestyle=":", lw=1.4, label=f"ISO G{g_grade:g} Limit"
            )

        colors = ["#1f77b4", "#2ca02c", "#ff7f0e"]
        for idx, (ang, Vi, col) in enumerate(zip(trial_angles, v_trials, colors), start=1):
            cx = -V0 * np.cos(np.deg2rad(ang))
            cy = -V0 * np.sin(np.deg2rad(ang))
            xs = cx + Vi * np.cos(phi)
            ys = cy + Vi * np.sin(phi)
            self.ax_polar.plot(
                np.arctan2(ys, xs), np.hypot(xs, ys), color=col, lw=1.1, label=f"Run {idx + 1} ({ang:g}°)"
            )

        tu = np.deg2rad(theta_u_deg)
        tc = np.deg2rad(theta_c_deg)
        self.ax_polar.annotate("", xy=(tu, E), xytext=(0, 0), arrowprops=dict(arrowstyle="->", color="red", lw=2.2))
        self.ax_polar.plot(tu, E, "ro", label=f"Heavy Spot: {theta_u_deg:.1f}°")

        self.ax_polar.annotate("", xy=(tc, E), xytext=(0, 0), arrowprops=dict(arrowstyle="->", color="green", lw=2.2))
        self.ax_polar.plot(tc, E, "go", label=f"Correction: {m_corr_g:.2f}g @ {theta_c_deg:.1f}°")

        self.ax_polar.set_title(f"Four-Run Polar Solution ({axis}-Axis)", fontsize=9.5, pad=16)
        self.ax_polar.legend(loc="lower center", bbox_to_anchor=(0.5, -0.34), ncol=2, fontsize=7.5)
        self.canvas.draw()

    def _reset_runs(self):
        self.run_results = [None, None, None, None, None]
        for k in self.result_vars:
            self.result_vars[k].set("---")
        self.iso_status_label.config(text="AWAITING DATA", bg="#e0e0e0", fg="#333333")
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
