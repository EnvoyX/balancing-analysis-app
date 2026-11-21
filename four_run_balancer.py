import time
import numpy as np
import matplotlib.pyplot as plt
import device_model

# configuration
PORT = "COM7"
BAUD_RATE = 115200
MODBUS_ID = 0x50

CAPTURE_SECONDS = 4.0  # recording duration per run (seconds)
TRIAL_ANGLES_DEG = (0.0, 120.0, 240.0)  # three equally spaced trial angles
FREQ_BANDS_HZ = (2.0, 60.0)  # frequency search band (Hz)


# data capture
def capture_run(dev, run_label, duration_s):
    """
    clears old samples, records `duration_s` seconds of AX/AY/AZ, and runs FFT.
    """
    dev.clear_accel_buffer()
    print(f"Recording {run_label} for {duration_s:.1f}s... keep vibration/RPM steady")
    time.sleep(duration_s)

    samples = dev.get_accel_snapshot()
    if len(samples) < 20:
        raise RuntimeError(
            f"Only {len(samples)} samples were captured for {run_label}. Check COM port ({PORT}), baudrate ({BAUD_RATE})"
        )
    fft_res = compute_triaxial_fft(samples, freq_band=FREQ_BANDS_HZ)

    print(f"\n--- {run_label} Peak Acceleration Summary (fs = {fft_res['fs']:.1f} Hz, N = {fft_res['N']}) ---")
    for axis in ["X", "Y", "Z"]:
        a = fft_res["axes"][axis]
        print(
            f"  {axis}-Axis Max Accel: {a['peak_accel_g']:.5f} g  @ {a['peak_freq_hz']:.2f} Hz ({a['peak_rpm']:.0f} RPM)"
        )

    plot_run_fft(fft_res, run_label)
    return fft_res


# fft spectrum & peak extraction
def compute_triaxial_fft(samples, freq_band=None):
    """
    computes single-sided amplitude FFT (in g) for X, Y, Z axes
    and extracts the largest acceleration peak and frequency from each axis.
    :param samples:
    :param freq_band:
    :return:
    """
    data = np.array(samples)
    t = data[:, 0]
    dt = np.diff(t)
    fs = 1.0 / np.mean(dt)
    N = len(t)

    # hann window + coherent gain correction for accurate peak amplitude
    window = np.hanning(N)
    coherent_gain = np.sum(window) / N

    freqs = np.fft.rfftfreq(N, d=1.0 / fs)
    results = {"fs": fs, "N": N, "freqs": freqs, "axes": {}}

    for col_idx, axis_name in enumerate(["X", 'Y', 'Z'], start=1):
        sig = data[:, col_idx]
        sig_ac = sig - np.mean(sig)  # remove DC component (e.g., 1g gravity on Z-axis)

        fft_vals = np.fft.rfft(sig_ac * window)
        amp = np.abs(fft_vals) / (N * coherent_gain)
        if N > 2:
            amp[1:-1] *= 2.0  # single-sided spectrum scaling
        if freq_band is not None:
            mask = (freqs >= freq_band[0]) & (freqs <= min(freq_band[1], fs / 2.0))
        else:
            mask = freqs >= 1.0
        if not np.any(mask):
            mask = freqs >= 0.5
        valid_freqs = freqs[mask]
        valid_amps = amp[mask]
        peak_idx = int(np.argmax(valid_amps))

        peak_amp = float(valid_amps[peak_idx])
        peak_freq = float(valid_freqs[peak_idx])

        results["axes"][axis_name] = {
            "amp": amp,
            "peak_accel_g": peak_amp,
            "peak_freq_hz": peak_freq,
            "peak_rpm": peak_freq * 60.0,
        }

    return results


# fft plotter
def plot_run_fft(fft_res, run_label):
    """
    plots the X, Y, Z FFT frequency spectra and highlights the max peak per axis.
    """
    freqs = fft_res["freqs"]
    fig, axes = plt.subplots(3, 1, figsize=(9.5, 6.8), sharex=True)
    colors = {"X": "#1f77b4", "Y": "#2ca02c", "Z": "#d62728"}

    for ax_plot, axis_name in zip(axes, ["X", "Y", "Z"]):
        info = fft_res["axes"][axis_name]
        ax_plot.plot(freqs, info["amp"], color=colors[axis_name], lw=1.2)
        ax_plot.plot(
            info["peak_freq_hz"],
            info["peak_accel_g"],
            "ko",
            markersize=6,
            label=f"Max: {info['peak_accel_g']:.5f} g @ {info['peak_freq_hz']:.2f} Hz ({info['peak_rpm']:.0f} RPM)",
        )
        ax_plot.set_ylabel(f"{axis_name}-Accel (g)")
        ax_plot.grid(True, linestyle="--", alpha=0.5)
        ax_plot.legend(loc="upper right")

    axes[0].set_title(f"{run_label} — Triaxial Frequency Spectrum (fs = {fft_res['fs']:.1f} Hz)")
    axes[-1].set_xlabel("Frequency (Hz)")
    plt.tight_layout()
    plt.show(block=False)
    plt.pause(0.2)


# four-run method balancing solver
def calculate_four_run_balancing(V0, V1, V2, V3, m_trial_g, r_trial_mm, r_corr_mm, angles_deg=(0.0, 120.0, 240.0)):
    """
    calculates unbalance location, initial & residual unbalance, correction mass
    location, and correction mass/weight using the Four-Run (amplitude-only) method.
    """
    a1, a2, a3 = np.deg2rad(angles_deg)

    # solve linear system derived from Law of Cosines across the 3 trial positions:
    # Vi^2 - V0^2 = E^2 + (2*V0*E*cos(theta_u))*cos(ai) + (2*V0*E*sin(theta_u))*sin(ai)
    A = np.array([
        [1.0, np.cos(a1), np.sin(a1)],
        [1.0, np.cos(a2), np.sin(a2)],
        [1.0, np.cos(a3), np.sin(a3)],
    ])
    b = np.array([V1 ** 2 - V0 ** 2, V2 ** 2 - V0 ** 2, V3 ** 2 - V0 ** 2])
    x1, x2, x3 = np.linalg.solve(A, b)

    # trial mass vibration effect E (vector-derived E handles noisy measurements safely)
    E_circle = np.sqrt(max(x1, 0.0))
    E_vec = np.hypot(x2, x3) / (2.0 * V0) if V0 > 0 else E_circle
    E = E_vec if E_vec > 0 else E_circle

    if E <= 1e-12:
        raise ValueError("Zero vibration change detected between runs (E = 0). Vary vibration/trial mass between runs.")

    # Unbalance location (heavy spot angle from 0° reference)
    theta_u_deg = np.rad2deg(np.arctan2(x3, x2)) % 360.0

    # initial & residual Unbalance
    U_trial_g_mm = m_trial_g * r_trial_mm
    sensitivity_g_per_gmm = E / U_trial_g_mm
    U_initial_g_mm = U_trial_g_mm * (V0 / E)

    # circle-closure mismatch estimates residual unbalance floor after correction
    residual_vib_est_g = abs(E_vec - E_circle)
    residual_unbalance_est_g_mm = residual_vib_est_g / sensitivity_g_per_gmm

    # position & location of correction mass (180° opposite heavy spot)
    theta_c_deg = (theta_u_deg + 180.0) % 360.0

    # mass & weight of correction mass
    m_corr_g = U_initial_g_mm / r_corr_mm
    g_std = 9.80665
    weight_N = (m_corr_g / 1000.0) * g_std
    weight_gf = m_corr_g

    return {
        "V0": V0,
        "V1": V1,
        "V2": V2,
        "V3": V3,
        "E_effect_g": E,
        "unbalance_angle_deg": theta_u_deg,
        "initial_unbalance_g_mm": U_initial_g_mm,
        "residual_unbalance_est_g_mm": residual_unbalance_est_g_mm,
        "residual_vibration_est_g": residual_vib_est_g,
        "correction_angle_deg": theta_c_deg,
        "correction_radius_mm": r_corr_mm,
        "correction_mass_g": m_corr_g,
        "correction_weight_N": weight_N,
        "correction_weight_gf": weight_gf,
    }


# four-run polar circle-intersection plotter
def plot_four_run_polar(res, angles_deg=(0.0, 120.0, 240.0)):
    """
    plots the graphical Four-Run circles and unbalance/correction vectors.
    """
    V0, V1, V2, V3 = res["V0"], res["V1"], res["V2"], res["V3"]
    E = res["E_effect_g"]
    theta_u = np.deg2rad(res["unbalance_angle_deg"])
    theta_c = np.deg2rad(res["correction_angle_deg"])

    fig, ax = plt.subplots(subplot_kw={"projection": "polar"}, figsize=(7.5, 7.5))
    ax.set_theta_zero_location("E")
    ax.set_theta_direction(1)

    phi = np.linspace(0, 2 * np.pi, 360)
    ax.plot(phi, np.full_like(phi, V0), "k--", lw=1.2, label=f"Base Circle V0 = {V0:.5f} g")

    circle_colors = ["#1f77b4", "#2ca02c", "#ff7f0e"]
    for idx, (ang_d, Vi, col) in enumerate(zip(angles_deg, [V1, V2, V3], circle_colors), start=1):
        cx = -V0 * np.cos(np.deg2rad(ang_d))
        cy = -V0 * np.sin(np.deg2rad(ang_d))
        xs = cx + Vi * np.cos(phi)
        ys = cy + Vi * np.sin(phi)
        ax.plot(np.arctan2(ys, xs), np.hypot(xs, ys), color=col, lw=1.3,
                label=f"Run {idx} ({ang_d:.0f}°): V{idx} = {Vi:.5f} g")

    # heavy spot (red) & correction mass (green) vectors
    ax.annotate("", xy=(theta_u, E), xytext=(0, 0), arrowprops=dict(arrowstyle="->", color="red", lw=2.5))
    ax.plot(theta_u, E, "ro", label=f"Unbalance Heavy Spot: {res['unbalance_angle_deg']:.1f}°")

    ax.annotate("", xy=(theta_c, E), xytext=(0, 0), arrowprops=dict(arrowstyle="->", color="green", lw=2.5))
    ax.plot(
        theta_c,
        E,
        "go",
        label=f"Correction Mass: {res['correction_mass_g']:.3f} g @ {res['correction_angle_deg']:.1f}°",
    )

    ax.set_title("Four-Run Method — Circle Intersection & Balancing Vectors", pad=20)
    ax.legend(loc="upper right", bbox_to_anchor=(1.35, 1.12), fontsize=8.5)
    plt.tight_layout()
    plt.show()


def main():
    print("==========================================================")
    print("   WTVB01-485 FOUR-RUN ROTOR BALANCING SYSTEM")
    print("==========================================================")

    m_trial = float(input("Enter Trial Mass (grams, e.g. 10.0): "))
    r_trial = float(input("Enter Trial Mass Radius (mm, e.g. 50.0): "))
    r_corr_in = input(f"Enter Correction Mass Radius (mm, press Enter for {r_trial}): ").strip()
    r_corr = float(r_corr_in) if r_corr_in else r_trial

    axis_choice = input("Select axis for balancing [X / Y / Z] (use Z on desk/PC, X/Y on rotor): ").strip().upper()
    if axis_choice not in ("X", "Y", "Z"):
        axis_choice = "Z"

    dev = device_model.DeviceModel("WTVB01-485", PORT, BAUD_RATE, MODBUS_ID)
    dev.openDevice()
    dev.startLoopRead()
    time.sleep(0.5)

    run_prompts = [
        "Run 0: Initial Run (NO trial mass attached)",
        f"Run 1: Trial mass ({m_trial} g) at {TRIAL_ANGLES_DEG[0]}° (Reference Mark)",
        f"Run 2: Move trial mass ({m_trial} g) to {TRIAL_ANGLES_DEG[1]}°",
        f"Run 3: Move trial mass ({m_trial} g) to {TRIAL_ANGLES_DEG[2]}°",
    ]

    peak_V = []
    try:
        for i, desc in enumerate(run_prompts):
            input(f"\n[Step {i + 1}/4] {desc}\nPress ENTER to start capturing Run {i}...")
            fft_res = capture_run(dev, f"Run {i}", CAPTURE_SECONDS)
            peak_V.append(fft_res["axes"][axis_choice]["peak_accel_g"])

        bal = calculate_four_run_balancing(
            V0=peak_V[0],
            V1=peak_V[1],
            V2=peak_V[2],
            V3=peak_V[3],
            m_trial_g=m_trial,
            r_trial_mm=r_trial,
            r_corr_mm=r_corr,
            angles_deg=TRIAL_ANGLES_DEG,
        )

        print("\n==========================================================")
        print(f"   FOUR-RUN BALANCING RESULTS (Based on {axis_choice}-Axis)")
        print("==========================================================")
        print(f"1. Unbalance Location (Heavy Spot) : {bal['unbalance_angle_deg']:.2f}° (from 0° reference)")
        print(f"2. Initial Unbalance (U0)          : {bal['initial_unbalance_g_mm']:.2f} g·mm")
        print(
            f"   Residual Unbalance (Est.)       : {bal['residual_unbalance_est_g_mm']:.2f} g·mm (~{bal['residual_vibration_est_g']:.5f} g)")
        print(
            f"3. Correction Mass Position        : {bal['correction_angle_deg']:.2f}° @ radius {bal['correction_radius_mm']:.1f} mm")
        print(
            f"4. Correction Mass & Weight        : {bal['correction_mass_g']:.3f} g  ({bal['correction_weight_N']:.5f} N / {bal['correction_weight_gf']:.3f} gf)")
        print("==========================================================")
        print("NOTE: Remove the trial mass before installing the permanent correction mass.")

        plot_four_run_polar(bal, angles_deg=TRIAL_ANGLES_DEG)

    finally:
        dev.stopLoopRead()
        dev.closeDevice()


if __name__ == "__main__":
    main()
