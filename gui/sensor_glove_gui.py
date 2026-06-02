import csv
import json
import queue
import threading
import time
import tkinter as tk
from collections import deque
from pathlib import Path
from tkinter import messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover - shown in the GUI when launched by user
    serial = None
    list_ports = None

import itertools

BAUD_RATE = 115200
MAX_POINTS = 240
FLEX_ANGLES = ("0", "45", "90", "180")
FLEX_SENSORS = (
    ("MIDDLE_DIP", "Middle Finger DIP"),
    ("MIDDLE_MP", "Middle Finger MP"),
    ("INDEX_DIP", "Index Finger DIP"),
    ("INDEX_MP", "Index Finger MP"),
    ("RING_DIP", "Ring Finger DIP"),
    ("RING_MP", "Ring Finger MP"),
)
FLEX_SENSOR_LABELS = tuple(label for _sensor_id, label in FLEX_SENSORS)
FLEX_SENSOR_ID_BY_LABEL = {label: sensor_id for sensor_id, label in FLEX_SENSORS}
FLEX_SENSOR_LABEL_BY_ID = {sensor_id: label for sensor_id, label in FLEX_SENSORS}
DEFAULT_FLEX_SENSOR_ID = FLEX_SENSORS[0][0]
CALIBRATION_DATA_FILE = Path(__file__).resolve().parents[1] / "data" / "calibrated_data.json"


STREAMS = {
    "Raw sensors": {
        "command": "STREAM_RAW",
        "labels": ("MP raw", "DIP raw", "FSR raw"),
        "y_min": 0.0,
        "y_max": 4095.0,
    },
    "FSR raw": {
        "command": "STREAM_FSR",
        "labels": ("FSR raw",),
        "y_min": 0.0,
        "y_max": 4095.0,
    },
}


class SerialWorker:
    def __init__(self, on_line):
        self.on_line = on_line
        self._serial = None
        self._thread = None
        self._running = threading.Event()
        self._write_lock = threading.Lock()

    @property
    def is_connected(self):
        return self._serial is not None and self._serial.is_open

    def connect(self, port):
        if serial is None:
            raise RuntimeError("pyserial is not installed. Run: py -m pip install -r gui/requirements.txt")
        self.disconnect()
        self._serial = serial.Serial(port, BAUD_RATE, timeout=0.1)
        self._running.set()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def disconnect(self):
        self._running.clear()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=0.8)
        self._thread = None
        if self._serial:
            try:
                self._serial.close()
            finally:
                self._serial = None

    def send(self, text):
        if not self.is_connected:
            raise RuntimeError("Serial port is not connected.")
        with self._write_lock:
            self._serial.write(text.encode("utf-8"))
            self._serial.flush()

    def _read_loop(self):
        while self._running.is_set():
            try:
                raw = self._serial.readline()
            except Exception as exc:
                self.on_line(f"[serial error] {exc}")
                self._running.clear()
                break
            if raw:
                self.on_line(raw.decode("utf-8", errors="replace").strip())


class PlotCanvas(tk.Canvas):
    COLORS = ("#2563eb", "#dc2626", "#059669", "#9333ea")

    def __init__(self, master):
        super().__init__(master, bg="#ffffff", highlightthickness=1, highlightbackground="#d7dde8")
        self.series = []
        self.labels = ()
        self.y_min = 0.0
        self.y_max = 4095.0
        self.bind("<Configure>", lambda _event: self.redraw())

    def reset(self, labels, y_min, y_max):
        self.labels = labels
        self.series = [deque(maxlen=MAX_POINTS) for _ in labels]
        self.y_min = y_min if y_min is not None else 0.0
        self.y_max = y_max if y_max is not None else 1.0
        self.redraw()

    def add_values(self, values, y_min=None, y_max=None):
        if not self.series:
            return
        for idx, value in enumerate(values[: len(self.series)]):
            self.series[idx].append(value)
        if y_min is None or y_max is None:
            flat = [item for line in self.series for item in line]
            if flat:
                low = min(flat)
                high = max(flat)
                pad = max((high - low) * 0.15, 1.0)
                self.y_min = low - pad
                self.y_max = high + pad
        else:
            self.y_min = y_min
            self.y_max = y_max
        self.redraw()

    def redraw(self):
        self.delete("all")
        width = max(self.winfo_width(), 10)
        height = max(self.winfo_height(), 10)
        left, top, right, bottom = 58, 18, width - 20, height - 42
        if right <= left or bottom <= top:
            return

        self.create_rectangle(left, top, right, bottom, outline="#d7dde8")
        for step in range(5):
            y = top + (bottom - top) * step / 4
            value = self.y_max - (self.y_max - self.y_min) * step / 4
            self.create_line(left, y, right, y, fill="#eef2f7")
            self.create_text(left - 8, y, text=f"{value:.1f}", anchor="e", fill="#536173", font=("Segoe UI", 8))

        span = self.y_max - self.y_min
        if span == 0:
            span = 1.0

        for idx, line in enumerate(self.series):
            if len(line) < 2:
                continue
            color = self.COLORS[idx % len(self.COLORS)]
            points = []
            values = list(line)
            for point_idx, value in enumerate(values):
                x = left + (right - left) * point_idx / max(MAX_POINTS - 1, 1)
                y = bottom - (bottom - top) * ((value - self.y_min) / span)
                points.extend((x, y))
            self.create_line(*points, fill=color, width=2, smooth=True)

        legend_x = left
        for idx, label in enumerate(self.labels):
            color = self.COLORS[idx % len(self.COLORS)]
            self.create_rectangle(legend_x, height - 28, legend_x + 12, height - 16, fill=color, outline=color)
            self.create_text(legend_x + 18, height - 22, text=label, anchor="w", fill="#1f2937", font=("Segoe UI", 9))
            legend_x += 120


class SensorGloveGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("DMMS GUI")
        self.geometry("1000x680")
        self.minsize(820, 560)

        self.lines = queue.Queue()
        self.worker = SerialWorker(self.lines.put)
        self.current_stream = tk.StringVar(value="Raw sensors")
        self.calibration_sensor = tk.StringVar(value="Flex Sensor (Angle)")
        self.flex_sensor = tk.StringVar(value=FLEX_SENSOR_LABEL_BY_ID[DEFAULT_FLEX_SENSOR_ID])
        self.flex_angle = tk.StringVar(value=FLEX_ANGLES[0])
        self.status = tk.StringVar(value="Disconnected")
        self.latest_values = tk.StringVar(value="No data")
        self.flex_status = {angle: tk.StringVar(value="Not saved") for angle in FLEX_ANGLES}
        self.flex_calibration = {
            sensor_id: {angle: None for angle in FLEX_ANGLES}
            for sensor_id, _label in FLEX_SENSORS
        }
        self.active_stream_config = None
        self.last_plot_time = 0.0

        self.multi_stream_active = False
        self.active_multi_selection = []
        self.active_multi_indices = []
        self.last_multi_plot_time = 0.0

        self._load_calibration_data()
        self._build_ui()
        self.refresh_ports()
        self.after(40, self._process_lines)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self):
        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)

        top = ttk.Frame(self, padding=(12, 10))
        top.grid(row=0, column=0, sticky="ew")
        top.columnconfigure(1, weight=1)

        ttk.Label(top, text="Port").grid(row=0, column=0, sticky="w")
        self.port_combo = ttk.Combobox(top, width=24, state="readonly")
        self.port_combo.grid(row=0, column=1, sticky="w", padx=(8, 10))
        ttk.Button(top, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=(0, 8))
        self.connect_button = ttk.Button(top, text="Connect", command=self.toggle_connection)
        self.connect_button.grid(row=0, column=3, padx=(0, 12))
        ttk.Label(top, textvariable=self.status).grid(row=0, column=4, sticky="w")

        tabs = ttk.Notebook(self)
        tabs.grid(row=1, column=0, sticky="nsew", padx=12, pady=(0, 12))

        self.calibration_tab = ttk.Frame(tabs, padding=12)
        self.plot_tab = ttk.Frame(tabs, padding=12)
        tabs.add(self.calibration_tab, text="Calibration")
        tabs.add(self.plot_tab, text="Plotting")

        self._build_calibration_tab()
        self._build_plot_tab()

    def _build_calibration_tab(self):
        self.calibration_tab.columnconfigure(0, weight=1)
        self.calibration_tab.rowconfigure(1, weight=1)

        selector = ttk.Frame(self.calibration_tab)
        selector.grid(row=0, column=0, sticky="ew", pady=(0, 12))
        ttk.Label(selector, text="Sensor").grid(row=0, column=0, sticky="w", padx=(0, 8))
        sensor_combo = ttk.Combobox(
            selector,
            textvariable=self.calibration_sensor,
            values=("Flex Sensor (Angle)", "FSR Sensor (Force)"),
            state="readonly",
            width=26,
        )
        sensor_combo.grid(row=0, column=1, sticky="w")
        sensor_combo.bind("<<ComboboxSelected>>", lambda _event: self._show_calibration_settings())

        settings = ttk.Frame(self.calibration_tab)
        settings.grid(row=1, column=0, sticky="nsew")
        settings.columnconfigure(0, weight=1)
        settings.rowconfigure(0, weight=1)

        self.flex_settings = ttk.Frame(settings)
        self.fsr_settings = ttk.Frame(settings)
        for frame in (self.flex_settings, self.fsr_settings):
            frame.grid(row=0, column=0, sticky="nsew")

        flex_tab = self.flex_settings
        flex_tab.columnconfigure(1, weight=1)
        flex_tab.rowconfigure(3, weight=1)

        # Row 0 – sensor selector
        ttk.Label(flex_tab, text="Flex position").grid(row=0, column=0, sticky="w", padx=(0, 8))
        flex_sensor_combo = ttk.Combobox(
            flex_tab,
            textvariable=self.flex_sensor,
            values=FLEX_SENSOR_LABELS,
            state="readonly",
            width=24,
        )
        flex_sensor_combo.grid(row=0, column=1, sticky="w")
        flex_sensor_combo.bind("<<ComboboxSelected>>", lambda _event: self._refresh_flex_status())

        # Row 1 – angle selector + action buttons
        action_row = ttk.Frame(flex_tab)
        action_row.grid(row=1, column=0, columnspan=6, sticky="w", pady=(8, 4))

        ttk.Label(action_row, text="Angle").pack(side="left", padx=(0, 8))
        ttk.Combobox(
            action_row,
            textvariable=self.flex_angle,
            values=FLEX_ANGLES,
            state="readonly",
            width=12,
        ).pack(side="left", padx=(0, 20))

        ttk.Button(action_row, text="Confirm / Save", command=self.save_flex_angle).pack(side="left", padx=(0, 8))
        ttk.Button(action_row, text="Clear Flex Calibration", command=self.clear_flex_calibration).pack(side="left", padx=(0, 8))
        ttk.Button(action_row, text="Load from JSON", command=self.load_flex_calibration_from_json).pack(side="left")

        # Row 2 – saved points table
        table = ttk.LabelFrame(flex_tab, text="Saved flex points", padding=8)
        table.grid(row=2, column=0, columnspan=6, sticky="ew", pady=(12, 10))
        for col, text in enumerate(("Angle", "Status")):
            ttk.Label(table, text=text).grid(row=0, column=col, sticky="w", padx=(0, 24))
        for row, angle in enumerate(FLEX_ANGLES, start=1):
            ttk.Label(table, text=f"{angle} deg").grid(row=row, column=0, sticky="w", padx=(0, 24), pady=2)
            ttk.Label(table, textvariable=self.flex_status[angle]).grid(row=row, column=1, sticky="w", pady=2)

        # Row 3 – serial log
        log_frame = ttk.LabelFrame(flex_tab, text="Serial log", padding=8)
        log_frame.grid(row=3, column=0, columnspan=6, sticky="nsew")
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        self.log = tk.Text(log_frame, height=14, wrap="word", state="disabled", font=("Consolas", 10))
        self.log.grid(row=0, column=0, sticky="nsew")
        scroll = ttk.Scrollbar(log_frame, command=self.log.yview)
        scroll.grid(row=0, column=1, sticky="ns")
        self.log.configure(yscrollcommand=scroll.set)

        fsr_tab = self.fsr_settings
        fsr_tab.columnconfigure(0, weight=1)
        fsr_tab.rowconfigure(1, weight=1)
        controls = ttk.Frame(fsr_tab)
        controls.grid(row=0, column=0, sticky="ew")
        ttk.Button(controls, text="Start FSR Force Stream", command=lambda: self._start_named_stream("FSR raw")).grid(
            row=0, column=0, sticky="w", padx=(0, 8)
        )
        ttk.Button(controls, text="Stop", command=self.stop_stream).grid(row=0, column=1, sticky="w")

        fsr_info = ttk.LabelFrame(fsr_tab, text="FSR force settings", padding=8)
        fsr_info.grid(row=1, column=0, sticky="nsew", pady=(12, 0))
        fsr_info.columnconfigure(0, weight=1)
        ttk.Label(
            fsr_info,
            text="FSR force currently uses the raw ADC range from 0 to 4095.",
        ).grid(row=0, column=0, sticky="w")
        ttk.Label(
            fsr_info,
            text="Use the FSR stream to check force readings from the pressure sensor.",
        ).grid(row=1, column=0, sticky="w", pady=(6, 0))

        self._show_calibration_settings()
        self._refresh_flex_status()

    def _build_plot_tab(self):
        self.plot_tab.columnconfigure(0, weight=1)
        self.plot_tab.rowconfigure(1, weight=1)

        # ---- Single‑stream controls ----
        single_frame = ttk.LabelFrame(self.plot_tab, text="Single sensor stream", padding=8)
        single_frame.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        single_controls = ttk.Frame(single_frame)
        single_controls.pack(fill="x")
        ttk.Label(single_controls, text="Stream").pack(side="left")
        stream_combo = ttk.Combobox(
            single_controls,
            textvariable=self.current_stream,
            values=list(STREAMS.keys()),
            width=26,
            state="readonly",
        )
        stream_combo.pack(side="left", padx=(8, 10))
        ttk.Button(single_controls, text="Start Plot", command=self.start_stream).pack(side="left", padx=(0, 8))
        ttk.Button(single_controls, text="Stop", command=self.stop_stream).pack(side="left", padx=(0, 14))
        ttk.Label(single_controls, textvariable=self.latest_values).pack(side="left")

        # Checkbuttons for raw sensor selection (only for Raw sensors stream)
        raw_sel_frame = ttk.Frame(single_frame)
        raw_sel_frame.pack(fill="x", pady=(5, 0))
        self.raw_show_mp = tk.IntVar(value=1)
        self.raw_show_dip = tk.IntVar(value=1)
        self.raw_show_fsr = tk.IntVar(value=1)
        ttk.Checkbutton(raw_sel_frame, text="MP", variable=self.raw_show_mp).pack(side="left", padx=5)
        ttk.Checkbutton(raw_sel_frame, text="DIP", variable=self.raw_show_dip).pack(side="left", padx=5)
        ttk.Checkbutton(raw_sel_frame, text="FSR", variable=self.raw_show_fsr).pack(side="left", padx=5)

        # ---- Single‑stream plot canvas ----
        self.plot = PlotCanvas(self.plot_tab)
        self.plot.grid(row=1, column=0, sticky="nsew")
        initial = STREAMS[self.current_stream.get()]
        self.plot.reset(initial["labels"], initial["y_min"], initial["y_max"])

        multi_frame = ttk.LabelFrame(self.plot_tab, text="Multi‑finger plot", padding=8)
        multi_frame.grid(row=2, column=0, sticky="ew", pady=(10, 0))
        multi_controls = ttk.Frame(multi_frame)
        multi_controls.pack(fill="x")
        ttk.Label(multi_controls, text="Tick joints to plot:").pack(side="left")
        ttk.Button(multi_controls, text="Start Multi Plot", command=self.start_multi_stream).pack(side="left", padx=(8, 10))
        ttk.Button(multi_controls, text="Stop", command=self.stop_stream).pack(side="left")

        self.multi_vars = {}
        fingers_order = ["INDEX", "MIDDLE", "RING"]   # adjust as needed
        cb_frame = ttk.Frame(multi_frame)
        cb_frame.pack(fill="x", pady=(8, 0))
        row_idx = 0
        for finger in fingers_order:
            dip_id = f"{finger}_DIP"
            mp_id = f"{finger}_MP"
            for joint, sensor_id in [("DIP", dip_id), ("MP", mp_id)]:
                if sensor_id not in FLEX_SENSOR_LABEL_BY_ID:
                    continue
                var = tk.IntVar(value=0)
                label = FLEX_SENSOR_LABEL_BY_ID[sensor_id]
                cb = ttk.Checkbutton(cb_frame, text=label, variable=var)
                cb.grid(row=row_idx, column=0, sticky="w", padx=5)
                self.multi_vars[sensor_id] = var
                row_idx += 1

        # ---- Multi‑finger plot canvas ----
        self.multi_plot = PlotCanvas(self.plot_tab)
        self.multi_plot.grid(row=3, column=0, sticky="nsew", pady=(10, 0))
        self.multi_plot.reset([], 0, 180)

        self.plot_tab.rowconfigure(1, weight=1)   # single plot row
        self.plot_tab.rowconfigure(3, weight=1)   # multi plot row

    def start_multi_stream(self):
        selected_ids = [sensor_id for sensor_id, var in self.multi_vars.items() if var.get() == 1]
        if not selected_ids:
            messagebox.showinfo("No selection", "Please tick at least one finger joint.")
            return

        sensor_index_map = {}
        for idx, (sensor_id, _) in enumerate(FLEX_SENSORS):
            sensor_index_map[sensor_id] = idx

        self.active_multi_selection = selected_ids
        self.active_multi_indices = [sensor_index_map[sid] for sid in selected_ids]

        labels = [FLEX_SENSOR_LABEL_BY_ID[sid] for sid in selected_ids]
        self.multi_plot.reset(labels, 0.0, 180.0)
        self.multi_plot.grid()

        self.active_stream_config = None
        self.send_command("STREAM_MULTI_RAW")

        self.multi_stream_active = True

    def _handle_multi_plot_line(self, line):
        values = parse_numeric_csv(line)
        if len(values) < len(FLEX_SENSORS):
            return

        selected_raws = []
        for idx in self.active_multi_indices:
            if idx < len(values):
                selected_raws.append(values[idx])
            else:
                selected_raws.append(0.0)

        angles = []
        for sensor_id, raw in zip(self.active_multi_selection, selected_raws):
            angle = self._compute_angle_from_calibration(sensor_id, raw)
            angles.append(angle)

        now = time.monotonic()
        if now - self.last_multi_plot_time >= 0.03:
            self.multi_plot.add_values(angles, y_min=0.0, y_max=180.0)
            self.latest_values.set("  ".join(
                f"{lbl}: {a:.1f}°" for lbl, a in zip(self.active_multi_selection, angles)
            ))
            self.last_multi_plot_time = now

    def _compute_angle_from_calibration(self, sensor_id, raw_adc):
        """Use the stored calibration points to interpolate an angle (0‑180)."""
        points = self.flex_calibration.get(sensor_id, {})
        calib_list = []
        for angle_str, adc_val in points.items():
            if adc_val is not None:
                calib_list.append((int(angle_str), adc_val))
        if len(calib_list) < 2:
            return 0.0   # not enough data
        calib_list.sort(key=lambda x: x[0])  # sort by angle

        angles = [p[0] for p in calib_list]
        adcs   = [p[1] for p in calib_list]

        increasing = adcs[-1] > adcs[0]
        if increasing:
            if raw_adc <= adcs[0]: return float(angles[0])
            if raw_adc >= adcs[-1]: return float(angles[-1])
            for i in range(len(adcs)-1):
                if adcs[i] <= raw_adc <= adcs[i+1]:
                    t = (raw_adc - adcs[i]) / (adcs[i+1] - adcs[i])
                    return angles[i] + t * (angles[i+1] - angles[i])
        else:
            if raw_adc >= adcs[0]: return float(angles[0])
            if raw_adc <= adcs[-1]: return float(angles[-1])
            for i in range(len(adcs)-1):
                if adcs[i] >= raw_adc >= adcs[i+1]:
                    t = (adcs[i] - raw_adc) / (adcs[i] - adcs[i+1])
                    return angles[i] + t * (angles[i+1] - angles[i])
        return 0.0

    def load_flex_calibration_from_json(self):
        self._load_calibration_data()
        self._refresh_flex_status()
        if self.worker.is_connected:
            self.restore_saved_flex_calibration()
            self.status.set("Calibration loaded from JSON and sent to device")
        else:
            self.status.set("Calibration loaded from JSON (device not connected)")

    def refresh_ports(self):
        if list_ports is None:
            self.port_combo["values"] = ()
            self.status.set("Install pyserial to list ports")
            return
        ports = [port.device for port in list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and not self.port_combo.get():
            self.port_combo.set(ports[0])

    def toggle_connection(self):
        if self.worker.is_connected:
            self.worker.disconnect()
            self.connect_button.configure(text="Connect")
            self.status.set("Disconnected")
            return
        port = self.port_combo.get()
        if not port:
            messagebox.showwarning("No serial port", "Select the ESP32 serial port first.")
            return
        try:
            self.worker.connect(port)
        except Exception as exc:
            messagebox.showerror("Connection failed", str(exc))
            return
        self.connect_button.configure(text="Disconnect")
        self.status.set(f"Connected to {port} at {BAUD_RATE}")
        self.after(1500, self.restore_saved_flex_calibration)

    def send_command(self, command):
        try:
            self.worker.send(f"{command}\n")
        except Exception as exc:
            messagebox.showerror("Serial command failed", str(exc))

    def save_flex_angle(self):
        sensor_id = self._selected_flex_sensor_id()
        self.send_command(f"CAL_FLEX {sensor_id} {self.flex_angle.get()}")

    def clear_flex_calibration(self):
        for value in self.flex_status.values():
            value.set("Not saved")
        self.flex_calibration = {
            sensor_id: {angle: None for angle in FLEX_ANGLES}
            for sensor_id, _label in FLEX_SENSORS
        }
        self._clear_calibration_data_file()
        if self.worker.is_connected:
            self.send_command("CLEAR_FLEX")
        else:
            self.status.set("Saved calibration data cleared")

    def restore_saved_flex_calibration(self):
        if not self.worker.is_connected:
            return
        for sensor_id, points in self.flex_calibration.items():
            for angle, adc in points.items():
                if adc is not None:
                    self.send_command(f"SET_FLEX {sensor_id} {angle} {adc}")
                    time.sleep(0.02)
    
    def start_stream(self):
        config = STREAMS[self.current_stream.get()]
        self.active_stream_config = config
        if self.current_stream.get() == "Raw sensors":
            labels = []
            if self.raw_show_mp.get():
                labels.append("MP raw")
            if self.raw_show_dip.get():
                labels.append("DIP raw")
            if self.raw_show_fsr.get():
                labels.append("FSR raw")
            if not labels:   # at least one must be selected
                labels = ["MP raw"]   # default fallback
            config["labels"] = tuple(labels)
        else:
            # FSR raw stream – use its predefined label
            pass

        self.plot.reset(config["labels"], config["y_min"], config["y_max"])
        self.latest_values.set("Waiting for data")
        self.send_command(config["command"])

    def _start_named_stream(self, stream_name):
        self.current_stream.set(stream_name)
        self.start_stream()

    def stop_stream(self):
        self.active_stream_config = None
        self.multi_stream_active = False
        self.send_command("STOP")

    def _show_calibration_settings(self):
        if self.calibration_sensor.get().startswith("FSR"):
            self.fsr_settings.tkraise()
        else:
            self.flex_settings.tkraise()

    def _selected_flex_sensor_id(self):
        return FLEX_SENSOR_ID_BY_LABEL.get(self.flex_sensor.get(), DEFAULT_FLEX_SENSOR_ID)

    def _refresh_flex_status(self):
        points = self.flex_calibration[self._selected_flex_sensor_id()]
        for angle in FLEX_ANGLES:
            adc = points.get(angle)
            self.flex_status[angle].set(f"ADC {adc} (saved)" if adc is not None else "Not saved")

    def _load_calibration_data(self):
        if not CALIBRATION_DATA_FILE.exists():
            return
        try:
            data = json.loads(CALIBRATION_DATA_FILE.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return

        flex_data = data.get("flex", {})
        flex_sensors = flex_data.get("sensors")
        if isinstance(flex_sensors, dict):
            for sensor_id, sensor_data in flex_sensors.items():
                if sensor_id not in self.flex_calibration:
                    continue
                angles_dict = sensor_data.get("angles", {}) if isinstance(sensor_data, dict) else {}
                self._load_flex_points(sensor_id, angles_dict)
        else:
            legacy_points = flex_data.get("angles", {})
            if isinstance(legacy_points, dict):
                self._load_flex_points(DEFAULT_FLEX_SENSOR_ID, legacy_points)
        print("Loaded calibration:", self.flex_calibration)

    def _load_flex_points(self, sensor_id, points):
        for angle in FLEX_ANGLES:
            adc = points.get(angle)
            if isinstance(adc, int):
                self.flex_calibration[sensor_id][angle] = adc

    def _save_calibration_data(self):
        CALIBRATION_DATA_FILE.parent.mkdir(parents=True, exist_ok=True)
        flex_sensors = {}
        for sensor_id, points in self.flex_calibration.items():
            saved_points = {
                angle: adc
                for angle, adc in points.items()
                if adc is not None
            }
            if saved_points:
                flex_sensors[sensor_id] = {
                    "label": FLEX_SENSOR_LABEL_BY_ID[sensor_id],
                    "angles": saved_points,
                }
        data = {"flex": {"sensors": flex_sensors}} if flex_sensors else {}
        CALIBRATION_DATA_FILE.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")

    def _clear_calibration_data_file(self):
        CALIBRATION_DATA_FILE.parent.mkdir(parents=True, exist_ok=True)
        CALIBRATION_DATA_FILE.write_text("{}\n", encoding="utf-8")

    def _process_lines(self):
        while True:
            try:
                line = self.lines.get_nowait()
            except queue.Empty:
                break
            self._append_log(line)
            self._handle_status_line(line)
            self._handle_plot_line(line)
            if self.multi_stream_active:
                self._handle_multi_plot_line(line)
        self.after(40, self._process_lines)

    def _append_log(self, line):
        self.log.configure(state="normal")
        self.log.insert("end", line + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def _handle_status_line(self, line):
        parts = [part.strip() for part in line.split(",")]
        if len(parts) >= 5 and parts[0] == "FLEX_CAL":
            self._handle_flex_calibration_line(parts[1], parts[2], parts[3], parts[4])
        elif len(parts) >= 6 and parts[0] == "ACK" and parts[1] == "SET_FLEX":
            self._handle_flex_restore_line(parts[2], parts[3], parts[4], parts[5])
        elif line == "ACK,CLEAR_FLEX":
            for value in self.flex_status.values():
                value.set("Not saved")

    def _handle_flex_calibration_line(self, sensor_id, angle, adc, complete):
        if sensor_id not in self.flex_calibration or angle not in FLEX_ANGLES:
            return
        try:
            adc_value = int(adc)
        except ValueError:
            return
        self.flex_calibration[sensor_id][angle] = adc_value
        self._save_calibration_data()
        if sensor_id == self._selected_flex_sensor_id():
            self.flex_status[angle].set(f"ADC {adc_value} ({complete.lower()})")

    def _handle_flex_restore_line(self, sensor_id, angle, adc, complete):
        if sensor_id == self._selected_flex_sensor_id() and angle in FLEX_ANGLES:
            self.flex_status[angle].set(f"ADC {adc} ({complete.lower()})")

    def _handle_plot_line(self, line):
        if not self.active_stream_config:
            return
        config = self.active_stream_config
        values = parse_numeric_csv(line)
        if not values:
            return

        if self.current_stream.get() == "Raw sensors":
            # Values come as [MP, DIP, FSR] from the firmware.
            # We need to pick the ones the user wants to see.
            # Build a mask from the checkbuttons (same order)
            mask = []
            if self.raw_show_mp.get():
                mask.append(0)   # MP is index 0
            if self.raw_show_dip.get():
                mask.append(1)   # DIP is index 1
            if self.raw_show_fsr.get():
                mask.append(2)   # FSR is index 2
            if len(values) < 3:
                return
            selected = [values[i] for i in mask if i < len(values)]
            labels = config["labels"]
            if len(selected) != len(labels):
                return
            values = selected
        else:
            if len(values) < len(config["labels"]):
                return
            values = values[: len(config["labels"])]

        now = time.monotonic()
        if now - self.last_plot_time >= 0.03:
            self.plot.add_values(values, config["y_min"], config["y_max"])
            self.latest_values.set("  ".join(
                f"{label}: {value:.2f}" for label, value in zip(config["labels"], values)
            ))
            self.last_plot_time = now

    def _on_close(self):
        self.worker.disconnect()
        self.destroy()


def parse_numeric_csv(line):
    try:
        row = next(csv.reader([line], skipinitialspace=True))
    except csv.Error:
        return []
    values = []
    for item in row:
        try:
            values.append(float(item))
        except ValueError:
            return []
    return values


if __name__ == "__main__":
    app = SensorGloveGUI()
    app.mainloop()
