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
    "Mixed view": {
        "command": "STREAM_MIXED",
        "labels": ("MP raw", "DIP angle", "FSR raw"),
        "y_min": None,
        "y_max": None,
    },
    "Flex angle": {
        "command": "STREAM_FLEX",
        "labels": ("Flex deg",),
        "y_min": 0.0,
        "y_max": 180.0,
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
        flex_tab.rowconfigure(2, weight=1)

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

        ttk.Label(flex_tab, text="Angle").grid(row=0, column=2, sticky="w", padx=(12, 8))
        ttk.Combobox(
            flex_tab,
            textvariable=self.flex_angle,
            values=FLEX_ANGLES,
            state="readonly",
            width=12,
        ).grid(row=0, column=3, sticky="w")
        ttk.Button(flex_tab, text="Confirm / Save", command=self.save_flex_angle).grid(
            row=0, column=4, sticky="w", padx=(12, 0)
        )
        ttk.Button(flex_tab, text="Clear Flex Calibration", command=self.clear_flex_calibration).grid(
            row=0, column=5, sticky="w", padx=(8, 0)
        )

        table = ttk.LabelFrame(flex_tab, text="Saved flex points", padding=8)
        table.grid(row=1, column=0, columnspan=6, sticky="ew", pady=(12, 10))
        for col, text in enumerate(("Angle", "Status")):
            ttk.Label(table, text=text).grid(row=0, column=col, sticky="w", padx=(0, 24))
        for row, angle in enumerate(FLEX_ANGLES, start=1):
            ttk.Label(table, text=f"{angle} deg").grid(row=row, column=0, sticky="w", padx=(0, 24), pady=2)
            ttk.Label(table, textvariable=self.flex_status[angle]).grid(row=row, column=1, sticky="w", pady=2)

        log_frame = ttk.LabelFrame(flex_tab, text="Serial log", padding=8)
        log_frame.grid(row=2, column=0, columnspan=6, sticky="nsew")
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

        controls = ttk.Frame(self.plot_tab)
        controls.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        ttk.Label(controls, text="Stream").grid(row=0, column=0, sticky="w")
        stream_combo = ttk.Combobox(
            controls,
            textvariable=self.current_stream,
            values=list(STREAMS.keys()),
            width=26,
            state="readonly",
        )
        stream_combo.grid(row=0, column=1, padx=(8, 10))
        ttk.Button(controls, text="Start Plot", command=self.start_stream).grid(row=0, column=2, padx=(0, 8))
        ttk.Button(controls, text="Stop", command=self.stop_stream).grid(row=0, column=3, padx=(0, 14))
        ttk.Label(controls, textvariable=self.latest_values).grid(row=0, column=4, sticky="w")

        self.plot = PlotCanvas(self.plot_tab)
        self.plot.grid(row=1, column=0, sticky="nsew")
        initial = STREAMS[self.current_stream.get()]
        self.plot.reset(initial["labels"], initial["y_min"], initial["y_max"])

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
        self.after(500, self.restore_saved_flex_calibration)

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

    def start_stream(self):
        config = STREAMS[self.current_stream.get()]
        self.active_stream_config = config
        self.plot.reset(config["labels"], config["y_min"], config["y_max"])
        self.latest_values.set("Waiting for data")
        self.send_command(config["command"])

    def _start_named_stream(self, stream_name):
        self.current_stream.set(stream_name)
        self.start_stream()

    def stop_stream(self):
        self.active_stream_config = None
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
            for sensor_id, points in flex_sensors.items():
                if sensor_id in self.flex_calibration and isinstance(points, dict):
                    self._load_flex_points(sensor_id, points)
        else:
            legacy_points = flex_data.get("angles", {})
            if isinstance(legacy_points, dict):
                self._load_flex_points(DEFAULT_FLEX_SENSOR_ID, legacy_points)

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
        values = parse_numeric_csv(line)
        if not values:
            return

        labels = self.active_stream_config["labels"]
        if len(values) < len(labels):
            return

        y_min = self.active_stream_config["y_min"]
        y_max = self.active_stream_config["y_max"]
        if len(values) >= len(labels) + 2:
            y_min = values[-2]
            y_max = values[-1]
            values = values[: len(labels)]
        else:
            values = values[: len(labels)]

        now = time.monotonic()
        if now - self.last_plot_time >= 0.03:
            self.plot.add_values(values, y_min, y_max)
            self.latest_values.set("  ".join(f"{label}: {value:.2f}" for label, value in zip(labels, values)))
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
