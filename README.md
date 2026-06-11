# DMMS — Digital Motion Measurement System

Sensor glove that reads flex and pressure data from three fingers (Index, Middle, Ring) using an ESP32 and a CD74HC4067 16-channel multiplexer. A Python GUI handles live plotting and angle calibration.

---

## Hardware Overview

| Component | Part |
|-----------|------|
| Microcontroller | ESP32 (esp32dev) |
| Multiplexer | CD74HC4067 (16-channel) |
| Flex sensors | 6× (MP + DIP joint per finger) |
| Pressure sensors | 3× FSR (one per finger) |

---

## Wiring

### ESP32 → CD74HC4067

| ESP32 Pin | Mux Pin | Purpose |
|-----------|---------|---------|
| GPIO 26 | S0 | Mux select bit 0 |
| GPIO 25 | S1 | Mux select bit 1 |
| GPIO 33 | S2 | Mux select bit 2 |
| GPIO 32 | S3 | Mux select bit 3 |
| GPIO 34 | SIG | Analog signal input (ADC) |
| GPIO 2 | — | Onboard LED (flashes during calibration) |

### CD74HC4067 Channel Map

| Channel | Sensor | Description |
|---------|--------|-------------|
| C0 | INDEX_MP | Index finger MP joint flex |
| C1 | INDEX_DIP | Index finger DIP joint flex |
| C2 | INDEX_FSR | Index finger pressure (FSR) |
| C3 | MIDDLE_MP | Middle finger MP joint flex |
| C4 | MIDDLE_DIP | Middle finger DIP joint flex |
| C5 | MIDDLE_FSR | Middle finger pressure (FSR) |
| C6 | RING_MP | Ring finger MP joint flex |
| C7 | RING_DIP | Ring finger DIP joint flex |
| C8 | RING_FSR | Ring finger pressure (FSR) |

> **MP** = Metacarpophalangeal joint (knuckle), **DIP** = Distal Interphalangeal joint (fingertip)

---

## Project Structure

```
DMMS/
├── src/
│   └── Full_System.ino     # ESP32 firmware (PlatformIO)
├── gui/
│   ├── sensor_glove_gui.py # Python GUI (calibration + live plot)
│   └── requirements.txt    # Python dependencies
├── data/
│   └── calibrated_data.json # Saved flex calibration points
└── platformio.ini           # PlatformIO build config
```

---

## Firmware Setup

1. Install [PlatformIO](https://platformio.org/).
2. Open the project folder in VS Code with the PlatformIO extension.
3. Build and upload to the ESP32:
   ```
   pio run --target upload
   ```
4. Serial baud rate: **115200**

---

## GUI Setup

```bash
pip install -r gui/requirements.txt
python gui/sensor_glove_gui.py
```

Requires Python 3.x. Uses only `pyserial` and the standard library (tkinter).

---

## Calibration

Flex sensors must be calibrated at three angles (0°, 45°, 90°) before angle output is meaningful.

1. Connect to the ESP32 via the GUI (select COM port → Connect).
2. Go to the **Calibration** tab.
3. Select the sensor (e.g. *Index Finger DIP*) and angle (e.g. *0*).
4. Hold the finger at that angle, click **Confirm / Save**.
5. Repeat for 45° and 90°.
6. Calibration is saved automatically to `data/calibrated_data.json` and restored on the next connection.

To reload saved calibration without re-capturing, click **Load from JSON**.

---

## Serial Commands

The GUI communicates over serial at 115200 baud. Commands can also be sent manually via any serial monitor.

| Command | Description |
|---------|-------------|
| `STREAM_RAW_INDEX` | Stream Index MP, DIP, FSR raw ADC |
| `STREAM_RAW_MIDDLE` | Stream Middle MP, DIP, FSR raw ADC |
| `STREAM_RAW_RING` | Stream Ring MP, DIP, FSR raw ADC |
| `STREAM_ALL_SENSORS` | Stream all 6 flex sensors (DIP+MP for each finger) |
| `STREAM_FSR_ALL` | Stream FSR raw ADC for all three fingers |
| `STREAM_MULTI_RAW` | Stream all 9 sensors (FSR, DIP, MP × 3 fingers) |
| `STREAM_MIXED` | Stream Middle MP raw, DIP angle (if calibrated), FSR raw |
| `STREAM_FLEX` | Stream Middle DIP angle (requires calibration) |
| `STREAM_FSR` | Stream Middle FSR raw ADC |
| `CAL_FLEX <SENSOR_ID> <angle>` | Capture calibration point (angle = 0, 45, or 90) |
| `SET_FLEX <SENSOR_ID> <angle> <adc>` | Load a saved calibration point |
| `CLEAR_FLEX` | Clear all flex calibration data |
| `STOP` | Stop any active stream |

**Sensor IDs**: `INDEX_DIP`, `INDEX_MP`, `MIDDLE_DIP`, `MIDDLE_MP`, `RING_DIP`, `RING_MP`

---

## Left Hand Support

The GUI has a **Hand** selector (Right / Left). When Left is selected, the Index and Ring channel assignments are swapped in software — no rewiring needed.
