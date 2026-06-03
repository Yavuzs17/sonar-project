# Hexagonal Phased-Array Sonar System

A real-time 3-D ultrasonic sonar system built on a **Raspberry Pi 4B**.  
Seven-element hexagonal transducer array with electronic beam-steering, DBSCAN-based obstacle detection, and a PyQt6 live visualisation GUI.

---

## Table of Contents

1. [System Overview](#system-overview)  
2. [Repository Structure](#repository-structure)  
3. [Hardware Requirements](#hardware-requirements)  
4. [Firmware — Build & Run](#firmware--build--run)  
5. [Python UI — Setup & Run](#python-ui--setup--run)  
6. [Calibration Files](#calibration-files)  
7. [CLI Reference](#cli-reference)  
8. [Communication Protocol](#communication-protocol)  

---

## System Overview

```
┌─────────────────────────────────────────────────────┐
│                  Raspberry Pi 4B                    │
│                                                     │
│  ┌──────────────┐   Unix socket   ┌──────────────┐  │
│  │  C Firmware  │◄───────────────►│  Python GUI  │  │
│  │  (./sonar)   │  JSON events /  │  (PyQt6)     │  │
│  └──────┬───────┘  commands       └──────────────┘  │
│         │ I²C / GPIO                                │
└─────────┼───────────────────────────────────────────┘
          │
    ┌─────┴──────────────────────────────────────┐
    │  Hardware peripherals (I²C bus 1 & 3)      │
    │  PLL (CD4046 + MCP3221)  Phase-shift (MCP4651 + PCF8574) │
    │  Preamp (MCP4651)        TVG (MCP4651 + INA821)          │
    │  ADC (ADS1115)           Temp (LM60)                     │
    └────────────────────────────────────────────┘
```

| Layer | Technology |
|---|---|
| Beam-steering | All-pass filter (APF) phase-shift network — 7 channels |
| Frequency synthesis | Software PLL, 38–42 kHz, 100 Hz step |
| Time-varying gain (TVG) | INA821 + MCP4651 digital potentiometer |
| Obstacle detection | DBSCAN (scikit-learn) on Cartesian echo cloud |
| Visualisation | PyQtGraph (2-D polar + 3-D OpenGL) |

---

## Repository Structure

```
sonar-project/
├── main.c                  # Entry point, CLI, socket command handler
├── pll/                    # Phase-locked loop + VCO feedback
├── preamp/                 # Programmable preamplifier (MCP4651)
├── tvg/                    # Time-varying gain (INA821 ramp)
├── phase_shift/            # APF beam-steering + DBSCAN calibration fit
├── beamforming/            # Phase-delay LUT + scan-range management
├── adc/                    # ADS1115 driver (envelope, INA, Doppler, temp)
├── sensor/                 # Echo capture, Doppler, temperature, LUT
├── comms/                  # Unix domain socket server (JSON protocol)
├── tests/                  # Unit tests (gcc, no hardware needed)
├── calib/                  # Calibration LUT CSV files
│   ├── preamp_lut.csv
│   ├── phase_shift_lut.csv
│   ├── tvg_lut.csv
│   ├── doppler_lut.csv
│   └── pll_lut.csv
└── python/
    ├── main.py             # GUI entry point
    ├── socket_client.py    # Async JSON socket client
    ├── requirements.txt    # Python dependencies
    ├── processing/
    │   └── obstacle_detector.py   # DBSCAN worker thread
    └── ui/
        ├── main_window.py  # Main window, layout, event handling
        ├── sonar_map.py    # 2-D polar map (PyQtGraph)
        ├── map_3d.py       # 3-D point cloud (PyQtGraph OpenGL)
        └── toggle_switch.py
```

---

## Hardware Requirements

| Component | Part |
|---|---|
| SBC | Raspberry Pi 4B (2 GB+ RAM) |
| OS | Raspberry Pi OS Bookworm (64-bit) |
| Transducer array | 7-element hexagonal, 40 kHz |
| PLL VCO | CD4046 + passive RC network |
| Frequency divider | CD4060 + CD4017 → 100 Hz reference |
| ADC (PLL feedback) | MCP3221 (I²C bus 3, GPIO 4/5) |
| Phase shifter | MCP4651 Wiper0 + PCF8574 inverter (I²C bus 1) |
| Preamplifier | MCP4651 (I²C bus 1) |
| TVG | MCP4651 Wiper1 + INA821 (I²C bus 1) |
| ADC (signal chain) | ADS1115 (I²C bus 1) |
| Temperature | LM60 (via ADS1115 channel) |

### I²C Bus Assignment

| Bus | GPIO | Devices |
|---|---|---|
| Hardware I²C-1 | SDA=2, SCL=3 | MCP4651 (phase), MCP4651 (preamp/TVG), ADS1115 |
| Software I²C-3 | SDA=4, SCL=5 | MCP3221 (PLL ADC) |

Enable software I²C in `/boot/firmware/config.txt`:
```
dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=4,i2c_gpio_scl=5
```

---

## Firmware — Build & Run

### Dependencies

```bash
# lgpio (GPIO/I²C library)
sudo apt install liblgpio-dev

# cJSON (JSON parsing)
sudo apt install libcjson-dev
```

### Build

```bash
gcc -o sonar main.c \
    pll/pll.c preamp/preamp.c tvg/tvg.c \
    phase_shift/phase_shift.c beamforming/beamforming.c \
    adc/adc.c sensor/sensor.c \
    comms/socket_server.c \
    -llgpio -lpthread -lm -lrt -lcjson -Wall -Wextra
```

### Run

```bash
# Full hardware mode (requires I²C devices)
sudo ./sonar

# Simulation mode — no hardware required, generates fake echoes
./sonar --no-hardware

# Skip calibration file loading
./sonar --no-calib

# Custom calibration directory
./sonar --calib-dir /path/to/calib

# All options
./sonar --help
```

> **Note:** `sudo` is required for GPIO access in hardware mode.  
> In `--no-hardware` mode the firmware runs fully on any Linux machine.

### Unit Tests

```bash
gcc -o test_pll        tests/test_pll.c        pll/pll.c        -llgpio -lpthread -lrt
gcc -o test_preamp     tests/test_preamp.c     preamp/preamp.c  -llgpio -lm
gcc -o test_beamforming tests/test_beamforming.c beamforming/beamforming.c -lm
./test_pll && ./test_preamp && ./test_beamforming
```

---

## Python UI — Setup & Run

### System Requirements

- Python 3.11+
- Raspberry Pi OS Bookworm (Wayland) **or** any Linux/Windows desktop

### Install Dependencies

```bash
cd python
python -m venv .venv
source .venv/bin/activate        # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

`requirements.txt`:
```
PyQt6
pyqtgraph>=0.13
PyOpenGL>=3.1
numpy
scikit-learn>=1.3
```

### Run

```bash
# Start firmware first (separate terminal)
./sonar --no-hardware   # or sudo ./sonar for real hardware

# Then launch GUI
cd python
source .venv/bin/activate
python main.py
```

### Raspberry Pi — Wayland / OpenGL Fix

The GUI auto-sets the following environment variables on startup
(see `python/main.py`). If you encounter display or OpenGL errors,
set them manually before running:

```bash
export XDG_RUNTIME_DIR=/run/user/1000
export DISPLAY=:0
export QT_QPA_PLATFORM=xcb        # force X11 backend
export PYOPENGL_PLATFORM=glx      # desktop OpenGL via GLX
python main.py
```

If the 3-D map tab still fails to render, try software OpenGL:
```bash
export QT_OPENGL=software
python main.py
```

### GUI Overview

| Area | Function |
|---|---|
| Top toolbar | Connect/disconnect, live temperature, sound speed, PLL frequency |
| Left — Scan Range | Set azimuth ±°, elevation ±°, step °; send to firmware |
| Left — Obstacle Detection | Toggle DBSCAN, set ε (m) and min-points |
| Left — Map | Clear all echoes and clusters |
| Centre — 2D Map tab | Real-time azimuth and elevation polar plots |
| Centre — 3D Map tab | OpenGL point cloud with cluster markers |
| Right | Detected cluster list (Cartesian + polar coordinates) |
| Bottom — Steering | QDial azimuth / elevation manual steering |
| Bottom — Frequency | Manual PLL frequency + frequency sweep (min/max/step/dwell) |
| Bottom — Gain | Per-channel and global preamplifier gain |
| Bottom — Action | Burst start/stop, scan start/stop, dwell-time setting |

---

## Calibration Files

Located in `calib/`. Loaded automatically at startup (override with `--calib-dir`).

| File | Format | Description |
|---|---|---|
| `preamp_lut.csv` | `kanal,kazanc_db,step` | Preamplifier gain linearisation — 7 channels |
| `phase_shift_lut.csv` | `alici,step,inverter,olculen_aci` | APF phase measurement points for `ps_kalib_fit()` |
| `tvg_lut.csv` | `t_ms,N` | TVG wiper ramp — 31 points, 0–30 ms |
| `doppler_lut.csv` | `vctrl frekans_hz` | VCO voltage→frequency map (space-separated, no header) |
| `pll_lut.csv` | `vctrl frekans_hz` | PLL feedback VCO map (same format) |

> The supplied values are **theoretical defaults** computed from component datasheets.  
> For best accuracy, replace `phase_shift_lut.csv` with measurements taken  
> from the physical APF circuit using an oscilloscope or phase meter.

---

## CLI Reference

```
./sonar [options]

Options:
  --no-hardware        Simulation mode (no I²C/GPIO required)
  --calib-dir <path>   Calibration directory (default: ./calib)
  --no-calib           Skip calibration file loading
  --log-level <0-3>    0=debug  1=info  2=warn  3=error
  --help               Show this message

Interactive commands (stdin or socket JSON):
  status               System state snapshot
  steer <az> <el>      Beam direction (degrees, ±30)
  freq <hz>            PLL target frequency (38000–42000)
  gain <ch> <db>       Channel gain (ch 0–6)
  gain_all <db>        All channels same gain
  burst_start          Begin burst/listen cycle
  burst_stop           Stop burst
  scan_start [dwell]   Begin raster scan (dwell ms per position)
  scan_stop            Stop scan
  scan_range <az> <el> <step>   Set scan grid
  freq_sweep_start [min] [max] [step] [dwell]   PLL frequency sweep
  freq_sweep_stop      Stop frequency sweep
  read_temp            Temperature (°C) and sound speed (m/s)
  read_echo            Last captured echo
  read_doppler         Doppler frequency and target velocity
```

---

## Communication Protocol

The firmware exposes a **Unix domain socket** at `/tmp/sonar.sock`.  
All messages are newline-terminated JSON.

**Command (client → server):**
```json
{"cmd": "steer", "az": 15.0, "el": 10.0}
```

**Response (server → client):**
```json
{"type": "response", "cmd": "steer", "status": "ok", "az": 15.0, "el": 10.0}
```

**Async events (server → all clients):**
```json
{"type": "event", "name": "echo",          "az": 10.0, "el": 5.0, "dist": 2.34}
{"type": "event", "name": "scan_progress", "step": 42, "total": 169, "az": 10.0, "el": 5.0}
{"type": "event", "name": "sweep_complete","direction": "forward", "count": 3}
{"type": "event", "name": "temperature",   "celsius": 22.5, "sound_speed": 344.8}
{"type": "event", "name": "doppler",       "freq": 40012.0, "velocity": 0.15}
```

---

*Dokuz Eylül University — Electrical & Electronics Engineering*  
*Senior Design Project — 2024–2025*
