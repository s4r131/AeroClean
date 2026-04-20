# AeroClean

Autonomous MAVLink drone that cleans dirty dry-erase boards. A **Raspberry Pi 5** acts as the companion computer — running a **YOLO11n object detector** and a **Tesseract OCR pipeline** to detect board state, then commanding an **ArduPilot** flight controller over UART to take off, scan the room, approach the board, and trigger a pump to clean it.

---

## Quick start

Complete setup first (see [Setup](#setup)), then verify each component in order before flying.

Once all hardware tests pass, start the mission:

```bash
python main.py --mode mission
```

The drone arms, takes off to 1.5 m, yaw-spins to scan for a dirty board, approaches it using camera centering and the range sensor, activates the pump to clean, then returns home and lands. All parameters are in `config.json`.

---

## Project structure

```
AeroClean/
├── main.py               # Entry point — --mode mission | --model ocr | yolo
├── camera.py             # picamera2 capture wrapper (IMX708)
├── ocr_model.py          # Model 1: Tesseract OCR pipeline
├── yolo_model.py         # Model 2: YOLO11n NCNN inference
├── mission.py            # Autonomous mission state machine (IDLE→SCAN→APPROACH→CLEAN→RETURN)
├── mavlink_controller.py # DroneKit wrapper — arm, takeoff, velocity commands, RTL
├── sensors.py            # MTF-02P optical flow reader + VL53L3CX I2C range (sensor A) + TF-Luna/TFMini UART range (sensor B)
├── pump.py               # GPIO pump controller
├── wiper.py              # Wiper arm controller (actuator TBD)
├── sensor_tf_test.py     # TF-Luna / TFMini UART range sensor standalone test
├── camera_test.py        # Pi camera live view + FPS test
├── sensor_range_test.py  # VL53L3CX standalone distance test
├── sensor_flow_test.py   # MTF-02P optical flow standalone test
├── sensor_ocr_test.py    # Range sensor + OCR integration test
├── collect_data.py       # Capture training images from the Pi camera
├── config.json           # All tunable parameters (camera, YOLO, mission)
├── requirements.txt      # Python dependencies
├── train/
│   ├── train_colab.ipynb # Google Colab training notebook
│   └── dataset.yaml      # YOLO dataset config (class names + split paths)
├── assets/               # Diagram images for this README
├── weights/              # Trained model files        [not tracked in git]
├── images/               # Training images            [not tracked in git]
├── labels/               # YOLO annotation files      [not tracked in git]
└── output/               # Saved inference frames     [not tracked in git]
```

---

## Hardware requirements

| Component | Specification |
|---|---|
| **Board** | Raspberry Pi 5 (4 GB or 8 GB RAM) |
| **Camera** | Arducam / Raspberry Pi Camera Module 3 — IMX708, 12MP, 75° diagonal |
| **Camera connection** | CSI ribbon cable (included with Camera Module 3) |
| **Flight controller** | ArduPilot-compatible FC (e.g. Pixhawk) — connected to Pi via UART (confirm path with `ls -l /dev/ttyAMA*`) |
| **Flow sensor** | MicoAir MTF-02P optical flow sensor — connected to Pi via UART (second port) |
| **Range sensor** | **Sensor A (default):** VL53L3CX ToF — I2C, GPIO 2/3 (pins 3/5), 3 m range  **or  Sensor B:** TF-Luna / TFMini — UART (set `range_sensor.type = "b"` in config.json) |
| **Pump** | Relay-driven pump on BCM GPIO pin (configurable in `config.json`) |
| **Wiper** | Wiper arm on BCM GPIO pin — actuator type TBD (configurable in `config.json`) |
| **OS** | Raspberry Pi OS Bookworm (64-bit) — December 2023 or later |
| **Storage** | 32 GB SD card minimum (64 GB recommended for training images) |

---

## System architecture

How all the hardware pieces connect and what each one does.

```
┌─────────────────────────────────────────┐
│           Raspberry Pi 5                │
│                                         │
│  IMX708 Camera     →  YOLO detection    │
│  MTF-02P (UART)    →  optical flow      │
│  VL53L3CX (I2C)    →  forward range     │
│  Mission state machine                  │
│  GPIO pump pin     →  pump relay        │
│  GPIO wiper pin    →  wiper arm         │
│                                         │
│  DroneKit (MAVLink over UART)           │
└──────────────┬──────────────────────────┘
               │ UART — MAVLink
┌──────────────▼──────────────────────────┐
│         ArduPilot FC                    │
│  Attitude stabilisation (own IMU)       │
│  Motor control                          │
│  GUIDED mode — accepts velocity targets │
└─────────────────────────────────────────┘
```

ArduPilot handles all low-level stabilisation. The Pi sends body-frame velocity setpoints (`SET_POSITION_TARGET_LOCAL_NED`) and the FC executes them while keeping the drone stable.

### Wiring

| Pi connection | Device | Purpose |
|---|---|---|
| UART (your `/dev/ttyAMAx`) | ArduPilot FC TELEM port | MAVLink command channel (DroneKit) |
| UART (your `/dev/ttyAMAy`) | MTF-02P UART | Optical flow (pymavlink) |
| I2C GPIO 2/3 (pins 3/5) | VL53L3CX (sensor A, default) | Forward range for approach controller |
| UART (your `/dev/ttyAMAz`) | TF-Luna / TFMini (sensor B, alternative) | Forward range — UART alternative to sensor A |
| GPIO BCM pin (configurable) | Pump relay IN | Cleaning mechanism trigger |
| GPIO BCM pin (configurable) | Wiper arm control wire | Wiper arm actuation |

---

## Hardware checklist

Gather everything before starting setup.

- [ ] Raspberry Pi 5 (4 GB or 8 GB)
- [ ] Camera Module 3 (IMX708) on CSI ribbon
- [ ] ArduPilot flight controller — TELEM/UART port wired to Pi UART
- [ ] MicoAir MTF-02P — UART wired to a second Pi UART
- [ ] Range sensor — **sensor A (default):** VL53L3CX (I2C, wire to GPIO 2/3, pins 3/5)  **or  sensor B:** TF-Luna / TFMini (UART, wire to a free Pi UART)
- [ ] Pump + relay — relay IN wired to a free BCM GPIO pin
- [ ] Wiper arm — control wire wired to a free BCM GPIO pin
- [ ] 32 GB+ SD card with Raspberry Pi OS Bookworm 64-bit

---

## Setup

### 1. Flash and configure the Pi

Download **Raspberry Pi OS Bookworm (64-bit)** from the official site and flash it with Raspberry Pi Imager. Enable SSH and set your hostname if needed.

### 2. Install dependencies, clone the repo, and set up the Python environment

Complete all four sub-steps in order. The Python packages must go inside the virtual environment — do not run `pip install` before the venv is active.

#### 2a — Update and install system packages

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y python3-picamera2 tesseract-ocr libcap-dev i2c-tools
```

Verify the camera is detected before continuing:
```bash
libcamera-hello --list-cameras
# Expected: "Available cameras" showing IMX708
```

#### 2b — Clone the repository

```bash
git clone https://github.com/<your-username>/AeroClean.git
cd AeroClean
```

#### 2c — Create and activate the virtual environment

```bash
python -m venv aeroclean_env
source aeroclean_env/bin/activate
```

Your prompt will show `(aeroclean_env)` when the environment is active. **All pip installs, test scripts, and mission runs must be done inside this environment.**

To reactivate in a new terminal session:
```bash
source aeroclean_env/bin/activate
```

#### 2d — Install Python dependencies

With the environment active:

```bash
pip install -r requirements.txt
```

`dronekit`, `pymavlink`, and the Adafruit sensor libraries are already in `requirements.txt` — do not install them again separately.

### 3. Enable hardware interfaces (UART and I2C)

All hardware interfaces on Pi 5 are activated by editing `/boot/firmware/config.txt` and rebooting. Use `pinctrl -p` after each reboot to verify that the change took effect.

---

#### Reading `pinctrl -p`

`pinctrl -p` shows the live state of all 40 GPIO header pins. Run it any time to check whether an interface is active. Each line follows this format:

```
<physical pin>: <function>  <pull> | <level>  // <GPIO name> = <role>
```

| Column | Values | Meaning |
|---|---|---|
| `function` | `no` | Pin not configured — no peripheral attached |
| | `a2` / `a3` / `a4` | Alternate function active (hardware peripheral owns this pin) |
| | `ip` | Software input |
| `pull` | `pu` | Pull-up resistor enabled |
| | `pd` | Pull-down resistor enabled |
| | `pn` | No pull resistor |
| `level` | `--` | Indeterminate — pin not driven (expected on unconfigured pins) |
| | `hi` | Pin driven high |
| | `lo` | Pin driven low |
| `role` | `none` | No function assigned — interface is **not** active |
| | `TXD0`, `SDA1`, etc. | Peripheral function name — interface **is** active |

**Before any interfaces are enabled**, every GPIO pin shows `no` and `= none`:

```
 1: 3v3
 2: 5v
 3: no pu | -- // GPIO2 = none
 4: 5v
 5: no pu | -- // GPIO3 = none
 6: gnd
 7: no pu | -- // GPIO4 = none
 8: no pd | -- // GPIO14 = none
 9: gnd
10: no pd | -- // GPIO15 = none
11: no pd | -- // GPIO17 = none
...
21: no pd | -- // GPIO9  = none
...
24: no pu | -- // GPIO8  = none
...
```

The `no ... = none` pattern means the pin is sitting idle with no peripheral attached. After each step below you will see specific pins switch from `no ... = none` to an alternate function with a real role name. That transition is your confirmation the interface is active.

> **Note:** `pinctrl -p` uses physical pin numbers (1–40 on the header), not BCM GPIO numbers. GPIO14 lives on physical pin 8; GPIO2 lives on physical pin 3. The `// GPIOx` comment tells you the BCM number.

---

#### Sub-step 0 — Disable the serial login shell

> **Do this before enabling UART0.** The Pi OS attaches a login console to the primary UART by default. If it is still running when ArduPilot connects, the OS and the flight controller will fight over the same wire and MAVLink will never connect.

**a) Remove the serial console from the kernel command line:**

```bash
sudo nano /boot/firmware/cmdline.txt
```

The file contains a single long line. Find and delete the token `console=serial0,115200` from that line. Leave everything else exactly as-is. Save and close (`Ctrl+O`, `Enter`, `Ctrl+X`).

> **Do NOT use `raspi-config` for this step on Pi 5.** On Bookworm, `raspi-config` edits the wrong file and the change has no effect. Edit `/boot/firmware/cmdline.txt` directly.

**b) Disable the serial getty service:**

```bash
sudo systemctl disable serial-getty@ttyAMA0.service
sudo systemctl stop serial-getty@ttyAMA0.service
```

Reboot:
```bash
sudo reboot
```

After rebooting, verify nothing is holding the UART:
```bash
# Should return nothing — no getty process on the serial port
ls -l /proc/tty/driver/ | grep serial
```

---

#### Sub-step 1 — Enable UART0 (ArduPilot FC)

Add to `/boot/firmware/config.txt`:
```
enable_uart=1
```

```bash
sudo nano /boot/firmware/config.txt
# Add the line above, save, then reboot:
sudo reboot
```

Run `pinctrl -p` and look for physical pins 8 and 10:

```
Before:
 8: no pd | -- // GPIO14 = none
10: no pd | -- // GPIO15 = none

After (expected):
 8: a4 pn | hi // GPIO14 = TXD0
 9: gnd
10: a4 pu | hi // GPIO15 = RXD0
```

`GPIO14 = TXD0` and `GPIO15 = RXD0` confirm UART0 is live. If you still see `= none`, the line was not saved correctly — open `config.txt` again and confirm `enable_uart=1` is present and not commented out.

---

#### Sub-step 2 — Enable the range sensor interface

Do **only one** of 2A or 2B depending on your hardware. Check `config.json → range_sensor.type` if you are unsure which sensor you have set up.

---

##### Sub-step 2A — Enable I2C (sensor A: VL53L3CX)

Skip this if you are using a TF-Luna / TFMini (go to 2B instead).

Add to `/boot/firmware/config.txt`:
```
dtparam=i2c_arm=on
```

```bash
sudo reboot
```

Run `pinctrl -p` and look for physical pins 3 and 5:

```
Before:
 3: no pu | -- // GPIO2 = none
 5: no pu | -- // GPIO3 = none

After (expected):
 3: a3 pu | hi // GPIO2 = SDA1
 5: a3 pu | hi // GPIO3 = SCL1
```

Then confirm the sensor is visible on the I2C bus:
```bash
sudo i2cdetect -y 1
```
Expected: `29` appears at address `0x29` in the grid. If the grid is all dashes, check the VIN/GND/SDA/SCL wiring.

---

##### Sub-step 2B — Enable additional UART (sensor B: TF-Luna / TFMini)

Skip this if you are using a VL53L3CX (go to 2A instead).

> **Note for sensor B builds:** you will need three UARTs total — UART0 (ArduPilot FC, sub-step 1), one for the MTF-02P (sub-step 3), and this one for the TF sensor. Plan your `dtoverlay=uartX` entries accordingly before rebooting.

Add to `/boot/firmware/config.txt`:
```
dtoverlay=uart4
```

> Use any free overlay UART (`uart1`–`uart5`) that is not already taken by the MTF-02P. The example here uses `uart4` — verify the actual pins with `pinctrl -p` after rebooting and cross-reference with `ls -l /dev/ttyAMA*`.

```bash
sudo reboot
```

Run `pinctrl -p` and confirm the new UART pins changed from `none` to an alternate function — the exact pins depend on which overlay you chose. Then find the device path:
```bash
ls -l /dev/ttyAMA*
```
Note this path — it goes into `config.json → tf_sensor.uart`.

---

#### Sub-step 3 — Enable additional UART (MTF-02P optical flow sensor)

> The ArduPilot FC (sub-step 1) and the MTF-02P both use UART — each needs its own port and its own `dtoverlay=uartX` line in `config.txt`. Sensor B builds need a third port for the TF sensor (sub-step 2B).

Add to `/boot/firmware/config.txt`:
```
dtoverlay=uart3
```

> Additional UARTs (`uart1`–`uart5`) use `dtoverlay=uartX`. Pin assignments vary by UART number — always verify with `pinctrl -p` after rebooting.

```bash
sudo reboot
```

Run `pinctrl -p` and look for physical pins 21 and 24 (UART3):

```
Before:
21: no pd | -- // GPIO9  = none
24: no pu | -- // GPIO8  = none

After (expected):
21: a2 pu | hi // GPIO9  = RXD3
24: a2 pn | hi // GPIO8  = TXD3
```

`GPIO8 = TXD3` and `GPIO9 = RXD3` confirm UART3 is active.

---

#### Sub-step 4 — Add your user to the dialout group

Without this, any attempt to read `/dev/ttyAMA*` or open a DroneKit connection will fail with `Permission denied`.

```bash
sudo usermod -aG dialout $USER
```

Log out and back in (or reboot) for the group change to take effect. Verify:
```bash
groups
# Expected output includes: dialout
```

---

#### Verified `/boot/firmware/config.txt`

After all sub-steps, your `config.txt` should contain at minimum:

**Sensor A (VL53L3CX — I2C):**
```
enable_uart=1
dtparam=i2c_arm=on
dtoverlay=uart3
```

**Sensor B (TF-Luna / TFMini — UART):**
```
enable_uart=1
dtoverlay=uart3
dtoverlay=uart4
```

Add more `dtoverlay=uartX` lines if you have additional serial peripherals.

---

#### Find your device paths, then update config.json

The kernel assigns `ttyAMAx` numbers dynamically — don't guess, confirm first:

```bash
ls -l /dev/serial*
# e.g. /dev/serial0 -> ttyAMA0

ls -l /dev/ttyAMA*
# Lists all active UART devices — cross-reference with pinctrl -p pin numbers to know which is which
```

Once you've confirmed which `ttyAMAx` corresponds to each UART, update `config.json`:
```json
"mavlink_uart": "/dev/ttyAMAx",   ← replace x with the number for the ArduPilot FC UART
"sensor_uart":  "/dev/ttyAMAy"    ← replace y with the number for the MTF-02P UART
```

> `ttyAMAx` numbering can differ between Pi OS versions. Always verify with `ls -l /dev/ttyAMA*` before running the mission.

---

### 4. Map config.json and verify each hardware component

**Do this before attempting a mission.** Every interface you enabled in Step 3 must be mapped in `config.json` and confirmed working with its test script.

#### 4a — Fill in config.json

Open `config.json` and set the values you discovered in Step 3:

```bash
# Find your UART device paths
ls -l /dev/ttyAMA*

# Confirm I2C sensor is visible
sudo i2cdetect -y 1   # expect 0x29
```

Then update `config.json`:
```json
"mission": {
  "mavlink_uart": "/dev/ttyAMAx",   ← UART to ArduPilot FC
  "sensor_uart":  "/dev/ttyAMAy"    ← UART to MTF-02P
}
```

Set your range sensor type — default is `"a"`:
```json
"range_sensor": {
  "type": "a"    ← "a" = VL53L3CX (I2C),  "b" = TF-Luna / TFMini (UART)
}
```
If using sensor B, also set `tf_sensor.uart` to your UART path. `range_sensor.i2c_address` only needs to change if your VL53L3CX was remapped from the default `0x29`.

If running without a monitor:
```json
"display": false
```

#### 4b — Step 1: Camera

Confirm the camera is detected and delivering frames before anything else.

```bash
python camera_test.py
```
Expected: live window opens, terminal prints resolution and FPS every second.
```
[CAM TEST] 1920x1080  28.3 FPS
```

---

#### 4c — Step 2: OCR and YOLO inference

Confirm the vision models run on camera frames. No sensors, no drone. Run one or both:

```bash
# OCR — look for the word "dirty" on the board
python main.py --model ocr

# OR — YOLO — classify the board as clean or dirty
python main.py --model yolo
```
Expected: live window with bounding boxes or OCR overlays. Press `q` to quit.

---

#### 4d — Step 3: Range sensor

Run the test for whichever sensor you have wired.

**Option A — VL53L3CX (I2C):**
```bash
python sensor_range_test.py
```
Expected:
```
[RANGE TEST] 0.452 m  (45.2 cm)
```

**Option B — TF-Luna / TFMini (UART):**
```bash
python sensor_tf_test.py
```
Expected:
```
[TF TEST] 0.452 m  (45.2 cm)  | strength=412  | temp=32.1 C
```

Requires `tf_sensor.uart` to be set in `config.json` first — the script will error clearly if it is not.

---

#### 4e — Step 4: Optical flow sensor

Confirm the UART is active and the MTF-02P is sending data. Use the path you found in Step 3 of hardware setup.

```bash
python sensor_flow_test.py
```
Expected: flow and distance values printing every second.
```
[FLOW TEST] flow_x=0.012  flow_y=-0.003  quality=210
```

---

#### 4f — Step 5: Range sensor + OCR together

Confirm the full detection pipeline — this is what the mission APPROACH state does.

**Option A — VL53L3CX (I2C, default):**
```bash
python sensor_ocr_test.py
```

**Option B — TF-Luna / TFMini (UART):**
```bash
python sensor_ocr_test.py --sensor b --uart /dev/ttyAMAx
```

Expected for both: `CLEAN` banner when nothing is detected. When a board marked "dirty" is in view:
```
[TEST] DIRTY detected — range=0.842m
```
Window shows bounding box + distance overlay.

---

#### 4g — Step 6: Range sensor + OCR + Wiper

> **Not yet implemented** — wiper actuator type is TBD. This step is a placeholder for when the wiper mechanism is confirmed and `wiper.py` is fully implemented.

Once the wiper is wired and implemented, this test will confirm the full cleaning cycle end-to-end: OCR detects dirty board → range sensor confirms distance → wiper engages.

**Do not run `--mode mission` until all steps pass.**

---

## Pre-mission checklist

Use this after completing all setup steps to confirm nothing was missed before arming.

### Software

- [ ] System packages installed: `sudo apt install python3-picamera2 tesseract-ocr libcap-dev i2c-tools`
- [ ] Virtual environment created and active (`source aeroclean_env/bin/activate`)
- [ ] Python packages installed inside the venv: `pip install -r requirements.txt`
- [ ] Serial login shell disabled — `console=serial0,115200` removed from `/boot/firmware/cmdline.txt` and `serial-getty@ttyAMA0.service` disabled
- [ ] `dtparam=i2c_arm=on` in `/boot/firmware/config.txt` **(sensor A only)**
- [ ] `enable_uart=1` and `dtoverlay=uartX` in `/boot/firmware/config.txt`
- [ ] User in `dialout` group (`groups` output includes `dialout`)
- [ ] YOLO weights trained and placed in `weights/best_ncnn_model/`

### config.json

Set these `null` values before running mission mode. Run `ls -l /dev/ttyAMA*` to find the actual device paths — do not copy the placeholders below verbatim:

```json
"mission": {
  "mavlink_uart": "<your ttyAMAx for ArduPilot FC>",
  "sensor_uart":  "<your ttyAMAx for MTF-02P>",
  "pump_gpio_pin": <BCM pin connected to pump relay IN>
},
"range_sensor": {
  "type": "a"    ← "a" = VL53L3CX (I2C, default),  "b" = TF-Luna/TFMini (UART)
},
"wiper": {
  "wiper_gpio_pin": <BCM pin connected to wiper arm>
}
```

If using sensor B (`"type": "b"`), also set:
```json
"tf_sensor": {
  "uart": "<your ttyAMAx for TF sensor>"
}
```

If running headless (no monitor), also set `"display": false` — OpenCV will crash on a headless Pi if this is left `true`.

---

## Usage

### Mission mode (autonomous drone)

```bash
python main.py --mode mission
```

The drone will:
1. Arm and take off to 1.5 m
2. Yaw-spin slowly to scan the room with YOLO
3. Approach the dirty board (camera centering + range sensor)
4. Activate the pump to clean
5. Return to launch and land

All mission parameters (altitude, speed, pump duration, UART ports, etc.) are in `config.json` under the `"mission"` key.

### Inference mode (vision models, no drone)

#### OCR model — find the word "dirty"

```bash
# Continuous live feed
python main.py --model ocr

# Single frame — print True/False and exit
python main.py --model ocr --once

# Test on a saved image (no camera needed)
python main.py --model ocr --source board.jpg --once
```

#### YOLO model — detect board state

```bash
# Continuous live feed
python main.py --model yolo

# Override confidence threshold
python main.py --model yolo --conf 0.5

# Save every annotated frame to output/
python main.py --model yolo --save

# Test on a video file
python main.py --model yolo --source clip.mp4
```

Press `q` to quit any live window.

### All flags

| Flag | Mode | Default | Description |
|---|---|---|---|
| `--mode` | both | `inference` | `inference` (vision only) or `mission` (full drone flight) |
| `--config` | both | `config.json` | Path to a different config file |
| `--model` | inference only | *(required)* | `ocr` or `yolo` |
| `--source` | inference only | Pi camera | Path to image or video for offline testing |
| `--once` | inference only | off | Process one frame then exit |
| `--conf` | inference only | from config.json | YOLO confidence threshold override |
| `--save` | inference only | off | Write annotated frames to `output/` |

> `--mode mission` only reads `--config`. All other flags are ignored in mission mode.

---

## Mission mode

### State machine

```
IDLE → SCAN → APPROACH → CLEAN → RETURN → DONE
         │         │
   timeout→RETURN  └─ board lost → SCAN
Any exception → ABORTED (safe shutdown + RTL attempted)
```

| State | What happens |
|---|---|
| **IDLE** | Arms the flight controller and initiates takeoff |
| **SCAN** | Slow constant yaw spin; YOLO runs on every frame looking for `dirty_board` |
| **APPROACH** | **Phase 1 — Align:** holds position (vx=0), corrects lateral/vertical until board is centred within `align_threshold_px`. **Phase 2 — Approach:** drives forward proportionally to remaining distance (`vx = kp_forward × (dist − stop_dist)`), naturally decelerating to zero at the board; stops when the range sensor reads ≤ `approach_stop_dist_m` |
| **CLEAN** | Holds position; activates pump for `pump_duration_s` seconds, then actuates the wiper arm sweep |
| **RETURN** | Switches ArduPilot to RTL mode; waits for landing |
| **DONE / ABORTED** | Terminal states — subsystems shut down cleanly |

## Reference

For full model details, training walkthrough, configuration reference, and system architecture, see [system_guide.html](system_guide.html).

---

## License

See [LICENSE](LICENSE).
