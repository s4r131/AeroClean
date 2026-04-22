# AeroClean

Autonomous MAVLink drone that cleans dirty dry-erase boards. A **Raspberry Pi 5** acts as the companion computer — running a **YOLO11n object detector** and/or a **Tesseract OCR pipeline** to detect board state, then commanding an **ArduPilot** flight controller over UART to take off, scan the room, approach the board, and trigger a pump to clean it.

---

## Quick Start

AeroClean runs on a **Raspberry Pi**. You can interact with it over SSH from your laptop, or directly using the Raspberry Pi OS desktop with a monitor and keyboard attached.

**Step 1 — Find your Pi's IP address**

On the Pi itself (if you have a monitor attached once):
```bash
hostname -I
```
This prints your Pi's IP address (e.g. `192.168.1.42`). Or check your router's connected devices list.

**Step 2 — Connect from your laptop**

Open a terminal on your laptop and SSH in:
```bash
ssh pi@<PI_IP>
```
Replace `<PI_IP>` with your Pi's address (e.g. `192.168.1.42`). You are now running commands on the Pi remotely.

> **First time here?** You must complete the [Setup](#setup) steps below before running anything. Do not skip ahead.

**Step 3 — Activate the environment (every session)**

Every time you open a new SSH session, activate the Python environment before running anything:
```bash
cd ~/AeroClean
source aeroclean_env/bin/activate
```
Your prompt will change to show `(aeroclean_env)` at the start — that means it's active:
```
(aeroclean_env) pi@raspberrypi:~/AeroClean$
```
If you don't see `(aeroclean_env)`, run the activate command again before continuing.

**Step 4 — Start the mission**

Once all hardware tests pass (see [Setup → Step 4](#4-map-configjson-and-verify-each-hardware-component)):
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
├── sensors.py            # MTF-02P optical flow reader + TF-Luna/TFMini UART range (sensor A, default) + VL53L3CX I2C range (sensor B)
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
| **Flight controller** | ArduPilot-compatible FC (e.g. Pixhawk) — connected to Pi via UART (confirm path with `ls -l /dev/serial*`) |
| **Flow sensor** | MicoAir MTF-02P optical flow sensor — connected to Pi via UART (second port) |
| **Range sensor** | **Sensor A (default):** TF-Luna / TFMini — UART (set `tf_sensor.uart` in config.json)  **or  Sensor B:** VL53L3CX ToF — I2C, GPIO 2/3 (pins 3/5), 3 m range (set `range_sensor.type = "b"`) |
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
│  TF-Luna/TFMini (UART) →  forward range  │
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
| UART (e.g. `/dev/serial0`) | ArduPilot FC TELEM port | MAVLink command channel (DroneKit) |
| UART (e.g. `/dev/serial2`) | MTF-02P UART | Optical flow (pymavlink) |
| UART (e.g. `/dev/serial3`) | TF-Luna / TFMini (sensor A, default) | Forward range for approach controller |
| I2C GPIO 2/3 (pins 3/5) | VL53L3CX (sensor B, alternative) | Forward range — I2C alternative to sensor A |
| GPIO BCM pin (configurable) | Pump relay IN | Cleaning mechanism trigger |
| GPIO BCM pin (configurable) | Wiper arm control wire | Wiper arm actuation |

> **UART wiring is crossed — Pi TX connects to the device's RX, and Pi RX connects to the device's TX.**
> Connecting TX→TX or RX→RX will produce no data. This applies to all UART devices: ArduPilot FC, MTF-02P, and TF-Luna/TFMini.
>
> Full connection pattern for each UART device:
> - Pi **TX** → Device **RX**
> - Pi **RX** → Device **TX**
> - Pi **GND** → Device **GND**
> - Pi **3.3 V or 5 V** → Device **VIN** (check your sensor's voltage spec before wiring)

---

## Hardware checklist

Gather everything before starting setup.

- [ ] Raspberry Pi 5 (4 GB or 8 GB)
- [ ] Camera Module 3 (IMX708) on CSI ribbon
- [ ] ArduPilot flight controller — TELEM/UART port wired to Pi UART
- [ ] MicoAir MTF-02P — UART wired to a second Pi UART
- [ ] Range sensor — **sensor A (default):** TF-Luna / TFMini (UART, wire to a free Pi UART)  **or  sensor B:** VL53L3CX (I2C, wire to GPIO 2/3, pins 3/5)
- [ ] Pump + relay — relay IN wired to a free BCM GPIO pin
- [ ] Wiper arm — control wire wired to a free BCM GPIO pin
- [ ] 32 GB+ SD card with Raspberry Pi OS Bookworm 64-bit

---

## Setup

Work through these steps in order. Each step builds on the last — do not skip ahead.

### 1. Flash and configure the Pi

Download **Raspberry Pi OS Bookworm (64-bit)** from the official site and flash it with Raspberry Pi Imager. Enable SSH and set your hostname if needed. Once flashed, boot the Pi and SSH in from your laptop (see [Quick Start](#quick-start) above).

### 2. Install dependencies, clone the repo, and set up the Python environment

Complete all four sub-steps in order. The Python packages must go inside the virtual environment — do not run `pip install` before the venv is active.

#### 2a — Update and install system packages

This downloads and installs the camera library, OCR engine, and I2C diagnostic tools. It will take a few minutes.

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y python3-picamera2 tesseract-ocr libcap-dev i2c-tools
```

Then verify the camera is detected:
```bash
libcamera-hello --list-cameras
```
The output should contain `IMX708` if the camera is detected correctly.

> ✓ **Before continuing:** you should see `IMX708` in the output above. If the camera is not listed, check the CSI ribbon cable connection and try again.

#### 2b — Clone the repository

This downloads the AeroClean code onto your Pi.

```bash
git clone https://github.com/s4r131/AeroClean.git
cd AeroClean
```

Verify the files are there:
```bash
ls
```
You should see `main.py`, `config.json`, `requirements.txt`, and the rest of the project files.

> ✓ **Before continuing:** confirm `main.py` and `config.json` appear in the output above.

#### 2c — Create and activate the virtual environment

A virtual environment is an isolated Python workspace. It keeps AeroClean's packages separate from the rest of the Pi — this prevents version conflicts and makes the project self-contained.

```bash
python -m venv aeroclean_env
source aeroclean_env/bin/activate
```

Your prompt will change — look for `(aeroclean_env)` at the start:
```
# Before:
pi@raspberrypi:~/AeroClean$

# After (expected):
(aeroclean_env) pi@raspberrypi:~/AeroClean$
```

> ✓ **Before continuing:** confirm your prompt shows `(aeroclean_env)`. If it doesn't, run `source aeroclean_env/bin/activate` again.

**Important:** every time you open a new SSH session, you must run `source aeroclean_env/bin/activate` again before running any scripts. The environment does not stay active between sessions.

#### 2d — Install Python dependencies

With the environment active, install all required Python packages:

```bash
pip install -r requirements.txt
```

This installs everything the project needs — DroneKit, pymavlink, Ultralytics YOLO, the Adafruit sensor libraries, and more. It will take a few minutes the first time.

> ✓ **Before continuing:** the install should finish without any `ERROR` lines. Warnings are fine.

### 3. Enable hardware interfaces (UART and I2C)

The Pi's UART and I2C ports are **off by default**. These steps turn them on by editing a config file and rebooting. Without this, the sensors and flight controller will not be detected at all.

All changes go in `/boot/firmware/config.txt`. After each change, reboot and use `pinctrl -p` to confirm it worked — this command shows the live state of every GPIO pin on the 40-pin header.

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

#### Sub-step 1 — Enable UART0 (ArduPilot FC)

Open the file with:
```bash
sudo nano /boot/firmware/config.txt
```
Add the following line, then save and exit: press `Ctrl+O` → `Enter` → `Ctrl+X`.
```
enable_uart=1
```

Then reboot:
```bash
sudo reboot
```

After rebooting, reconnect via SSH, then re-enter the project and activate the environment:
```bash
cd ~/AeroClean
source aeroclean_env/bin/activate
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

> ✓ **Before continuing:** confirm `GPIO14 = TXD0` and `GPIO15 = RXD0` in your `pinctrl -p` output. If you still see `= none`, the line was not saved correctly — open `config.txt` again and confirm `enable_uart=1` is present and not commented out.

---

#### Sub-step 2 — Enable the range sensor interface

Do **only one** of 2A or 2B depending on your hardware. Check `config.json → range_sensor.type` if you are unsure which sensor you have set up.

---

##### Sub-step 2A — Enable additional UART (sensor A: TF-Luna / TFMini — default)

Skip this if you are using a VL53L3CX (go to 2B instead).

> **Note for sensor A builds:** you will need three UARTs total — UART0 (ArduPilot FC, sub-step 1), one for the MTF-02P (sub-step 3), and this one for the TF sensor. Plan your `dtoverlay=uartX` entries before rebooting.

Open the file with:
```bash
sudo nano /boot/firmware/config.txt
```
Add a free overlay UART not already taken by the MTF-02P (e.g. `dtoverlay=uart4` or another free number), then save and exit: press `Ctrl+O` → `Enter` → `Ctrl+X`.
```
dtoverlay=uartX
```

Then reboot:
```bash
sudo reboot
```

After rebooting, reconnect via SSH, then re-enter the project and activate the environment:
```bash
cd ~/AeroClean
source aeroclean_env/bin/activate
```

Run `pinctrl -p` and confirm the new UART pins changed from `none` to an alternate function — the exact pins depend on which overlay you chose.

Then find the device path:
```bash
ls -l /dev/serial*
```
This shows symlinks like `/dev/serial0 -> ttyAMA0` — use the **`/dev/serialX` path directly** in config.json (not the `ttyAMAx` it points to). Note the alias for the TF sensor — it goes into `config.json → tf_sensor.uart` (section `_s6`).

**Debug check — confirm raw bytes are arriving before running any scripts:**

From the `ls -l /dev/serial*` output, find the `ttyAMAx` name that your TF sensor port points to (e.g. `ttyAMA3`). Then run:
```bash
sudo cat /dev/ttyAMA3
```
Replace `ttyAMA3` with whatever name appeared in the symlink output. If the sensor is powered and wired correctly you will see a stream of garbled characters — that is the raw binary frames from the TF sensor. Press `Ctrl+C` to stop.

If you see nothing, check TX/RX wiring (swap them if needed), confirm the sensor has power, and re-check which overlay you added in config.txt.

> ✓ **Before continuing:** confirm the new UART device appears in `ls -l /dev/serial*`, raw bytes appear with `sudo cat`, and you have noted the `/dev/serialX` path.

---

##### Sub-step 2B — Enable I2C (sensor B: VL53L3CX)

Skip this if you are using a TF-Luna / TFMini (go to 2A instead).

Open the file with:
```bash
sudo nano /boot/firmware/config.txt
```
Add the following line, then save and exit: press `Ctrl+O` → `Enter` → `Ctrl+X`.
```
dtparam=i2c_arm=on
```

Then reboot:
```bash
sudo reboot
```

After rebooting, reconnect via SSH, then re-enter the project and activate the environment:
```bash
cd ~/AeroClean
source aeroclean_env/bin/activate
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

> ✓ **Before continuing:** confirm `29` appears in the `i2cdetect` grid above.

---

#### Sub-step 0 — Disable the serial login shell

> **Do this after enabling your UARTs and I2C above, before the mission step.** The Pi OS attaches a login console to the primary UART by default. If it is still running when ArduPilot connects, the OS and the flight controller will fight over the same wire and MAVLink will never connect.

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

After rebooting, reconnect via SSH, then re-enter the project and activate the environment:
```bash
cd ~/AeroClean
source aeroclean_env/bin/activate
```

Verify nothing is holding the UART:
```bash
ls -l /proc/tty/driver/ | grep serial
```
This should return nothing — if a getty process appears, the service did not disable correctly.

> ✓ **Before continuing:** confirm the command above returns no output.

---

#### Sub-step 3 — Enable additional UART (MTF-02P optical flow sensor)

> The ArduPilot FC (sub-step 1) and the MTF-02P both use UART — each needs its own port and its own `dtoverlay=uartX` line in `config.txt`. Sensor A builds need a third port for the TF sensor (sub-step 2A).

Open the file with:
```bash
sudo nano /boot/firmware/config.txt
```
Add a free overlay UART for the MTF-02P (e.g. `uart3`), then save and exit: press `Ctrl+O` → `Enter` → `Ctrl+X`.
```
dtoverlay=uartX
```

Pin assignments vary by UART number — always verify with `pinctrl -p` after rebooting.

Then reboot:
```bash
sudo reboot
```

After rebooting, reconnect via SSH, then re-enter the project and activate the environment:
```bash
cd ~/AeroClean
source aeroclean_env/bin/activate
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

> ✓ **Before continuing:** confirm `GPIO8 = TXD3` and `GPIO9 = RXD3` in your `pinctrl -p` output.

---

#### Sub-step 4 — Add your user to the dialout group

Without this, any attempt to read `/dev/ttyAMA*` or open a DroneKit connection will fail with `Permission denied`.

```bash
sudo usermod -aG dialout $USER
```

Log out and back in (or reboot) for the group change to take effect. Verify:
```bash
groups
```
The output should include `dialout`.

> ✓ **Before continuing:** confirm `dialout` appears in the `groups` output above.

---

#### Verified `/boot/firmware/config.txt`

After all sub-steps, your `config.txt` should contain at minimum:

**Sensor A (TF-Luna / TFMini — UART, default):**
```
enable_uart=1
dtoverlay=uartX
dtoverlay=uartY
```
Two overlay UARTs needed: one for MTF-02P (sub-step 3) and one for TF sensor (sub-step 2A). Replace `uartX` and `uartY` with the numbers you chose.

**Sensor B (VL53L3CX — I2C):**
```
enable_uart=1
dtparam=i2c_arm=on
dtoverlay=uartX
```
One overlay UART needed for the MTF-02P (sub-step 3). Replace `uartX` with the number you chose.

Add more `dtoverlay=uartX` lines if you have additional serial peripherals.

---

#### Find your device paths, then update config.json

Run `ls -l /dev/serial*` to see the named aliases for each UART:

```bash
ls -l /dev/serial*
```
Example output:
```
/dev/serial0 -> ttyAMA0   ← primary UART (ArduPilot FC via enable_uart=1)
/dev/serial1 -> ttyAMA5   ← mini UART (Bluetooth)
/dev/serial2 -> ttyAMA3   ← overlay UART (e.g. MTF-02P)
```
**Use the `/dev/serialX` path directly in config.json** — not the `ttyAMAx` it points to. This is what makes the port reliably accessible.

Once you know which alias belongs to each device, open `config.json`:
```bash
sudo nano config.json
```
Find the `_s8` section (Mission) and fill in the paths, then save and exit: press `Ctrl+O` → `Enter` → `Ctrl+X`.
```json
"mavlink_uart": "/dev/serial0",
"sensor_uart":  "/dev/serial2"
```
Replace the numbers with the aliases you found above. **Values must stay inside double quotes** — e.g. `"/dev/serial0"`, not `/dev/serial0`.

---

### 4. Map config.json and verify each hardware component

Now that you know which `/dev/serialX` alias belongs to each UART (from Step 3), you need to enter those values into `config.json`. **The mission will not start until these are filled in** — it will print a clear error if any required value is still `null`.

**Do this before attempting a mission.** Every interface you enabled in Step 3 must be mapped in `config.json` and confirmed working with its test script.

#### 4a — Fill in config.json

First find your UART device paths:
```bash
ls -l /dev/serial*
```
Sensor B (VL53L3CX) only — confirm the I2C sensor is visible:
```bash
sudo i2cdetect -y 1
```

Open `config.json`:
```bash
sudo nano config.json
```
Find the `_s8` section (Mission) and fill in your UART aliases, then save and exit: press `Ctrl+O` → `Enter` → `Ctrl+X`.
```json
"mavlink_uart": "/dev/serial0",
"sensor_uart":  "/dev/serial2"
```
Replace the numbers with the aliases from `ls -l /dev/serial*`. **Values must stay inside double quotes** — e.g. `"/dev/serial0"`, not `/dev/serial0`.

Find the `_s5` section (Range sensor) and set your sensor type. Default is `"a"` (TF-Luna/TFMini). Change to `"b"` if using VL53L3CX:
```json
"type": "a"
```
If using sensor A (TF-Luna, default), find the `_s6` section (TF-Luna / TFMini) and set:
```json
"uart": "/dev/serial3"
```
Replace the number with the alias for the TF sensor from `ls -l /dev/serial*`. **The value must stay inside double quotes** — e.g. `"/dev/serial3"`. If using sensor B (VL53L3CX), `i2c_address` in `_s5` only needs changing if the sensor was remapped from `0x29`.

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

**Option A — TF-Luna / TFMini (UART, default):**
```bash
python sensor_tf_test.py
```
Expected:
```
[TF TEST] 0.452 m  (45.2 cm)  | strength=412  | temp=32.1 C
```
Requires `tf_sensor.uart` to be set in `config.json` first — the script will error clearly if it is not.

**Option B — VL53L3CX (I2C):**
```bash
python sensor_range_test.py
```
Expected:
```
[RANGE TEST] 0.452 m  (45.2 cm)
```

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

**Option A — TF-Luna / TFMini (UART, default):**
```bash
python sensor_ocr_test.py
```

**Option B — VL53L3CX (I2C):**
```bash
python sensor_ocr_test.py --sensor b
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
- [ ] `dtparam=i2c_arm=on` in `/boot/firmware/config.txt` **(sensor B only — VL53L3CX)**
- [ ] `enable_uart=1` and `dtoverlay=uartX` in `/boot/firmware/config.txt`
- [ ] User in `dialout` group (`groups` output includes `dialout`)
- [ ] YOLO weights trained and placed in `weights/best_ncnn_model/`

### config.json

Set these `null` values before running mission mode. Run `ls -l /dev/serial*` to find the device aliases — use the `/dev/serialX` path, not the raw `ttyAMAx`. Do not copy the placeholders below verbatim:

```json
"mission": {
  "mavlink_uart": "<your /dev/serialX for ArduPilot FC>",
  "sensor_uart":  "<your /dev/serialX for MTF-02P>",
  "pump_gpio_pin": <BCM pin number for pump relay IN>
},
"range_sensor": {
  "type": "a"
},
"wiper": {
  "wiper_gpio_pin": <BCM pin number for wiper arm>
}
```

Set `range_sensor.type` to `"a"` for TF-Luna/TFMini (UART, default) or `"b"` for VL53L3CX (I2C).

If using sensor A (TF-Luna, default), also set:
```json
"tf_sensor": {
  "uart": "<your /dev/serialX for TF sensor>"
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
| `--model` | inference only | `ocr` | `ocr` (default) or `yolo` |
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

## Supplementary — Benewake Binary Frame Protocol

The TF-Luna and TFMini sensors use a 9-byte binary frame over UART. Every reading is delivered in this exact structure:

```
Byte 0    Byte 1    Byte 2    Byte 3    Byte 4    Byte 5    Byte 6    Byte 7    Byte 8
0x59      0x59      DIST_L    DIST_H    STR_L     STR_H     TEMP_L    TEMP_H    CHECKSUM
```

| Byte(s) | Name | Description |
|---|---|---|
| 0–1 | Header | Always `0x59 0x59` — marks the start of every frame |
| 2–3 | Distance | Distance in centimetres, little-endian. `dist_cm = byte2 \| (byte3 << 8)` |
| 4–5 | Strength | Signal strength — how much light reflected back. Higher = more reliable reading |
| 6–7 | Temperature | Chip temperature. `temp_c = (byte6 \| (byte7 << 8)) / 8.0 - 256` |
| 8 | Checksum | Sum of bytes 0–7 truncated to 8 bits: `sum(frame[:8]) & 0xFF` |

**Special values:**
- `0xFFFF` in the distance bytes means the reading is out of range or invalid — treat as no reading
- Checksum mismatch means the frame was corrupted in transit — discard it and read the next one

**Little-endian** means the low byte comes first. So a distance of 150 cm would be stored as `byte2 = 0x96, byte3 = 0x00` — you shift byte3 left 8 bits and OR it with byte2 to reconstruct the full number.

---

## License

See [LICENSE](LICENSE).
