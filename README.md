# AeroClean

Autonomous MAVLink drone that cleans dirty dry-erase boards. A **Raspberry Pi 5** acts as the companion computer — running a **YOLO26n object detector** and/or a **Tesseract OCR pipeline** to detect board state, then commanding an **ArduPilot** flight controller over UART to take off, scan the room, approach the board, and trigger a pump to clean it.

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
ssh user@<PI_IP>
```
Replace `user` with your Pi's username and `<PI_IP>` with your Pi's address (e.g. `192.168.1.42`). You are now running commands on the Pi remotely.

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

Once all hardware tests pass (see [Setup → Step 5](#5-test-each-hardware-component)):
```bash
python main.py --mode mission
```
The drone arms, takes off to 1.5 m, yaw-spins to scan for a dirty board, approaches it using camera centering and the range sensor, activates the pump to clean, then returns home and lands. All parameters are in `config.json`.

---

## Project structure

```
AeroClean/
├── main.py               # Entry point — --mode mission | --model ocr | yolo
├── camera.py             # Camera wrapper — Camera A: OV2311 USB (cv2.VideoCapture) | Camera B: IMX708 CSI (picamera2)
├── ocr_model.py          # Model 1: Tesseract OCR pipeline
├── yolo_model.py         # Model 2: YOLO26n NCNN inference
├── mission.py            # Autonomous mission state machine (IDLE→SCAN→APPROACH→CLEAN→RETURN)
├── mavlink_controller.py # pymavlink wrapper — arm, takeoff, velocity commands, RTL
├── sensors.py            # TF-Luna/TFMini UART range (sensor A, default) + VL53L3CX I2C range (sensor B)
├── pump.py               # GPIO pump controller
├── wiper.py              # Wiper arm controller
├── camera_test.py        # Pi camera live view + FPS test
├── sensor_tf_test.py     # TF-Luna / TFMini UART range sensor standalone test
├── sensor_tf_i2c_test.py # VL53L3CX I2C range sensor standalone test
├── sensor_ocr_test.py    # Range sensor + OCR integration test
├── pump_test.py          # Pump test
├── wiper_test.py         # Wiper test
├── preflight_test.py     # Full pipeline test — camera → model → sensor → pump → wiper (no flight)
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
| **Camera** | **Camera A (default):** Arducam OV2311 2MP Global Shutter USB — 1600×1200, 50 fps, plug and play (set `camera_type = "a"`)  **or  Camera B:** Raspberry Pi Camera Module 3 — IMX708, 12MP, CSI ribbon, CDAF autofocus (set `camera_type = "b"`) |
| **Flight controller** | ArduPilot-compatible FC (e.g. Pixhawk) — connected to Pi via UART (confirm path with `ls -l /dev/ttyAMA*`) |
| **Flow sensor** | MicoAir MTF-02P optical flow sensor — connected to the FC optical flow UART port (not the Pi) |
| **Range sensor** | **Sensor A (default):** TF-Luna / TFMini — UART (set `range_sensor.uart` in config.json)  **or  Sensor B:** VL53L3CX ToF — I2C, GPIO 2/3 (pins 3/5), 3 m range (set `range_sensor.type = "b"`) |
| **Pump** | Relay-driven pump on BCM GPIO pin (configurable in `config.json`) |
| **Wiper** | Wiper arm on BCM GPIO pin — actuator type TBD (configurable in `config.json`) |
| **OS** | Raspberry Pi OS Trixie (64-bit) — Debian 13 |
| **Storage** | 32 GB SD card minimum (64 GB recommended for training images) |

---

## System architecture

How all the hardware pieces connect and what each one does.

```
┌─────────────────────────────────────────┐
│           Raspberry Pi 5                │
│                                         │
│  Camera (OV2311 / IMX708) → YOLO detect │
│  TF-Luna/TFMini (UART) →  forward range │
│  Mission state machine                  │
│  GPIO pump pin         →  pump relay    │
│  GPIO wiper pin        →  wiper arm     │
│                                         │
│  pymavlink (MAVLink over UART)          │
└──────────────┬──────────────────────────┘
               │ UART — MAVLink
┌──────────────▼──────────────────────────┐
│           ArduPilot FC                  │
│                                         │
│  Attitude stabilisation (own IMU)       │
│  Motor control                          │
│  GUIDED mode — accepts velocity targets │
│  MTF-02P (UART)      →  optical flow/EKF│
└─────────────────────────────────────────┘
```

ArduPilot handles all low-level stabilisation. The Pi sends body-frame velocity setpoints (`SET_POSITION_TARGET_LOCAL_NED`) and the FC executes them while keeping the drone stable.

### Wiring

**Raspberry Pi connections**

| Pi interface | Device | Purpose |
|---|---|---|
| UART (e.g. `/dev/ttyAMA0`) | ArduPilot FC TELEM port | MAVLink command channel (pymavlink) |
| UART (e.g. `/dev/ttyAMA3`) | TF-Luna / TFMini (sensor A, default) | Forward range for approach controller |
| I2C GPIO 2/3 (pins 3/5) | VL53L3CX (sensor B, alternative) | Forward range — I2C alternative to sensor A |
| GPIO BCM pin (configurable) | Pump relay IN | Cleaning mechanism trigger |
| GPIO BCM pin (configurable) | Wiper arm control wire | Wiper arm actuation |

**ArduPilot FC connections**

| FC interface | Device | Purpose |
|---|---|---|
| TELEM port (UART) | Raspberry Pi | MAVLink — receives velocity targets from pymavlink |
| Optical flow port (UART) | MicoAir MTF-02P | Optical flow input for EKF position hold |
| IMU (onboard) | — | Attitude estimation and stabilisation |
| ESC outputs | Motors | Motor speed control |

> **UART wiring is crossed — Pi TX connects to the device's RX, and Pi RX connects to the device's TX.**
> Connecting TX→TX or RX→RX will produce no data. This applies to all UART devices: ArduPilot FC and TF-Luna/TFMini.
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
- [ ] Camera — **Camera A (default):** Arducam OV2311 USB (plug into USB port)  **or  Camera B:** Raspberry Pi Camera Module 3 (CSI ribbon cable)
- [ ] ArduPilot flight controller — TELEM/UART port wired to Pi UART
- [ ] MicoAir MTF-02P — wired to the FC optical flow UART port (not the Pi)
- [ ] Range sensor — **sensor A (default):** TF-Luna / TFMini (UART, wire to a free Pi UART)  **or  sensor B:** VL53L3CX (I2C, wire to GPIO 2/3, pins 3/5)
- [ ] Pump + relay — relay IN wired to a free BCM GPIO pin
- [ ] Wiper arm — control wire wired to a free BCM GPIO pin
- [ ] 32 GB+ SD card with Raspberry Pi OS Trixie 64-bit (Debian 13)

---

## Setup

Work through these steps in order. Each step builds on the last — do not skip ahead.

### 1. Flash and configure the Pi

Download **Raspberry Pi OS Trixie (64-bit)** (Debian 13) from the official site and flash it with Raspberry Pi Imager. Enable SSH and set your hostname if needed. Once flashed, boot the Pi and SSH in from your laptop (see [Quick Start](#quick-start) above).

### 2. Install dependencies, clone the repo, and set up the Python environment

Complete all four sub-steps in order. The Python packages must go inside the virtual environment — do not run `pip install` before the venv is active.

#### 2a — Update and install system packages

This downloads and installs the Tesseract OCR engine, camera library, and I2C diagnostic tools. It will take a few minutes.

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y tesseract-ocr libcap-dev i2c-tools
# Camera B (IMX708) only — skip if using Camera A (OV2311 USB):
sudo apt install -y python3-picamera2
```

> ✓ **Before continuing:** the install should finish without any `ERROR` lines.

---

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

---

#### 2c — Create and activate the virtual environment

A virtual environment is an isolated Python workspace. It keeps AeroClean's packages separate from the rest of the Pi — this prevents version conflicts and makes the project self-contained.

```bash
python3 -m venv aeroclean_env --system-site-packages
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

---

#### 2d — Install Python dependencies

With the environment active, install all required Python packages:

```bash
pip install -r requirements.txt
```

This installs everything the project needs — pymavlink, Ultralytics YOLO, the Adafruit sensor libraries, and more. It will take a few minutes the first time.

> ✓ **Before continuing:** the install should finish without any `ERROR` lines. Warnings are fine.

---

#### 2e — Add your user to the dialout group

Without this, any attempt to read `/dev/ttyAMA*` or open a MAVLink connection will fail with `Permission denied`.

```bash
sudo usermod -aG dialout $USER
```

Log out and back in (or reboot) for the group change to take effect. Verify:
```bash
groups
```
The output should include `dialout`.

> ✓ **Before continuing:** confirm `dialout` appears in the `groups` output above.

On Pi OS Trixie, `/dev/ttyAMA*` devices are owned `root:root` by default — `dialout` group membership alone is not enough. Add a udev rule to permanently set the group to `dialout` for all UART devices:

```bash
echo 'KERNEL=="ttyAMA[0-9]*", GROUP="dialout", MODE="0660"' | sudo tee /etc/udev/rules.d/99-ttyama.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Verify the rule has applied:
```bash
ls -la /dev/ttyAMA*
```
Expected — the group column should now show `dialout`:
```
crw-rw---- 1 root dialout 204, 64 ... /dev/ttyAMA0
```
If it still shows `root root`, reboot and check again.

> ✓ **Before continuing:** confirm `dialout` appears as the group in `ls -la /dev/ttyAMA*`.

---

### 3. Enable hardware interfaces (camera, UART and I2C)

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

> **Note:** `pinctrl -p` uses physical pin numbers (1–40 on the header), not BCM GPIO numbers. GPIO14 lives on physical pin 8; GPIO2 lives on physical pin 3. The `// GPIOx` comment tells you the BCM number. To see a visual map of every pin on your board, run:
> ```bash
> pinout
> ```
> `pinout` prints a colour-coded diagram of the 40-pin header with GPIO numbers, power rails, and ground pins labelled — use it alongside `pinctrl -p` to match a BCM number to its physical position.

---

#### Sub-step 0 — Set up camera

Set `camera_type` in `config.json` (`_s1`) to match your hardware, then follow the steps for that camera.

**Camera A — OV2311 USB (default, `"camera_type": "a"`)**

UVC-compliant — no `config.txt` changes or reboot needed. Run this **before** plugging in the camera, then again **after** — the new entry is your device:

```bash
ls /dev/video*   # before — note existing entries
# plug in camera
ls /dev/video*   # after — new entry is your camera (e.g. /dev/video0 → camera_device: 0)
```

Set `camera_device` in `config.json` (`_s1`) to that index (default `0` works if no other USB video devices are connected):

```bash
sudo nano config.json   # confirm camera_type: "a" and camera_device index (_s1)
```

> ✓ Confirm a new `/dev/video*` entry appears when the camera is plugged in.

---

**Camera B — IMX708 CSI (`"camera_type": "b"`)**

Open `config.txt` and make these two changes:

```
camera_auto_detect=0
dtoverlay=imx708
```

> If your ribbon cable is in the CAM0 port (lower), use `dtoverlay=imx708,cam0` instead.

Save, reboot, then verify:

```bash
sudo reboot
rpicam-hello --list-cameras   # expected: IMX708 appears in output
```

```bash
sudo nano config.json   # set camera_type: "b" (_s1)
```

> ✓ Confirm `IMX708` appears in the `rpicam-hello` output before continuing.

---

#### Sub-step 1 — Enable UART0 (ArduPilot FC)

Open the file with:
```bash
sudo nano /boot/firmware/config.txt
```
Add the following line, then save and exit: press `Ctrl+X` → `Y` → `Enter`.
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

Run `pinctrl -p` and look for physical pins 8 and 10 — confirm they have changed from `none` to their UART0 alternate functions:

```
Before:
 8: no pd | -- // GPIO14 = none
10: no pd | -- // GPIO15 = none

After (expected):
 8: a4 pn | hi // GPIO14 = TXD0
 9: gnd
10: a4 pu | hi // GPIO15 = RXD0
```
Then confirm the device is present:

```bash
ls -l /dev/ttyAMA*
```

Note the path for the ArduPilot FC — now put it in config.json:
```bash
sudo nano config.json
```
Find the `mission` block (`_s7`) and set:
```json
"mavlink_uart": "/dev/ttyAMA0"
```
Replace `/dev/ttyAMA0` with the actual path from `ls -l /dev/ttyAMA*` above. Save: `Ctrl+X` → `Y` → `Enter`.

---

#### Sub-step 2 — Enable the range sensor interface

Do **only one** of 2A or 2B depending on your hardware. Check `config.json → range_sensor.type` if you are unsure which sensor you have set up.

---

##### Sub-step 2A — Enable additional UART (sensor A: TF-Luna / TFMini — default)

Skip this if you are using a VL53L3CX (go to 2B instead).

> **Note for sensor A builds:** you will need two overlay UARTs — one for the ArduPilot FC (sub-step 1) and one for the TF sensor (this step). The MTF-02P connects to the FC, not the Pi, so no extra Pi UART is needed for it.

Open the file with:
```bash
sudo nano /boot/firmware/config.txt
```
Add a free overlay UART not already taken by the MTF-02P (e.g. `dtoverlay=uart4` or another free number), then save and exit: press `Ctrl+X` → `Y` → `Enter`.
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

Then list all available UARTs:
```bash
ls -l /dev/ttyAMA*
```
`dtoverlay=uartX` creates a new `/dev/ttyAMAX` device. Note the path — now put it in config.json:
```bash
sudo nano config.json
```
Find the `range_sensor` block (`_s5`) and set:
```json
"uart": "/dev/ttyAMAX"
```
Replace `/dev/ttyAMAX` with the actual path. Save: `Ctrl+X` → `Y` → `Enter`.

**Debug check — confirm the sensor is live and transmitting:**

```bash
cat /dev/ttyAMAX   # replace X with your TF sensor UART number, e.g. cat /dev/ttyAMA3
```

You should see a stream of garbled binary data in the terminal — this confirms the sensor is transmitting. Press `Ctrl+C` to stop. If nothing appears, recheck wiring and that the overlay number matches.

> ✓ **Before continuing:** confirm the new UART device appears in `ls -l /dev/ttyAMA*` and you have noted the `/dev/ttyAMAX` path.

---

##### Sub-step 2B — Enable I2C (sensor B: VL53L3CX)

Skip this if you are using a TF-Luna / TFMini (go to 2A instead).

Open the file with:
```bash
sudo nano /boot/firmware/config.txt
```
Add the following line, then save and exit: press `Ctrl+X` → `Y` → `Enter`.
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

Now update config.json to select sensor B:
```bash
sudo nano config.json
```
Find the `range_sensor` block (`_s5`) and confirm:
```json
"type": "b"
```
Save: `Ctrl+X` → `Y` → `Enter`.

---

#### Verified `/boot/firmware/config.txt`

After all sub-steps, your `config.txt` should contain at minimum. Use **only one** of the two blocks below — whichever matches your hardware:

**Sensor A (TF-Luna / TFMini — UART, default):**
```
enable_uart=1
dtoverlay=uartX
```
One overlay UART needed for the TF sensor (sub-step 2A). Replace `uartX` with the number you chose. Add `dtoverlay=uartX` lines for any additional serial peripherals.

**Sensor B (VL53L3CX — I2C):**
```
enable_uart=1
dtparam=i2c_arm=on
```
No overlay UART needed — the VL53L3CX uses I2C. Add `dtoverlay=uartX` lines for any additional serial peripherals.

---

### 4. Configure config.json

Now that you know which `/dev/ttyAMAX` path belongs to each UART (from Step 3), enter those values into `config.json`. **The mission will not start until these are filled in** — it will print a clear error if any required value is still `null`.

---

### 5. Test each hardware component

Run each script in order. Every test must pass before moving on — do not run mission mode until all pass.

#### 5a — Camera

Confirm the camera is detected and delivering frames before anything else.

```bash
python camera_test.py
```
Expected: a startup banner prints, then a live window opens. The terminal only prints when FPS changes — a stable camera will print once and then go silent:
```
[CAM TEST] 1920x1080  28.3 FPS
```

---

#### 5b — OCR AND/OR YOLO inference

Confirm the vision models run on camera frames. No sensors, no drone. Run one or both:

```bash
# OCR — look for the word "dirty" on the board
python main.py --model ocr

# OR — YOLO — classify the board as clean or dirty
python main.py --model yolo
```
Expected: live window with bounding boxes or OCR overlays. The terminal only prints when the detected state changes — hold a dirty board in front of the camera then remove it:
```
[OCR]  active state — dirty
[OCR]  active state — clean
```
```
[YOLO] active state — dirty_board  conf=0.91
[YOLO] active state — no board detected
```
Press `q` to quit.

---

#### 5c — Range sensor

Run the test for whichever sensor you have wired.

**Option A — TF-Luna / TFMini (UART, default):**
```bash
python sensor_tf_test.py
```
Expected: a startup banner prints, a spinner animates while waiting for the first frame, then the terminal only prints when distance changes — move your hand toward and away from the sensor to see readings update:
```
[TF TEST] 0.452 m  (45.2 cm)
```
Requires `range_sensor.uart` to be set in `config.json` first — the script will error clearly if it is not.

**Option B — VL53L3CX (I2C):**
```bash
python sensor_tf_i2c_test.py
```
Expected: a startup banner prints, a spinner animates until the first reading arrives, then the terminal only prints when distance changes:
```
[RANGE TEST] 0.452 m  (45.2 cm)
```

---

#### 5d — Range sensor + OCR together

Confirm the full detection pipeline — this is what the mission APPROACH state does.

**Option A — TF-Luna / TFMini (UART, default):**
```bash
python sensor_ocr_test.py
```

**Option B — VL53L3CX (I2C):**
```bash
python sensor_ocr_test.py --sensor b
```

Expected for both: a startup banner prints, then the terminal only prints on state change (clean → dirty or dirty → clean) or when distance changes. Hold a board marked "dirty" in view then remove it:
```
[TEST] DIRTY detected — range=0.842m
[TEST] CLEAN
```
Window shows bounding box + distance overlay.

---

#### 5e — Pump

Confirm the pump relay fires on the configured GPIO pin. Requires `mission.pump_gpio_pin` to be set in `config.json`.

```bash
python pump_test.py
```

The script prints the banner, then waits for you to press Enter before firing — so you have time to position the pump. Expected output:

```
[PUMP TEST] Firing pump on pin 17 for 5.0s...
[PUMP TEST] Done — relay should have clicked OFF.
[PUMP TEST] If you heard two relay clicks and saw water, the pump is confirmed.
```

You should hear the relay click ON, water flow for the duration, then click OFF. If the relay clicks but no water flows, check the pump power supply and tube connections.

---

#### 5f — Wiper

Confirm the wiper relay fires on the configured GPIO pin. Requires `wiper.wiper_gpio_pin` to be set in `config.json`.

```bash
python wiper_test.py
```

The script prints the banner, then waits for you to press Enter before firing — so you have time to clear the area around the wiper arm. Expected output:

```
[WIPER TEST] Firing wiper on pin 18 for 2.0s...
[WIPER TEST] Done — relay should have clicked OFF.
[WIPER TEST] If you heard two relay clicks and the arm moved, the wiper is confirmed.
```

You should hear the relay click ON, the wiper arm sweep for the duration, then click OFF. If the relay clicks but the arm does not move, check the wiper power supply and mechanical linkage.

---

#### 5g — Preflight (full pipeline)

Run this only after 5a–5f all pass. Confirms the entire chain — camera → model → range sensor → pump → wiper — fires together correctly on the bench with no flight required.

```bash
python preflight_test.py --model ocr
# OR
python preflight_test.py --model yolo
```

A startup banner prints showing the active model, UART path, and trigger distance. A live window opens with the board state overlaid. The terminal only prints on state change or distance change:

```
[OCR]  active state — dirty   [TF] 0.38 m
[OCR]  active state — clean   [TF] 0.38 m
[ACTION] SPRAY → WIPE
```

Hold a dirty board within the trigger distance (`approach_stop_dist_m` in `config.json`) — the pump and wiper should fire automatically. Press `q` in the window or `Ctrl+C` to stop cleanly.

---

#### Final step — Disable the serial login shell

> **Do this only after all test scripts above have passed.** The `cat /dev/ttyAMAX` debug check and every test script that reads a UART require the serial console to still be active — removing it first means you lose the ability to see raw bytes and diagnose wiring problems. Once everything is confirmed working, disable it before running mission mode.
>
> The Pi OS attaches a login console to the primary UART by default. If it is still running when ArduPilot connects, the OS and the flight controller will fight over the same wire and MAVLink will never connect.

**a) Remove the serial console from the kernel command line:**

```bash
sudo nano /boot/firmware/cmdline.txt
```

The file contains a single long line. Find and delete the token `console=serial0,115200` from that line. Leave everything else exactly as-is. Save and exit (`Ctrl+X` → `Y` → `Enter`).

> **Do NOT use `raspi-config` for this step on Pi 5.** On Pi OS Trixie, `raspi-config` edits the wrong file and the change has no effect. Edit `/boot/firmware/cmdline.txt` directly.

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

## Pre-mission checklist

Use this after completing all setup steps to confirm nothing was missed before arming.

### Software

- [ ] System packages installed: `sudo apt install tesseract-ocr libcap-dev i2c-tools`
- [ ] **Camera A (default):** OV2311 USB plugged in — `ls /dev/video*` shows device, `camera_type: "a"` and `camera_device` set in config.json
- [ ] **Camera B only:** `python3-picamera2` installed, `camera_auto_detect=0` + `dtoverlay=imx708` in `config.txt`, `camera_type: "b"` in config.json
- [ ] Virtual environment created and active (`source aeroclean_env/bin/activate`)
- [ ] Python packages installed inside the venv: `pip install -r requirements.txt`
- [ ] Serial login shell disabled — `console=serial0,115200` removed from `/boot/firmware/cmdline.txt` and `serial-getty@ttyAMA0.service` disabled
- [ ] `dtparam=i2c_arm=on` in `/boot/firmware/config.txt` **(sensor B only — VL53L3CX)**
- [ ] `enable_uart=1` and `dtoverlay=uartX` in `/boot/firmware/config.txt`
- [ ] User in `dialout` group (`groups` output includes `dialout`)
- [ ] YOLO weights trained and placed in `weights/best_ncnn_model/`

### config.json

Verify that no `null` values remain before running mission mode.

If running headless (no monitor), also set `"display": false` (section `_s4`) — OpenCV will crash on a headless Pi if this is left `true`.

---

## Usage

> **Do not run `--mode mission` until all steps in Section 5 pass.**

### Mission mode (autonomous drone)

```bash
python main.py --mode mission
```

The drone will:
1. Arm and take off to the configured altitude
2. Yaw-spin slowly to scan the room using the configured detection model (`yolo` or `ocr`)
3. Approach the dirty board (camera centering + range sensor)
4. Activate the pump to clean, then actuate the wiper
5. Return to launch and land

All mission parameters (altitude, speed, pump duration, UART ports, detection model, etc.) are in `config.json` under the `"mission"` key. Key parameters:

| Key | Default | Description |
|---|---|---|
| `detection_model` | `"yolo"` | Detection model: `"yolo"` (visual, NCNN) or `"ocr"` (text-based, Tesseract) |
| `clean_timeout_s` | `30.0` | Max seconds in CLEAN state before forcing RETURN |
| `takeoff_altitude_m` | `0.3` | Target hover altitude in metres |
| `scan_timeout_s` | `60.0` | Abort scan and return if no board found within this time |

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
Any exception → ABORTED (safe shutdown + LAND commanded — GPS-denied safe)
```

| State | What happens |
|---|---|
| **IDLE** | Sets GUIDED mode, arms the flight controller, and initiates takeoff. If ArduPilot rejects the arm command, rejection reasons are printed as `[FC] STATUS: PreArm: ...` lines until the 15 s timeout aborts the mission |
| **SCAN** | Slow constant yaw spin; runs the model set by `detection_model` (`yolo` or `ocr`) on every frame looking for `dirty_board` |
| **APPROACH** | **Phase 1 — Align:** holds position (vx=0), corrects lateral/vertical until board is centred within `align_threshold_px`. **Phase 2 — Approach:** drives forward proportionally to remaining distance (`vx = kp_forward × (dist − stop_dist)`), naturally decelerating to zero at the board; stops when the range sensor reads ≤ `approach_stop_dist_m` |
| **CLEAN** | Holds position; activates pump for `pump_duration_s` seconds, then actuates the wiper arm sweep. Forces RETURN after `clean_timeout_s` if not complete |
| **RETURN** | Switches ArduPilot to RTL mode; waits for landing |
| **DONE / ABORTED** | Terminal states — subsystems shut down cleanly |

---

## Reference

For full model details, training walkthrough, configuration reference, and system architecture, see [system_guide.html](system_guide.html).

---

## Supplementary — Models

### Model 1 — OCR pipeline

The OCR model looks for the word "dirty" written in marker on the board surface.

#### Preprocessing stages

| Stage | What it does |
|---|---|
| **Greyscale** | Strips colour — OCR only needs luminance |
| **CLAHE** | Contrast Limited Adaptive Histogram Equalisation — lifts faint marker strokes that the board's reflective surface would otherwise wash out |
| **Adaptive threshold** | Converts to pure black-and-white using local pixel neighbourhoods — handles shadows and hotspots that a global threshold misses |
| **Morphological close** | Joins broken letter strokes so Tesseract sees whole characters instead of fragments |

Tesseract is configured with `--oem 3 --psm 6` (neural-network engine, assume a uniform block of text). The result is searched for the word `dirty` using a case-insensitive regular expression.

Matched words are highlighted with red bounding boxes; unmatched frames show a green `CLEAN` banner.

---

### Model 2 — YOLO26n board detector

The YOLO model classifies the board as `clean_board` or `dirty_board` and draws a bounding box around it.

#### Why YOLO26n?

YOLO26n is the 2026 nano variant from Ultralytics. It was chosen for this project because it is the fastest and most accurate nano model on ARM hardware:

| Model | Parameters | mAP50-95 (COCO) | Latency on RPi 5 (NCNN) |
|---|---|---|---|
| YOLOv8n | 3.2M | 37.3% | ~120 ms |
| YOLO11n | 2.6M | 39.5% | ~80 ms |
| YOLO26n | 2.6M | 40.1% | ~68 ms |

YOLO26n delivers the best speed and accuracy of any nano variant — the right choice for a resource-constrained device.

#### Export format: NCNN

NCNN is a neural network inference framework optimised for ARM CPUs. It is the fastest export format on the Raspberry Pi 5, achieving ~68ms per frame at 640×640 with no GPU required.

#### Classes

| ID | Name | Description |
|---|---|---|
| 0 | `clean_board` | Erased, usable board surface |
| 1 | `dirty_board` | Marker residue, ghost marks, or heavy smudging |

#### YOLO26 model sizes

| Model | mAP50-95 (COCO val) | Notes |
|---|---|---|
| yolo26n | 40.1% | Used in this project — best speed/accuracy for Pi |
| yolo26s | 47.8% | Good if you can afford ~3× more compute |
| yolo26m | 52.5% | Desktop/laptop training only |
| yolo26l | 54.4% | Requires GPU for practical inference |
| yolo26x | 56.9% | Highest accuracy; not viable on Pi |

n = nano · s = small · m = medium · l = large · x = extra-large

#### Task variants

| Variant | Weights file | What it does |
|---|---|---|
| Detection | `yolo26n.pt` | Axis-aligned bounding boxes — used in this project |
| OBB | `yolo26n-obb.pt` | Oriented (rotated) bounding boxes |
| Segmentation | `yolo26n-seg.pt` | Pixel-level instance masks |
| Pose | `yolo26n-pose.pt` | Keypoint detection (e.g. human joints) |
| Classification | `yolo26n-cls.pt` | Whole-image class label, no boxes |

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
