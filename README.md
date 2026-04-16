# AEROCLEAN

Autonomous MAVLink drone that cleans dirty dry-erase boards. A **Raspberry Pi 5** acts as the companion computer — running a **YOLO11n object detector** and a **Tesseract OCR pipeline** to detect board state, then commanding an **ArduPilot** flight controller over UART to take off, scan the room, approach the board, and trigger a pump to clean it.

---

## Quick start

### Mission mode — autonomous drone flight
```bash
python main.py --mode mission
```
Takes off, yaw-spins to scan for a dirty board, approaches using camera centering + range sensor, cleans via pump, and returns home.

### Inference mode — vision models only (no drone)
```bash
# YOLO board-state detector (live camera)
python main.py --model yolo

# OCR — find the word "dirty" on the board (live camera)
python main.py --model ocr

# Test on a saved image (no Pi camera needed)
python main.py --model yolo --source board.jpg --once
```

Press `q` to quit any live window.

---

## What it does

1. **Captures** a live feed from the 12MP IMX708 sensor with Contrast Detection Auto-Focus (CDAF) via `picamera2`.
2. **OCR mode** — preprocesses each frame (CLAHE → adaptive threshold) and runs Tesseract to find the word "dirty" written in marker. Returns `True` on match.
3. **YOLO mode** — runs a custom-trained YOLO11n model exported to NCNN format. Classifies the board as `clean_board` or `dirty_board` with a bounding box and confidence score.
4. **Mission mode** — full autonomous flight: takeoff → yaw scan → approach → clean → return to home.
5. **Switches** between modes with a single flag — no code changes required.
6. **Saves** annotated frames to `output/` when `--save` is passed.

Model performance after training on a balanced dataset (fill in after you train):

| Class | mAP50 |
|---|---|
| clean_board | — |
| dirty_board | — |
| **Overall** | — |

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
├── sensors.py            # MTF-02P optical flow reader + VL53L3CX I2C range sensor
├── pump.py               # GPIO pump controller
├── wiper.py              # Wiper arm controller (actuator TBD)
├── collect_data.py       # Capture training images from the Pi camera
├── config.json           # All tunable parameters (camera, YOLO, mission)
├── requirements.txt      # Python dependencies
├── requirements_pi.txt   # Pi-specific setup notes + UART wiring instructions
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
| **Flight controller** | ArduPilot-compatible FC (e.g. Pixhawk) — connected to Pi via UART (`/dev/ttyAMA0`) |
| **Flow sensor** | MicoAir MTF-02P optical flow sensor — connected to Pi via UART (second port) |
| **Range sensor** | VL53L3CX ToF sensor — connected to Pi I2C bus (GPIO pins 3/5), 3 m range, forward-facing |
| **Pump** | Relay-driven pump on BCM GPIO pin (configurable in `config.json`) |
| **Wiper** | Wiper arm on BCM GPIO pin — actuator type TBD (configurable in `config.json`) |
| **OS** | Raspberry Pi OS Bookworm (64-bit) — December 2023 or later |
| **Storage** | 32 GB SD card minimum (64 GB recommended for training images) |

---

## What you need on the Pi

Everything the mission requires — hardware, software, and wiring — in one place.

### Hardware checklist

- [ ] Raspberry Pi 5 (4 GB or 8 GB)
- [ ] Camera Module 3 (IMX708) on CSI ribbon
- [ ] ArduPilot flight controller — TELEM/UART port wired to Pi UART (e.g. `/dev/ttyAMA0`)
- [ ] MicoAir MTF-02P — UART wired to a second Pi UART (e.g. `/dev/ttyAMA2`)
- [ ] VL53L3CX ToF sensor — I2C wired to Pi GPIO 2 (SDA, pin 3) and GPIO 3 (SCL, pin 5)
- [ ] Pump + relay — relay IN wired to a free BCM GPIO pin
- [ ] Wiper arm — control wire wired to a free BCM GPIO pin
- [ ] 32 GB+ SD card with Raspberry Pi OS Bookworm 64-bit

### Software checklist

- [ ] `sudo apt install python3-picamera2 tesseract-ocr libcap-dev i2c-tools`
- [ ] `pip install -r requirements.txt`
- [ ] `pip install dronekit pymavlink RPi.GPIO` (mission mode)
- [ ] I2C enabled via `raspi-config → Interface Options → I2C → Enable`
- [ ] Two UARTs enabled in `/boot/firmware/config.txt` (`dtoverlay=uart0`, `dtoverlay=uart2`)
- [ ] Serial login shell **disabled** so `/dev/ttyAMA0` is free for MAVLink
- [ ] User added to `dialout` group: `sudo usermod -aG dialout $USER`
- [ ] YOLO weights trained and copied to `weights/best_ncnn_model/`

### config.json checklist

Before running `--mode mission`, set these `null` values:

```json
"mission": {
  "mavlink_uart": "/dev/ttyAMA0",   ← UART port to ArduPilot FC
  "sensor_uart":  "/dev/ttyAMA2",   ← UART port to MTF-02P
  "pump_gpio_pin": 17               ← BCM pin to pump relay
},
"wiper": {
  "wiper_gpio_pin": 18              ← BCM pin to wiper arm control wire
}
```

See `requirements_pi.txt` for the full step-by-step Pi setup commands.

---

## Setup

### 1. Flash and configure the Pi

Download **Raspberry Pi OS Bookworm (64-bit)** from the official site and flash it with Raspberry Pi Imager. Enable SSH and set your hostname if needed.

Enable the camera interface:
```bash
sudo raspi-config
# Interface Options → Camera → Enable
```

### 2. Update the system and install system dependencies

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y python3-picamera2 tesseract-ocr libcap-dev
```

Verify the camera is detected:
```bash
libcamera-hello --list-cameras
# Expected: "Available cameras" showing IMX708
```

### 3. Clone the repository

```bash
git clone https://github.com/<your-username>/AeroClean.git
cd AeroClean
```

### 4. Create a virtual environment

```bash
python -m venv aeroclean_env
source aeroclean_env/bin/activate
```

### 5. Install Python dependencies

```bash
pip install -r requirements.txt
```

> **Note:** `picamera2` is installed via `apt` in Step 2 — do not install it via pip.  
> If you see OpenCV import errors on the Pi, try `pip install opencv-python-headless` instead.

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

| Flag | Default | Description |
|---|---|---|
| `--mode` | `inference` | `inference` (vision only) or `mission` (full drone flight) |
| `--model` | *(required in inference mode)* | `ocr` or `yolo` |
| `--source` | Pi camera | Path to image or video for offline testing |
| `--once` | off | Process one frame then exit |
| `--conf` | from config.json | YOLO confidence threshold override |
| `--save` | off | Write annotated frames to `output/` |
| `--config` | `config.json` | Path to a different config file |

---

## Mission mode

### State machine

```
IDLE → TAKEOFF → SCAN → APPROACH → CLEAN → RETURN → DONE
                   │         │
             timeout→RETURN  └─ board lost → SCAN
Any exception → ABORTED (safe shutdown + RTL attempted)
```

| State | What happens |
|---|---|
| **IDLE** | Arms the flight controller and initiates takeoff |
| **TAKEOFF** | Climbs to `takeoff_altitude_m` (default 1.5 m), waits to stabilise |
| **SCAN** | Slow constant yaw spin; YOLO runs on every frame looking for `dirty_board` |
| **APPROACH** | **Phase 1 — Align:** holds position (vx=0), corrects lateral/vertical until board is centred within `align_threshold_px`. **Phase 2 — Approach:** drives forward proportionally to remaining distance (`vx = kp_forward × (dist − stop_dist)`), naturally decelerating to zero at the board; stops when VL53L3CX reads ≤ `approach_stop_dist_m` |
| **CLEAN** | Holds position; activates pump for `pump_duration_s` seconds |
| **RETURN** | Switches ArduPilot to RTL mode; waits for landing |
| **DONE / ABORTED** | Terminal states — subsystems shut down cleanly |

### Architecture

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
│  DroneKit (MAVLink over /dev/ttyAMA0)   │
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
| UART `/dev/ttyAMA0` | ArduPilot FC TELEM port | MAVLink command channel (DroneKit) |
| UART `/dev/ttyAMA2` | MTF-02P UART | Optical flow (pymavlink) |
| I2C GPIO 2/3 (pins 3/5) | VL53L3CX ToF sensor | Forward range for approach controller |
| GPIO BCM pin (configurable) | Pump relay IN | Cleaning mechanism trigger |
| GPIO BCM pin (configurable) | Wiper arm control wire | Wiper arm actuation |

See `requirements_pi.txt` for full UART setup instructions (dtoverlay, serial console disable, dialout group).

---

## Model 1 — OCR pipeline

The OCR model looks for the word **"dirty"** written in marker on the board surface.

![OCR preprocessing stages](assets/ocr_pipeline.png)

### Preprocessing stages

| Stage | What it does |
|---|---|
| **Greyscale** | Strips colour — OCR only needs luminance |
| **CLAHE** | Contrast Limited Adaptive Histogram Equalisation — lifts faint marker strokes that the board's reflective surface would otherwise wash out |
| **Adaptive threshold** | Converts to pure black-and-white using local pixel neighbourhoods — handles shadows and hotspots that a global threshold misses |
| **Morphological close** | Joins broken letter strokes so Tesseract sees whole characters instead of fragments |

Tesseract is configured with `--oem 3 --psm 6` (neural-network engine, assume a uniform block of text). The result is searched for the word `dirty` using a case-insensitive regular expression.

Matched words are highlighted with red bounding boxes; unmatched frames show a green `CLEAN` banner.

---

## YOLO11 model reference

### Sizes — same architecture, different capacity

| Model | Parameters | mAP50 (COCO val) | Notes |
|---|---|---|---|
| yolo11**n** | 2.6M | 39.5% | Used in this project — best speed/accuracy for Pi |
| yolo11**s** | 9.4M | 47.0% | Good if you can afford ~3× more compute |
| yolo11**m** | 20.1M | 51.5% | Desktop/laptop training only |
| yolo11**l** | 25.3M | 53.4% | Requires GPU for practical inference |
| yolo11**x** | 56.9M | 54.7% | Highest accuracy; not viable on Pi |

n = nano, s = small, m = medium, l = large, x = extra-large

### Task variants

| Variant | Weights file | What it does |
|---|---|---|
| Detection | `yolo11n.pt` | Axis-aligned bounding boxes — used in this project |
| OBB | `yolo11n-obb.pt` | Oriented (rotated) bounding boxes |
| Segmentation | `yolo11n-seg.pt` | Pixel-level instance masks |
| Pose | `yolo11n-pose.pt` | Keypoint detection (e.g. human joints) |
| Classification | `yolo11n-cls.pt` | Whole-image class label, no boxes |

---

## Model 2 — YOLO11n board detector

The YOLO model classifies the board as `clean_board` or `dirty_board` and draws a bounding box around it.

### Why YOLO11n?

YOLO11n is the latest nano variant from Ultralytics. It was chosen over YOLOv8n for this project because:

| Model | Parameters | mAP50 (COCO) | Latency on RPi 5 CPU (NCNN) |
|---|---|---|---|
| YOLOv8n | 3.2M | 37.3% | ~120ms |
| **YOLO11n** | **2.6M** | **39.5%** | **~80ms** |

YOLO11n is both **smaller and more accurate** than YOLOv8n — the right choice for a resource-constrained device.

### Export format: NCNN

NCNN is a neural network inference framework optimised for ARM CPUs. Exporting to NCNN instead of running raw PyTorch gives approximately 2× faster inference on the Raspberry Pi 5 with no GPU required.

### Classes

| ID | Name | Description |
|---|---|---|
| 0 | `clean_board` | Erased, usable board surface |
| 1 | `dirty_board` | Marker residue, ghost marks, or heavy smudging |

---

## Training walkthrough

This section covers the full pipeline: collect images on the Pi → label in Label Studio → train on Google Colab → export to NCNN → deploy back to the Pi.

### Step 1 — Collect training images

Run `collect_data.py` on the Pi to capture images for each class.

```bash
# Capture 150 images of a clean board (press SPACE to capture, q to quit)
python collect_data.py --class clean_board --count 150

# Capture 150 images of a dirty board
python collect_data.py --class dirty_board --count 150
```

Images are saved to `images/raw/<class_name>/`. Burst mode (auto-capture):
```bash
python collect_data.py --class dirty_board --count 100 --interval 2
```

**Tips for a strong dataset:**
- Vary the lighting: overhead fluorescent, natural window light, lamp at an angle
- Vary board coverage: lightly smudged, heavily marked, partially erased
- Vary your angle: straight-on plus small left/right tilts (~15°)
- Match the distance you will use in production

Aim for **100–200 images per class** minimum.

---

### Step 2 — Label with Label Studio

Label Studio is a free, local labeling tool. Install and launch it:

```bash
pip install label-studio
label-studio start
```

Open `http://localhost:8080` in your browser.

1. **Create a new project** → choose **Object Detection with Bounding Boxes**
2. Set the label names: `clean_board`, `dirty_board`
3. **Import** your images from `images/raw/`
4. **Draw bounding boxes** around the board in each image and assign the correct label

![Label Studio annotation interface](assets/labeling.png)

5. When done: **Export → YOLO format** → save the zip to your project folder
6. Extract the zip so your structure looks like:
   ```
   AeroClean/
     images/train/   images/val/   images/test/
     labels/train/   labels/val/   labels/test/
   ```
   An 80/10/10 train/val/test split is recommended.

---

### Step 3 — Train on Google Colab

Open `train/train_colab.ipynb` in Google Colab:

1. Set the runtime to **GPU → T4** *(Runtime → Change runtime type)*
2. Run all cells in order — the notebook will:
   - Mount your Google Drive
   - Upload your dataset zip
   - Update `dataset.yaml` with the correct paths
   - Train YOLO11n for 100 epochs
   - Plot loss curves, confusion matrix, and precision-recall curve
   - Export the best weights to NCNN format
   - Download `best_ncnn_model.zip`

Key training command inside the notebook:
```python
from ultralytics import YOLO
model = YOLO('yolo11n.pt')
model.train(data='dataset.yaml', epochs=100, imgsz=640, batch=16, device=0)
```

Training takes approximately **10–20 minutes** on a T4 GPU for 100 epochs with ~300 images.

---

### Step 4 — Export to NCNN

The Colab notebook handles this automatically. The relevant cell:
```python
export_model = YOLO('runs/aeroclean_yolo11n/weights/best.pt')
export_model.export(format='ncnn')
# Produces: best_ncnn_model/  (contains *.param and *.bin files)
```

---

### Step 5 — Deploy to the Pi

Copy the exported weights from your laptop to the Pi:
```bash
# From your laptop
scp best_ncnn_model.zip pi@<PI_IP>:~/AeroClean/weights/

# On the Pi
cd ~/AeroClean/weights
unzip best_ncnn_model.zip -d best_ncnn_model
```

Run inference:
```bash
python main.py --model yolo
```

---

## Configuration

All parameters live in `config.json` — no code changes needed.

**Hardware-dependent values** (UART ports and GPIO pins) are set to `null` by default. The mission will refuse to start with a clear error message until they are assigned. Fill them in once the hardware is physically wired.

> JSON does not support comments. Each setting in `config.json` has a companion `_note` key next to it explaining what it does and what values to use. These note keys are ignored by the code.

### Assigning hardware pins and ports

Before running mission mode, open `config.json` and set:

```json
"mission": {
  "mavlink_uart": "/dev/ttyAMA0",   ← UART to ArduPilot FC
  "sensor_uart":  "/dev/ttyAMA2",   ← UART to MTF-02P (different port)
  ...
},
"wiper": {
  "wiper_gpio_pin": 18,             ← BCM pin to wiper arm control wire
  ...
},
  "pump_gpio_pin": 17,              ← BCM pin to pump relay IN
```

To list available UART ports on the Pi:
```bash
ls /dev/ttyAMA*
```

See `requirements_pi.txt` for full UART setup instructions (dtoverlay, serial console, dialout group).

### Camera / inference

| Key | Default | Description |
|---|---|---|
| `resolution` | `[1920, 1080]` | Camera capture resolution `[width, height]` in pixels |
| `framerate` | `30` | Camera frames per second |
| `board_roi` | `null` | `[x, y, w, h]` crop region for OCR; `null` = full frame |
| `yolo_weights` | `"weights/best_ncnn_model"` | Path to the NCNN model directory |
| `yolo_conf` | `0.45` | Detection confidence threshold (0.0 – 1.0) |
| `display` | `true` | Show live OpenCV window (set `false` on headless Pi) |
| `save_output` | `false` | Auto-save annotated frames to `output_dir` |
| `output_dir` | `"output"` | Directory for saved frames |

Setting `board_roi` to a specific rectangle (e.g. `[100, 80, 1720, 920]`) speeds up OCR by limiting Tesseract to the board area only.

### Mission

All keys live under `"mission"` in `config.json`.

| Key | Default | Description |
|---|---|---|
| `mavlink_uart` | `null` ⚠️ | UART port to ArduPilot FC — assign before flying |
| `mavlink_baud` | `57600` | Baud rate — must match ArduPilot `SERIALx_BAUD` |
| `sensor_uart` | `null` ⚠️ | UART port to MTF-02P sensor — assign before flying |
| `sensor_baud` | `115200` | MTF-02P baud rate — fixed by hardware, do not change |
| `takeoff_altitude_m` | `1.5` | Hover altitude in metres after arming |
| `scan_yaw_rate_dps` | `20.0` | Yaw spin speed during board scan (deg/s). 20 = one full rotation in 18 s |
| `scan_timeout_s` | `60.0` | Abort scan and return home if no board found within this time |
| `approach_stop_dist_m` | `0.5` | Stop approaching when range sensor reads this distance (m). Match to wiper arm reach |
| `approach_kp` | `0.4` | Proportional gain for lateral/vertical camera-centering. Tune if drone oscillates sideways |
| `approach_kp_forward` | `0.5` | Proportional gain for forward speed in Phase 2. `vx = kp_forward × (dist − stop_dist)` |
| `align_threshold_px` | `40` | Phase 1 alignment radius in pixels. Drone won't move forward until board is centred within this many pixels |
| `approach_max_speed_ms` | `0.3` | Hard cap on velocity in any axis (m/s) — the controller never exceeds this |
| `approach_cautious_speed_ms` | `0.05` | Forward speed when range sensor has no reading yet. Intentionally slow |
| `pump_gpio_pin` | `null` ⚠️ | BCM GPIO pin to pump relay IN — assign once wired |
| `pump_duration_s` | `5.0` | How long to run the pump before wiping (seconds) |
| `frame_width_px` | `1920` | Must match `resolution[0]` — used by approach controller |
| `frame_height_px` | `1080` | Must match `resolution[1]` — used by approach controller |

### Range sensor

Keys live under `"range_sensor"` in `config.json`.

| Key | Default | Description |
|---|---|---|
| `i2c_address` | `41` | I2C address as a **decimal integer** — default is 0x29 = decimal 41. Change only if XSHUT was used to remap it |

### Wiper

All keys live under `"wiper"` in `config.json`.

| Key | Default | Description |
|---|---|---|
| `wiper_gpio_pin` | `null` ⚠️ | BCM GPIO pin to wiper arm control wire — assign once wired |
| `home_angle` | `90.0` | Wiper arm position when fully retracted (degrees) |
| `press_angle` | `45.0` | Wiper arm position when pressed against the board. Tune to arm geometry |
| `sweep_left` | `30.0` | Left limit of wipe sweep arc (degrees) |
| `sweep_right` | `150.0` | Right limit of wipe sweep arc (degrees) |
| `sweep_passes` | `2` | Number of left-right passes per clean cycle |
| `sweep_speed` | `0.01` | Seconds per degree step during sweep. Lower = faster |

---

## How it works

### Camera module

`camera.py` wraps `picamera2` and configures the IMX708 sensor:

- **Auto-focus** — IMX708 supports Contrast Detection Auto-Focus (CDAF). The camera is set to `AfMode = Continuous` so it refocuses automatically as the board surface changes.
- **Resolution** — the sensor is capable of 12MP (4608×2592), but inference runs at 1920×1080 by default for speed. You can change this in `config.json`.
- **BGR output** — `picamera2` returns RGB arrays; `camera.py` flips to BGR so frames are immediately compatible with OpenCV and Ultralytics.

### Frame routing

`main.py` initialises the camera once, then passes each frame to whichever model is active. Both models receive the same BGR array and return a `(result, annotated_frame)` tuple, so the main loop stays model-agnostic.

```
Camera → BGR frame
           │
    ┌──────┴──────┐
    ▼             ▼
 OCR model    YOLO model
  bool         dict | None
    └──────┬──────┘
           ▼
     OpenCV window / output/
```

---

## Dataset

- Images captured with `collect_data.py` on a Raspberry Pi 5 + IMX708
- Labeled in Label Studio using YOLO bounding box format
- Recommended split: 80% train / 10% val / 10% test
- 2 classes: `clean_board`, `dirty_board`

> Dataset images, labels, trained weights, and output frames are excluded from this repository (see `.gitignore`).

---

## License

See [LICENSE](LICENSE).
