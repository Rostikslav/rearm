"""Calibration and tuning constants for the vision unit"""

# ---- working area, mm (must match kinematics.h) ----
ZONE_W = 400.0
ZONE_H = 184.0

# ---- camera ----
CAMERA_HEIGHT_MM = 320.0 # lens to paper
PIN_HEIGHT_MM = 8.0      # corner marker height
BRICK_HEIGHT_MM = 22.0   # object height

# Resolution the pin coordinates below were measured at
REFERENCE_SIZE = (4608, 2592)

# Capture resolution
CAPTURE_SIZE = (2304, 1296)

# Pin positions in REFERENCE_SIZE pixels, at the point where each pin meets the paper
# Order: top-left, top-right, bottom-right, bottom-left
PINS_PX = [
    (125, 345),
    (4455, 347),
    (4475, 2347),
    (105, 2345),
]

# Where those pins are in the arm's frame (same order)
PINS_MM = [
    (0.0,    0.0),      # top-left     - far edge
    (ZONE_W, 0.0),      # top-right    - far edge
    (ZONE_W, ZONE_H),   # bottom-right - near the arm
    (0.0,    ZONE_H),   # bottom-left  - near the arm
]

# Optical center in REFERENCE_SIZE pixels (the point directly under the lens)
OPTICAL_CENTER_PX = (REFERENCE_SIZE[0] / 2, REFERENCE_SIZE[1] / 2)

# ---- red brick detection (HSV) ----
HUE_BANDS = [(0, 10), (170, 180)] # red needs two ranges in the hue circle
SAT_MIN = 90 # rejects gray shadows regardless of brightness
VAL_MIN = 60 # rejects near-black

MIN_AREA_PX = 1500 # at CAPTURE_SIZE; ignores noise
MAX_AREA_FRAC = 0.10 # a blob larger than this is not our brick

# The arm measures the claw angle counter-clockwise from vertical
ANGLE_COUNTERCLOCKWISE = True
ANGLE_OFFSET_DEG = 90 # from top

# ---- stability gate ----
STATIC_FRAMES = 3
STATIC_INTERVAL = 0.3 # seconds between frames
STATIC_TOL_MM = 4.0 # coordinate change below this value is considered static
STATIC_TOL_DEG = 4.0 # same but for angle (in degrees)

# ---- server ----
HOST = "0.0.0.0"
PORT = 8000
