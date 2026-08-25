# R.E.A.R.M. — Recognition-Enabled Automated Replacement Manipulator

R.E.A.R.M. is a robotic manipulation system designed to detect an object inside a defined working area, determine its position and orientation, pick it up, and move it to a predefined destination.

The system is divided into three cooperating units:

- **Main Unit** — controls the arm and coordinates the complete system.
- **AUX Unit** — controls the dedicated claw gripping motor.
- **Vision Unit** — detects the target object and converts camera measurements into real coordinates used by the robot.

## System Architecture

```text
┌──────────────────────────┐
│       Vision Unit        │
│      Raspberry Pi        │
│                          │
│     Camera + OpenCV      │
│     Object detection     │
│  Coordinate conversion   │
└────────────┬─────────────┘
             │
       Wi-Fi / HTTP
             │
             ▼
┌──────────────────────────┐
│        Main Unit         │
│          ESP32           │
│                          │
│    System coordination   │
│    Inverse kinematics    │
│    Arm motor control     │
│    Homing / sequencing   │
└────────────┬─────────────┘
             │
            I2C
             │
             ▼
┌──────────────────────────┐
│         AUX Unit         │
│          ESP32           │
│                          │
│    Grab motor control    │
│    Encoder monitoring    │
│    Grip state handling   │
└──────────────────────────┘
```

The **Main Unit** is the coordinator. It requests object information from the Vision Unit, calculates the required arm configuration, controls the arm joints, and sends grip commands to the AUX Unit.


# Main Unit

## Purpose

The Main Unit runs on an **ESP32** connected to an **EVShield**. It is responsible for the overall operation of R.E.A.R.M. and acts as the central controller between the Vision Unit, the robotic arm, and the AUX Unit.

Its main responsibilities are:

- initializing the system,
- communicating with the Vision Unit,
- communicating with the AUX Unit,
- calculating motor positions from Cartesian coordinates,
- controlling the arm motors,
- executing the pick-and-place sequence,
- detecting communication and movement failures.
- homing the arm,

The Main Unit directly controls four arm motors:

- base rotation,
- main arm,
- auxiliary arm,
- claw rotation.

The open/close movement of the gripper itself is not driven by the Main Unit. That motor is controlled separately by the AUX Unit, since an EV Shield provides only four outputs.

## Initialization

When the Main Unit starts, it initializes the ESP32, I2C interface, EVShield, and touch sensors.

It then checks whether the AUX Unit is available. The system waits for the AUX controller to appear on the I2C bus and stops startup if it cannot be reached.

The Main Unit also attempts to connect to the Vision Unit through Wi-Fi. Failure to contact the Vision Unit during startup is not considered fatal, because the connection can be retried later while the system is running.

After communication initialization, the arm motors are homed. Touch sensors are used as physical reference switches so that the controller can establish known encoder positions.

Finally, the gripping mechanism is manually placed in its fully open position. The Main Unit then tells the AUX Unit to use this position as the zero reference of the grab motor. Claw opening is not automated due to significantly higher structural complexity for little return.

Once these steps are complete, the system is ready for an object to be placed in the working area.

## Arm Control

The Main Unit uses the EVShield to control the motors connected to the mechanical arm.

Motor movement is based on encoder positions rather than only running motors for a fixed amount of time. The software provides operations for:

- relative movement,
- absolute movement,
- stopping individual motors,
- stopping all motors,
- reading encoder positions,
- checking motor status,
- waiting for movements to finish.

Movement operations include timeouts so that the system does not wait indefinitely if a motor fails to reach its target.

### Homing

Before normal motion, the arm must establish a known reference pose.

Each arm joint has a corresponding touch sensor. During homing, the motors move toward their physical reference positions until the appropriate switches are reached. Encoder values can then be reset relative to these known positions.

Homing is also used as a recovery step when a pick-and-place sequence fails.

## Inverse Kinematics

The Vision Unit provides object coordinates in the robot's working coordinate system. These Cartesian coordinates cannot be sent directly to the motors, so the Main Unit converts them into joint angles.

The kinematics module calculates the required positions for:

- the rotating base,
- the main arm,
- the auxiliary arm,
- the rotational claw joint.

The calculation uses the physical geometry of the robot, such as arm segment lengths, base position, gear ratios etc.

The resulting joint angles are converted into motor encoder degrees using the configured gear ratios.

If a requested position produces invalid mathematical results, the pose is marked invalid instead of allowing an unsafe motor command to be generated. The error is then propagated to the main control loop, the sequence is discarded and started over as soon as user confirmation in form of serial message "ok" arrives.

## Object Data

The Main Unit expects the Vision Unit to provide an object description containing:

| Field | Meaning |
|---|---|
| `x` | Object center X coordinate in millimetres |
| `y` | Object center Y coordinate in millimetres |
| `w` | Width of the short side of the object |
| `r` | Object rotation in degrees |
| `valid` | Whether the measurement can be used |

The width is included in the reported object data, although the current gripping sequence uses a fixed closing distance rather than calculating the gripper position directly from the measured width due to high motor play which makes the grab-by-given-width strategy unreliable. The width is therefore used only for debugging purposes.

## Pick-and-Place Sequence

Once valid object data is received, the Main Unit performs the following sequence:

1. **Hover above the object**  
   The arm moves to the detected X/Y position while remaining above the object.

2. **Lower the claw**  
   The arm moves down to the configured gripping height.

3. **Grip the object**  
   The Main Unit commands the AUX Unit to close the gripper.

4. **Lift the object**  
   The arm returns to the hover height.

5. **Move to the destination**  
   The object is transported to a predefined drop-off position.

6. **Lower the object**  
   The arm moves down to the drop height.

7. **Release the object**  
   The AUX Unit is commanded to return the gripping motor to its open position.

8. **Return home**  
   The arm is homed again and the gripping zero is re-established.

The sequence is blocking: each important movement is completed before the next stage begins.

## Failure Handling

The Main Unit performs several checks during operation.

Examples include:

- AUX Unit not responding,
- invalid I2C status or checksum,
- AUX Unit timeout,
- Vision Unit connection failure,
- invalid or unavailable object data,
- motor movement timeout,
- invalid kinematic solution,
- failed homing.

An invalid Vision result does not immediately stop the system. It normally means either that no target was detected or that the object has not yet become stationary, so the Main Unit waits and requests another reading.

A failure during mechanical movement aborts the current sequence. The Main Unit stops the arm and attempts to re-home it before another operation.

# AUX Unit

## Purpose

The AUX Unit consists of a second **ESP32 and EVShield** dedicated to the claw's gripping motor.
This motor controls the physical opening and closing of the claw. It is a separate unit simply because there are not enough ports on the main EVShield.

## I2C Configuration

The AUX ESP32 performs two I2C roles at the same time:

- **master** toward its local EVShield,
- **slave** toward the Main ESP32.

Because the two roles cannot use the same ESP32 I2C peripheral in this configuration, the AUX Unit uses two buses.
The local EVShield is connected through the default `Wire` bus, while communication with the Main Unit uses `Wire1`.
The AUX Unit appears to the Main controller at I2C address `0x09`.
A common ground between the two ESP32 boards is required.

## Grip Zero Reference

The grabbing motor does not have a physical homing or limit switch due to significantly higher complexity.

Because of this, the software cannot automatically determine when the claw is fully open. At system startup, the user manually opens the claw and confirms the position.

The Main Unit then sends a `SET_ZERO` command. The AUX Unit resets the grip motor encoder and treats that position as motor position `0`.

Until this reference has been established, the AUX Unit remains in the `UNHOMED` state and refuses normal movement commands.

## Commands

The AUX communication protocol supports the following commands:

| Command | Purpose |
|---|---|
| `PING` | Basic communication command |
| `MOVE_TO` | Move to an absolute grip motor position |
| `MOVE_BY` | Move by a relative number of motor degrees |
| `HOME` | Return to the remembered zero position |
| `SET_ZERO` | Set the current motor position as zero |
| `STOP` | Stop the grab motor |

The target values are represented as signed 16-bit motor-degree values.

## AUX States

The AUX Unit reports one of four states:

| State | Meaning |
|---|---|
| `IDLE` | Ready to accept a command |
| `BUSY` | Grip motor is currently moving |
| `UNHOMED` | Zero reference has not been established |
| `ERROR` | A command, transfer, or movement failed |

The Main Unit checks these states after issuing commands and normally waits until the AUX Unit returns to `IDLE`.

## Grip Movement

When a movement command is accepted, the AUX Unit starts the grab motor and continues monitoring its encoder position.

The controller considers the movement complete when either:

- the target encoder position has been reached within the configured tolerance, or
- the motor has stopped making progress for a short period.

The second condition is important during gripping. When the claw closes onto a physical object, the object can prevent the motor from reaching the requested encoder position. In this case, a stall is treated as a normal successful grip rather than automatically being treated as an error.

A longer overall timeout still exists to detect failed or abnormal movement.

## Command Processing

Incoming I2C frames are received through the AUX Unit's slave interface.

The receive handler performs only lightweight validation and stores the command. The main program loop then processes the command and controls the motor.

This keeps motor and EVShield operations outside the I2C interrupt handler.

The current status and encoder position are cached so that the AUX Unit can respond quickly when the Main Unit requests its state.

# Vision Unit

## Purpose

The Vision Unit runs on a **Raspberry Pi** connected to a camera.

It detects the target object in the robot's working area and converts the detected position from camera pixels into physical millimeter coordinates used by the Main Unit.

The Vision software is implemented in Python and uses OpenCV for image processing.

The current implementation is specifically configured to detect a **red object** to make the deteciton easier.

## Camera Capture

The Raspberry Pi camera is configured for still-image capture.

After the camera starts, automatic exposure and white balance are initially allowed to settle. Their resulting settings are then locked.

This prevents brightness or colour changes between captures from causing the same stationary object to appear different between frames.

Captured frames are rotated so that all later image processing uses the same orientation as the robot's coordinate system.

## Working Area Calibration

Four known reference points define the robot's rectangular working area.

Their locations are stored in two forms:

- camera pixel coordinates,
- physical coordinates in millimetres.

Using these corresponding points, OpenCV creates a perspective transformation from image coordinates into the robot coordinate system.

The configured working area is approximately `400 mm × 184 mm`

This coordinate system matches the geometry used by the Main Unit's kinematics calculations.

## Object Detection

Detection begins by converting the captured image from BGR into HSV color space.

Because red wraps around the beginning and end of the HSV hue range, two hue bands are used. Saturation and brightness thresholds reject dark or low-colour regions.

After thresholding, morphological operations clean the binary mask and then the contours are extracted from it.

The software expects a single target object, so the largest suitable red contour is used while smaller regions are treated as noise.

Very small and excessively large detections are rejected.

## Position and Orientation

Once a valid contour is found, OpenCV calculates a minimum-area rotated rectangle around the object.

From this rectangle, the Vision Unit obtains:

- center position,
- long-axis direction,
- short-side width,
- object orientation.

Pixel measurements are transformed into physical millimeters.

The reported orientation is normalized into the angle convention expected by the robotic arm.

## Perspective and Parallax Correction

A perspective transform converts points in the camera image into points on the robot's working surface.

The code also compensates for the fact that the reference markers and the detected object have height above the working surface.

Without this correction, objects above the surface would appear shifted outward relative to the camera's optical center.

The parallax correction uses:

- camera height,
- reference marker height,
- object height,
- optical center of the image.

This improves the relationship between the detected image position and the physical position used by the robot.

## Stability Check

The Vision Unit does not immediately return the result from a single image.

Several frames are captured with a short interval between them.

A result is accepted only if:

- an object is detected in every required frame,
- its X/Y position remains within the configured tolerance,
- its orientation remains within the configured angular tolerance.

If these conditions are not met, the object is considered to be moving.

For a stable object, the final reported values are calculated using the median of the individual measurements. This reduces the influence of a single inaccurate frame.

## HTTP Server

The Vision Unit exposes a small HTTP service.

### `GET /ping`

Used by the Main Unit to verify that the Vision Unit is online.

Example response:

```json
{
  "ok": true
}
```

### `GET /object`

Requests a stable object detection.

Example valid response:

```json
{
  "valid": true,
  "x": 205,
  "y": 96,
  "w": 16,
  "r": 87
}
```

If the scene is not usable, the response contains `valid: false`.

For example:

```json
{
  "valid": false,
  "reason": "moving"
}
```

Possible invalid conditions include:

- no target object detected,
- object still moving,
- internal detection error.

## Preview Mode

The Vision Unit can also be run independently of the robotic arm in preview mode.

This captures an image, runs object detection, and saves debugging images such as the original capture, mask, and detected rectangle.

This is useful when adjusting camera position, thresholds, or calibration values.

# Communication Between Units

R.E.A.R.M. uses two separate communication mechanisms:

```text
Vision Unit  <-- Wi-Fi / HTTP -->  Main Unit  <-- I2C -->  AUX Unit
```

The Main Unit is therefore the communication hub of the complete system.

There is no direct communication between the Vision Unit and AUX Unit.

## Main Unit ↔ Vision Unit

Communication between the Main ESP32 and Raspberry Pi uses **Wi-Fi and HTTP**.

The Raspberry Pi acts as the Wi-Fi access point and hosts the Vision HTTP server. The Main Unit connects to this network as a station.

The Main Unit communicates using simple HTTP `GET` requests.

### Connection Check

```text
Main Unit                         Vision Unit
    │                                  │
    │---------- GET /ping ------------>│
    │                                  │
    │<--------- {"ok": true} ----------│
    │                                  │
```

### Object Request

```text
Main Unit                         Vision Unit
    │                                  │
    │--------- GET /object ----------->│
    │                                  │
    │                           capture frames
    │                           detect object
    │                           stability check
    │                                  │
    │<---- x, y, w, r, valid ----------│
    │                                  │
```

The Main Unit uses a deliberately simple JSON parser because the expected response is a small flat object with known fields.

If Wi-Fi is disconnected, the Main Unit attempts to reconnect before performing an HTTP request.

If the server returns no usable object, the Main Unit simply retries later rather than treating it as a mechanical failure.

## Main Unit ↔ AUX Unit

The Main and AUX ESP32 controllers communicate over **I2C**.

- The Main Unit acts as **I2C Master**
- The AUX Unit acts as **I2C Slave** — address 0x09

### Message Format

Messages in both directions are exactly four bytes long.

#### Main → AUX

```text
Byte 0     Command
Byte 1     Payload high byte
Byte 2     Payload low byte
Byte 3     Checksum
```

The payload is a signed 16-bit value transmitted in big-endian order.

#### AUX → Main

```text
Byte 0     Status
Byte 1     Position high byte
Byte 2     Position low byte
Byte 3     Checksum
```

The returned position is the current grab motor encoder position.

### Checksum

The fourth byte is calculated using XOR over the three data bytes together with a fixed seed.

Its purpose is not security. It is a lightweight integrity check intended to catch malformed, truncated, or desynchronized I2C transfers.

If the Main Unit receives a response with the wrong length or checksum, it retries the read. If a correct response still cannot be obtained, the operation is treated as an AUX communication error.

### Typical Grip Command

```text
Main Unit                              AUX Unit
    │                                      │
    │-------- MOVE_TO + target ----------->│
    │                                      │
    │                                start motor
    │                                status = BUSY
    │                                      │
    │----------- request status ---------->│
    │<---------- BUSY + position ----------│
    │                                      │
    │           ... polling ...            │
    │                                      │
    │----------- request status ---------->│
    │<---------- IDLE + position ----------│
    │                                      │
```

The Main Unit waits for `IDLE` before continuing the pick-and-place sequence.

# Overall Operating Sequence

A normal operating cycle can be summarized as follows:

1. Power on system
2. Main Unit initializes EVShield and sensors
3. Main Unit checks AUX Unit
4. Main Unit connects to Vision Unit
5. Arm joints are homed
6. User manually opens gripper
7. AUX grip zero is established
8. Main Unit requests object data
9. Vision Unit returns stable x/y/orientation
10. Main Unit calculates motor angles
11. Arm moves above object
12. Arm lowers
13. AUX Unit closes gripper
14. Arm lifts and moves object
15. Arm lowers at drop-off position
16. AUX Unit opens gripper
17. Main arm returns to home position

# Source Layout

``` text
├───esp
│   ├───aux_unit
│   │   ├───.vscode
│   │   ├───lib
│   │   │   └───EVShield-master/     - EVShield library
│   │   └───src
│   │       ├───aux_protocol.h       - Shared Main <-> AUX I2C protocol
│   │       └───main.cpp             - AUX ESP32 firmware and grab motor control
│   └───main_unit
│       ├───lib
│       │   └───EVShield-master/     - EVShield library
│       └───src
│           ├───aux_protocol.h       - Shared Main <-> AUX I2C protocol
│           ├───kinematics.h/.cpp    - Arm geometry and inverse kinematics
│           ├───main.cpp             - Main program flow and pick-and-place sequence
│           └───rearm.h/.cpp         - Main Unit hardware control
├───pi
│   ├───config.py   - Vision calibration and detection constants
│   ├───detect.py   - Image processing, coordinate conversion, and object detection
│   └───server.py   - Raspberry Pi camera handling and HTTP interface
└───README.md       - this document
```

# Summary

R.E.A.R.M. uses a distributed architecture in which each controller has a focused responsibility.

The **Vision Unit** determines **where the object is**.

The **Main Unit** determines **how the robotic arm must move to reach it** and coordinates the full operation.

The **AUX Unit** controls **how the object is physically gripped and released**.

The Main Unit connects the three parts into one automated sequence using HTTP for vision data and I2C for low-level gripper commands.
