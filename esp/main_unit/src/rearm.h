#pragma once

#include <EVShield.h>
#include <EVs_NXTTouch.h>

#include "aux_protocol.h"

// ---- motor speeds (0..100) ----
#define SPEED_BASE 30
#define SPEED_MAIN 30
#define SPEED_AUX  15
#define SPEED_CLAW 80

// homing is done against physical end stops
#define BASE_HOMING_SPEED 30
#define MAIN_HOMING_SPEED 20
#define AUX_HOMING_SPEED 15
#define CLAW_HOMING_SPEED 80

// angle correction values when changing movement direction
#define BASE_CORRECTION_DEG 100
#define MAIN_CORRECTION_DEG 100
#define AUX_CORRECTION_DEG -100
#define CLAW_CORRECTION_DEG 300

#define HOMING_TIMEOUT   30000    // ms, whole homing sequence
#define MOTOR_TIMEOUT    15000    // ms, single motor move
#define AUX_UNIT_TIMEOUT 10000    // ms, aux unit move

// ms to wait for the shield to report that a move actually started
#define MOTOR_START_TIMEOUT 300

// ---- WiFi link to the Raspberry Pi ----
// Pi runs the access point, ESP is its client
#define PI_SSID     "rearm"
#define PI_PASSWORD "rearm1234"
#define PI_HOST     "10.42.0.1"
#define PI_PORT     8000
#define WIFI_TIMEOUT 60000 // ms to join the AP
#define HTTP_TIMEOUT 8000  // ms per request

namespace rearm {

typedef enum {
    MOTOR_BASE,
    MOTOR_MAIN,
    MOTOR_AUX,
    MOTOR_CLAW,
} Motor;

typedef enum {
    TOUCH_BASE,
    TOUCH_MAIN,
    TOUCH_AUX,
    TOUCH_CLAW,
} Touch;

/**
 * Which bank and which port a given motor lives on.
 */
struct MotorRef {
    EVShieldBank *bank;
    SH_Motor id;
};

/**
 * Object reported by the vision unit.
 */
struct ObjectData {
    int x;      // center, mm
    int y;      // center, mm
    int w;      // short side to grip on, mm
    int r;      // orientation, degrees 0..179
    bool valid;
};

extern EVShield shield;

extern EVs_NXTTouch touchBase;
extern EVs_NXTTouch touchMain;
extern EVs_NXTTouch touchClaw;
extern EVs_NXTTouch touchAux;

// Directions used during homing.
const SH_Direction BASE_HOMING_DIR = SH_Direction_Forward;
const SH_Direction MAIN_HOMING_DIR = SH_Direction_Forward;
const SH_Direction AUX_HOMING_DIR  = SH_Direction_Reverse;
const SH_Direction CLAW_HOMING_DIR = SH_Direction_Reverse;

// ---- setup ----

/**
 * @brief Initializes the EVShield and the touch sensors.
 */
void shieldInit();

/**
 * @brief Initializes a touch sensor.
 */
void touchInit(Touch sensor);

// ---- motor metadata ----

/**
 * @brief Returns the bank and port for the given motor.
 */
MotorRef motorRef(Motor motor);

/**
 * @brief Returns +1 or -1, converting between joint space and raw motor space.
 *
 * Positive joint angles move AWAY from the home switch so the driving
 * direction is the opposite of that motor's homing direction.
 */
int motorSign(Motor motor);

/**
 * @brief Returns the configured speed for the given motor (0..100).
 */
int motorSpeed(Motor motor);

/**
 * @brief Returns a short human-readable name, for log messages.
 */
const char *motorName(Motor motor);

// ---- aux unit ----

/**
 * @brief Checks whether the aux ESP32 answers on the I2C bus.
 */
bool auxUnitCheck();

/**
 * @brief Blocks until the aux unit answers, or the timeout expires.
 */
bool auxUnitWaitOnline(unsigned long timeout);

/**
 * @brief Reads the aux unit status byte (AUX_ST_*).
 * @param position Optional out-param, current grab motor position.
 * @return AUX_ST_ERROR if the read failed.
 */
uint8_t auxUnitStatus(int16_t *position = nullptr);

/**
 * @brief Sends a raw command frame to the aux unit.
 * @return true if the frame was acknowledged on the bus.
 */
bool auxUnitSend(uint8_t cmd, int16_t payload);

/**
 * @brief Blocks until the aux unit reports IDLE.
 * @return false on timeout or error status.
 */
bool auxUnitWaitIdle(unsigned long timeout);

/**
 * @brief Moves the grab motor to an absolute position and waits.
 *
 * The grab motor has no limit switch, so "absolute" is relative to the
 * zero declared at startup. See auxUnitSetZero().
 */
bool auxUnitMoveTo(int deg);

/**
 * @brief Moves the grab motor by a relative amount and waits.
 */
bool auxUnitMoveBy(int deg);

/**
 * @brief Declares the current grab motor position as zero (claw fully open).
 */
bool auxUnitSetZero();

/**
 * @brief Returns the grab motor to its remembered zero.
 */
bool auxUnitHome();

// ---- vision unit ----

/**
 * @brief Joins the Pi's access point.
 */
bool visionUnitConnect();

/**
 * @brief Checks whether the vision service answers.
 */
bool visionUnitCheck();

/**
 * @brief Requests one object from the Pi.
 *
 * valid == false means either "object is not settled" or "no object detected", retry rather than abort
 */
ObjectData visionUnitRequestData();

// ---- motors ----

/**
 * @brief Stops the given motor.
 */
void motorStop(Motor motor, SH_Next_Action action);

/**
 * @brief Reads the shield status byte for the given motor.
 *
 * Useful bits: SH_STATUS_TACHO (encoder move in progress),
 * SH_STATUS_STALL, SH_STATUS_MOVING, SH_STATUS_OVERLOAD.
 */
uint8_t motorStatus(Motor motor);

/**
 * @brief Reads the encoder position (in joint space).
 *
 * Positive means away from the home switch.
 */
long motorPosition(Motor motor);

/**
 * @brief Starts a relative move; does not wait for completion.
 */
bool motorStartMoveBy(Motor motor, int degrees);

/**
 * @brief Starts an absolute move; does not wait for completion.
 */
bool motorStartMoveTo(Motor motor, int degrees);

/**
 * @brief Blocks until the given motor finishes, or the timeout expires.
 * @return false on stall or timeout.
 */
bool motorWait(Motor motor, unsigned long timeout);

/**
 * @brief Blocks until all four motors finish, or the timeout expires.
 */
bool motorWaitAll(unsigned long timeout);

/**
 * @brief Rotates the given motor by the given number of degrees and waits.
 */
bool motorMoveBy(Motor motor, int degrees);

/**
 * @brief Rotates the given motor to the given absolute angle and waits.
 */
bool motorMoveTo(Motor motor, int degrees);

/**
 * @brief Resets the encoder to 0 at the current position.
 */
void motorResetEncoder(Motor motor);

/**
 * @brief Stops all four motors.
 */
void stopAllMotors(SH_Next_Action action);

/**
 * @brief Executes a homing sequence for all motors.
 */
bool homeAllMotors();

// ----------------------------------------------------------- high level

/**
 * @brief Moves the arm to a pose. Blocks until every joint has arrived.
 *
 * The grip is controlled separately via auxUnit...()
 *
 * @param x,y,z Target position in mm (z = 0 is the sheet surface).
 * @param a     Claw rotation in degrees, or -1 to leave it where it is.
 */
bool moveToPose(int x, int y, int z, int a);

/**
 * @brief Move without touching claw rotation.
 */
bool moveToPosition(int x_mm, int y_mm, int z_mm);

} // namespace rearm