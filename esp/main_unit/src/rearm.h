#pragma once

#include <EVShield.h>
#include <EVs_NXTTouch.h>

#include "aux_protocol.h"

// ---- motor speeds ----
#define SPEED_BASE 30
#define SPEED_MAIN 30
#define SPEED_AUX  30
#define SPEED_CLAW 30

// homing is done against physical end stops - keep it slow
#define SPEED_HOMING 15

#define HOMING_TIMEOUT 30000   // ms, whole homing sequence
#define MOTOR_TIMEOUT  15000   // ms, single motor move
#define AUX_TIMEOUT    10000   // ms, aux unit move

// ---- WiFi link to the Raspberry Pi ----
// The Pi runs the access point; the ESP32 is a client
#define PI_SSID     "rearm"
#define PI_PASSWORD "rearm1234"
#define PI_HOST     "192.168.4.1"
#define PI_PORT     8000
#define WIFI_TIMEOUT 20000     // ms to join the AP
#define HTTP_TIMEOUT 8000      // ms per request

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
 * Object as reported by the vision unit, already converted to millimetres
 * in the working-area frame (x: 0..ZONE_W, y: 0..ZONE_H).
 */
struct ObjectData {
    int x;      // center, mm
    int y;      // center, mm
    int w;      // long side, mm
    int h;      // short side, mm
    int r;      // orientation, degrees 0..179
    bool valid;
};

extern EVShield shield;

extern EVs_NXTTouch touchBase;
extern EVs_NXTTouch touchMain;
extern EVs_NXTTouch touchClaw;
extern EVs_NXTTouch touchAux;

// Directions used during homing.
const SH_Direction HOMING_DIR_BASE = SH_Direction_Forward;
const SH_Direction HOMING_DIR_MAIN = SH_Direction_Forward;
const SH_Direction HOMING_DIR_CLAW = SH_Direction_Reverse;
const SH_Direction HOMING_DIR_AUX  = SH_Direction_Reverse;

// ---------------------------------------------------------------- setup

/**
 * @brief Initializes the EVShield and the touch sensors.
 */
void shieldInit();

/**
 * @brief Initializes a touch sensor.
 */
void touchInit(Touch sensor);

// ------------------------------------------------------------- aux unit

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

// ---------------------------------------------------------- vision unit

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
 * The Pi only reports an object once it has confirmed the object is not
 * moving, so a result with valid == false usually just means "not settled
 * yet" and the caller should retry rather than abort.
 */
ObjectData visionUnitRequestData();

// --------------------------------------------------------------- motors

/**
 * @brief Stops the given motor.
 */
void motorStop(Motor motor, SH_Next_Action action);

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
 * @brief Reads the current encoder position of the given motor.
 */
long motorPosition(Motor motor);

/**
 * @brief Resets the internal position to 0.
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
 * @param x,y,z  Target position in mm (z = 0 is the sheet surface).
 * @param a      Claw rotation in degrees, or -1 to leave it where it is.
 * @param w      Claw opening in mm, or -1 to leave it where it is.
 */
bool moveToPose(int x, int y, int z, int a, int w);

/**
 * @brief Move without touching claw rotation or grip.
 */
bool moveToPosition(int x_mm, int y_mm, int z_mm);

} // namespace rearm
