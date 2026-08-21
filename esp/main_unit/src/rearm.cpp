#include <Arduino.h>
#include <EVShield.h>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>

#include "rearm.h"
#include "kinematics.h"

namespace rearm {

EVShield shield;

EVs_NXTTouch touchBase;
EVs_NXTTouch touchMain;
EVs_NXTTouch touchClaw;
EVs_NXTTouch touchAux;

namespace {

/**
 * The aux ESP32 hangs off the SAME on-board I2C bus as the EVShield banks.
 * Addresses don't collide (0x09 vs 0x34 / 0x36), so a single Wire instance
 * serves both. The old code created a second software bus on GPIO 16/17
 * that was never wired up.
 */

struct MotorRef {
    EVShieldBank *bank;
    SH_Motor id;
};

MotorRef motorRef(Motor motor) {
    switch (motor) {
    case MOTOR_BASE: return {&shield.bank_a, SH_Motor_1};
    case MOTOR_MAIN: return {&shield.bank_a, SH_Motor_2};
    case MOTOR_AUX:  return {&shield.bank_b, SH_Motor_2};
    case MOTOR_CLAW: return {&shield.bank_b, SH_Motor_1};
    }
    return {&shield.bank_a, SH_Motor_1};
}

int motorSpeed(Motor motor) {
    switch (motor) {
    case MOTOR_BASE: return SPEED_BASE;
    case MOTOR_MAIN: return SPEED_MAIN;
    case MOTOR_AUX:  return SPEED_AUX;
    case MOTOR_CLAW: return SPEED_CLAW;
    }
    return SPEED_BASE;
}

const char *motorName(Motor motor) {
    switch (motor) {
    case MOTOR_BASE: return "base";
    case MOTOR_MAIN: return "main";
    case MOTOR_AUX:  return "aux";
    case MOTOR_CLAW: return "claw";
    }
    return "?";
}

/**
 * Minimal JSON field readers. The Pi sends a flat object with known keys,
 * so a full parser would be overkill and one more library to install.
 */
bool jsonBool(const String &src, const char *key, bool fallback) {
    String needle = String("\"") + key + "\"";
    int k = src.indexOf(needle);
    if (k < 0) return fallback;

    int colon = src.indexOf(':', k);
    if (colon < 0) return fallback;

    String rest = src.substring(colon + 1, min((int)src.length(), colon + 8));
    rest.trim();
    return rest.startsWith("true");
}

int jsonInt(const String &src, const char *key, int fallback) {
    String needle = String("\"") + key + "\"";
    int k = src.indexOf(needle);
    if (k < 0) return fallback;

    int colon = src.indexOf(':', k);
    if (colon < 0) return fallback;

    int i = colon + 1;
    while (i < (int)src.length() && (src[i] == ' ' || src[i] == '"')) i++;

    int start = i;
    if (i < (int)src.length() && (src[i] == '-' || src[i] == '+')) i++;
    while (i < (int)src.length() && isDigit(src[i])) i++;

    if (i == start) return fallback;
    return src.substring(start, i).toInt();
}

} // namespace

// ---------------------------------------------------------------- setup

void touchInit(Touch sensor) {
    switch (sensor) {
    case TOUCH_BASE: touchBase.init(&shield, SH_BBS2); break;
    case TOUCH_MAIN: touchMain.init(&shield, SH_BAS1); break;
    case TOUCH_AUX:  touchAux.init(&shield, SH_BAS2);  break;
    case TOUCH_CLAW: touchClaw.init(&shield, SH_BBS1); break;
    }
}

void shieldInit() {
    shield.init(SH_HardwareI2C);

    touchInit(TOUCH_BASE);
    touchInit(TOUCH_MAIN);
    touchInit(TOUCH_AUX);
    touchInit(TOUCH_CLAW);
}

// ------------------------------------------------------------- aux unit

bool auxUnitCheck() {
    Wire.beginTransmission(AUX_I2C_ADDR);
    return Wire.endTransmission() == 0;
}

bool auxUnitWaitOnline(unsigned long timeout) {
    unsigned long start = millis();
    while (millis() - start < timeout) {
        if (auxUnitCheck()) return true;
        delay(250);
    }
    return false;
}

bool auxUnitSend(uint8_t cmd, int16_t payload) {
    uint8_t hi = (uint8_t)(payload >> 8);
    uint8_t lo = (uint8_t)(payload & 0xFF);

    Wire.beginTransmission(AUX_I2C_ADDR);
    Wire.write(cmd);
    Wire.write(hi);
    Wire.write(lo);
    Wire.write(auxChecksum(cmd, hi, lo));

    return Wire.endTransmission() == 0;
}

uint8_t auxUnitStatus(int16_t *position) {
    uint8_t received = Wire.requestFrom(AUX_I2C_ADDR, AUX_MSG_LEN);
    if (received != AUX_MSG_LEN) {
        // drain anything partial so the next read starts clean
        while (Wire.available()) Wire.read();
        return AUX_ST_ERROR;
    }

    uint8_t status = Wire.read();
    uint8_t hi = Wire.read();
    uint8_t lo = Wire.read();
    uint8_t sum = Wire.read();

    if (sum != auxChecksum(status, hi, lo))
        return AUX_ST_ERROR;

    if (position)
        *position = (int16_t)(((uint16_t)hi << 8) | lo);

    return status;
}

bool auxUnitWaitIdle(unsigned long timeout) {
    unsigned long start = millis();

    // Give the slave a moment to latch the command and report BUSY, so we
    // don't read a stale IDLE from before the command landed.
    delay(100);

    while (millis() - start < timeout) {
        uint8_t status = auxUnitStatus(nullptr);

        if (status == AUX_ST_IDLE)
            return true;

        if (status == AUX_ST_UNHOMED) {
            Serial.println("ERROR: aux unit has no zero reference.");
            return false;
        }

        if (status == AUX_ST_ERROR) {
            Serial.println("ERROR: aux unit reported an error.");
            return false;
        }

        delay(50);
    }

    Serial.println("ERROR: aux unit timed out.");
    return false;
}

bool auxUnitMoveTo(int deg) {
    if (!auxUnitSend(AUX_CMD_MOVE_TO, (int16_t)deg)) {
        Serial.println("ERROR: aux MOVE_TO not acknowledged.");
        return false;
    }
    return auxUnitWaitIdle(AUX_TIMEOUT);
}

bool auxUnitMoveBy(int deg) {
    if (!auxUnitSend(AUX_CMD_MOVE_BY, (int16_t)deg)) {
        Serial.println("ERROR: aux MOVE_BY not acknowledged.");
        return false;
    }
    return auxUnitWaitIdle(AUX_TIMEOUT);
}

bool auxUnitSetZero() {
    if (!auxUnitSend(AUX_CMD_SET_ZERO, 0)) {
        Serial.println("ERROR: aux SET_ZERO not acknowledged.");
        return false;
    }
    return auxUnitWaitIdle(AUX_TIMEOUT);
}

bool auxUnitHome() {
    if (!auxUnitSend(AUX_CMD_HOME, 0)) {
        Serial.println("ERROR: aux HOME not acknowledged.");
        return false;
    }
    return auxUnitWaitIdle(AUX_TIMEOUT);
}

// ---------------------------------------------------------- vision unit

bool visionUnitConnect() {
    if (WiFi.status() == WL_CONNECTED)
        return true;

    WiFi.mode(WIFI_STA);
    WiFi.begin(PI_SSID, PI_PASSWORD);

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED) {
        if (millis() - start > WIFI_TIMEOUT) {
            Serial.println("ERROR: could not join the Pi access point.");
            return false;
        }
        delay(250);
        Serial.print(".");
    }

    Serial.println();
    Serial.print("Joined ");
    Serial.print(PI_SSID);
    Serial.print(" as ");
    Serial.println(WiFi.localIP());
    return true;
}

namespace {

/**
 * Performs a GET and returns the body, or an empty string on failure.
 */
String visionGet(const char *path) {
    if (WiFi.status() != WL_CONNECTED && !visionUnitConnect())
        return String();

    HTTPClient http;
    String url = String("http://") + PI_HOST + ":" + PI_PORT + path;

    http.setTimeout(HTTP_TIMEOUT);
    if (!http.begin(url)) {
        Serial.println("ERROR: malformed vision URL.");
        return String();
    }

    int code = http.GET();
    String body;

    if (code == 200) {
        body = http.getString();
    } else {
        Serial.print("ERROR: vision request failed, HTTP ");
        Serial.println(code);
    }

    http.end();
    return body;
}

} // namespace

bool visionUnitCheck() {
    String body = visionGet("/ping");
    return body.length() > 0 && jsonBool(body, "ok", false);
}

ObjectData visionUnitRequestData() {
    ObjectData data = {0, 0, 0, 0, 0, false};

    String body = visionGet("/object");
    if (body.length() == 0)
        return data;

    if (!jsonBool(body, "valid", false)) {
        // NOT an error: the Pi reports invalid while the scene is still
        // moving, or when the sheet is empty.
        return data;
    }

    data.x = jsonInt(body, "x", 0);
    data.y = jsonInt(body, "y", 0);
    data.w = jsonInt(body, "w", 0);
    data.h = jsonInt(body, "h", 0);
    data.r = jsonInt(body, "r", 0);
    data.valid = true;

    return data;
}

// --------------------------------------------------------------- motors

void motorStop(Motor motor, SH_Next_Action action) {
    MotorRef ref = motorRef(motor);
    ref.bank->motorStop(ref.id, action);
}

long motorPosition(Motor motor) {
    MotorRef ref = motorRef(motor);
    return ref.bank->motorGetEncoderPosition(ref.id);
}

void motorResetEncoder(Motor motor) {
    MotorRef ref = motorRef(motor);
    ref.bank->motorResetEncoder(ref.id);
}

bool motorStartMoveBy(Motor motor, int degrees) {
    if (degrees == 0)
        return true;

    MotorRef ref = motorRef(motor);
    SH_Direction dir = (degrees > 0) ? SH_Direction_Forward : SH_Direction_Reverse;

    return ref.bank->motorRunDegrees(
        ref.id,
        dir,
        motorSpeed(motor),
        (long)abs(degrees),
        SH_Completion_Dont_Wait,
        SH_Next_Action_BrakeHold);
}

bool motorStartMoveTo(Motor motor, int degrees) {
    long current = motorPosition(motor);
    return motorStartMoveBy(motor, (int)(degrees - current));
}

bool motorWait(Motor motor, unsigned long timeout) {
    MotorRef ref = motorRef(motor);
    unsigned long start = millis();

    while (!ref.bank->motorIsTachoDone(ref.id)) {
        if (millis() - start > timeout) {
            motorStop(motor, SH_Next_Action_Brake);
            Serial.print("ERROR: timeout waiting for motor ");
            Serial.println(motorName(motor));
            return false;
        }
        delay(10);
    }

    return true;
}

bool motorWaitAll(unsigned long timeout) {
    bool ok = true;
    ok &= motorWait(MOTOR_BASE, timeout);
    ok &= motorWait(MOTOR_MAIN, timeout);
    ok &= motorWait(MOTOR_AUX, timeout);
    ok &= motorWait(MOTOR_CLAW, timeout);
    return ok;
}

bool motorMoveBy(Motor motor, int degrees) {
    if (!motorStartMoveBy(motor, degrees))
        return false;
    return motorWait(motor, MOTOR_TIMEOUT);
}

bool motorMoveTo(Motor motor, int degrees) {
    if (!motorStartMoveTo(motor, degrees))
        return false;
    return motorWait(motor, MOTOR_TIMEOUT);
}

void stopAllMotors(SH_Next_Action action) {
    shield.bank_a.motorStop(SH_Motor_1, action);
    shield.bank_a.motorStop(SH_Motor_2, action);
    shield.bank_b.motorStop(SH_Motor_1, action);
    shield.bank_b.motorStop(SH_Motor_2, action);
}

bool homeAllMotors() {
    bool baseHomed = touchBase.isPressed();
    bool mainHomed = touchMain.isPressed();
    bool auxHomed  = touchAux.isPressed();
    bool clawHomed = touchClaw.isPressed();
    unsigned long startTime;

    // --- stage 1: main and aux, so the arm folds up before it swings ---

    if (!mainHomed)
        shield.bank_a.motorRunUnlimited(SH_Motor_2, HOMING_DIR_MAIN, SPEED_HOMING);

    if (!auxHomed)
        shield.bank_b.motorRunUnlimited(SH_Motor_2, HOMING_DIR_AUX, SPEED_HOMING);

    startTime = millis();
    while (!(mainHomed && auxHomed)) {
        if (!mainHomed && touchMain.isPressed()) {
            motorStop(MOTOR_MAIN, SH_Next_Action_Brake);
            motorResetEncoder(MOTOR_MAIN);
            mainHomed = true;
        }

        if (!auxHomed && touchAux.isPressed()) {
            motorStop(MOTOR_AUX, SH_Next_Action_Brake);
            motorResetEncoder(MOTOR_AUX);
            auxHomed = true;
        }

        if (millis() - startTime > HOMING_TIMEOUT) {
            stopAllMotors(SH_Next_Action_Brake);
            Serial.println("ERROR: timeout homing main/aux.");
            return false;
        }

        delay(10);
    }

    // --- stage 2: base and claw rotation ---

    if (!baseHomed)
        shield.bank_a.motorRunUnlimited(SH_Motor_1, HOMING_DIR_BASE, SPEED_HOMING);

    if (!clawHomed)
        shield.bank_b.motorRunUnlimited(SH_Motor_1, HOMING_DIR_CLAW, SPEED_HOMING);

    startTime = millis();
    while (!(baseHomed && clawHomed)) {
        if (!baseHomed && touchBase.isPressed()) {
            motorStop(MOTOR_BASE, SH_Next_Action_Brake);
            motorResetEncoder(MOTOR_BASE);
            baseHomed = true;
        }

        if (!clawHomed && touchClaw.isPressed()) {
            motorStop(MOTOR_CLAW, SH_Next_Action_Brake);
            motorResetEncoder(MOTOR_CLAW);
            clawHomed = true;
        }

        if (millis() - startTime > HOMING_TIMEOUT) {
            stopAllMotors(SH_Next_Action_Brake);
            Serial.println("ERROR: timeout homing base/claw.");
            return false;
        }

        delay(10);
    }

    stopAllMotors(SH_Next_Action_Brake);
    return true;
}

// ----------------------------------------------------------- high level

bool moveToPose(int x, int y, int z, int a, int w) {
    // The claw opening only affects the aux unit, but calculate_motor_angles
    // needs a numerically valid width, so substitute a harmless one when the
    // caller doesn't want the grip touched.
    int width = (w < 0) ? (int)CLAW_BASE_DIST : w;
    int angle = (a < 0) ? 0 : a;

    MotorAngles target = calculate_motor_angles(x, y, z, angle, width);

    if (!target.valid) {
        Serial.print("ERROR: unreachable pose x=");
        Serial.print(x); Serial.print(" y="); Serial.print(y);
        Serial.print(" z="); Serial.println(z);
        return false;
    }

    // Start the joints together, then wait for all of them. Running them in
    // parallel roughly halves the cycle time versus one move at a time.
    bool started = true;
    started &= motorStartMoveTo(MOTOR_BASE, target.base);
    started &= motorStartMoveTo(MOTOR_MAIN, target.main);
    started &= motorStartMoveTo(MOTOR_AUX,  target.aux);

    if (a >= 0)
        started &= motorStartMoveTo(MOTOR_CLAW, target.claw);

    if (!started) {
        stopAllMotors(SH_Next_Action_Brake);
        Serial.println("ERROR: a motor refused its move command.");
        return false;
    }

    bool ok = true;
    ok &= motorWait(MOTOR_BASE, MOTOR_TIMEOUT);
    ok &= motorWait(MOTOR_MAIN, MOTOR_TIMEOUT);
    ok &= motorWait(MOTOR_AUX,  MOTOR_TIMEOUT);
    if (a >= 0)
        ok &= motorWait(MOTOR_CLAW, MOTOR_TIMEOUT);

    if (!ok)
        return false;

    // The grip is on the other ESP32, so it moves after the arm is in place.
    if (w >= 0 && !auxUnitMoveTo(target.grab))
        return false;

    return true;
}

bool moveToPosition(int x_mm, int y_mm, int z_mm) {
    return moveToPose(x_mm, y_mm, z_mm, -1, -1);
}

} // namespace rearm
