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

// ---- motor metadata

MotorRef motorRef(Motor motor) {
    switch (motor) {
    case MOTOR_BASE: return {&shield.bank_a, SH_Motor_1};
    case MOTOR_MAIN: return {&shield.bank_a, SH_Motor_2};
    case MOTOR_AUX:  return {&shield.bank_b, SH_Motor_2};
    case MOTOR_CLAW: return {&shield.bank_b, SH_Motor_1};
    }
    return {&shield.bank_a, SH_Motor_1};
}

int motorSign(Motor motor) {
    SH_Direction homingDir;

    switch (motor) {
    case MOTOR_BASE: homingDir = BASE_HOMING_DIR; break;
    case MOTOR_MAIN: homingDir = MAIN_HOMING_DIR; break;
    case MOTOR_AUX:  homingDir = AUX_HOMING_DIR;  break;
    case MOTOR_CLAW: homingDir = CLAW_HOMING_DIR; break;
    default:         homingDir = SH_Direction_Forward; break;
    }

    return (homingDir == SH_Direction_Forward) ? -1 : 1;
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

int motorCorrection(Motor motor) {
    switch (motor) {
    case MOTOR_BASE: return BASE_CORRECTION_DEG;
    case MOTOR_MAIN: return MAIN_CORRECTION_DEG;
    case MOTOR_AUX:  return AUX_CORRECTION_DEG;
    case MOTOR_CLAW: return CLAW_CORRECTION_DEG;
    }
    return 0;
}

namespace {

// direction of the last move for each joint: +1, -1, or 0 for "empty"
int lastDirection[4] = {0, 0, 0, 0};

// total compensation currently baked into each motor encoder
long backlashOffset[4] = {0, 0, 0, 0};

} // namespace

void motorResetBacklash(Motor motor) {
    lastDirection[motor] = 0;
    backlashOffset[motor] = 0;
}

namespace {

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

// ---- setup

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

// ---- aux unit

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
    for (int attempt = 0; attempt < 2; attempt++) {
        if (attempt > 0)
            delay(10);

        uint8_t received = Wire.requestFrom(AUX_I2C_ADDR, AUX_MSG_LEN);
        if (received != AUX_MSG_LEN) {
            while (Wire.available()) Wire.read();
            continue;
        }

        uint8_t status = Wire.read();
        uint8_t hi = Wire.read();
        uint8_t lo = Wire.read();
        uint8_t sum = Wire.read();

        if (sum != auxChecksum(status, hi, lo))
            continue;

        if (position)
            *position = (int16_t)(((uint16_t)hi << 8) | lo);

        return status;
    }

    return AUX_ST_ERROR;
}

bool auxUnitWaitIdle(unsigned long timeout) {
    unsigned long start = millis();

    delay(30);

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

        delay(20);
    }

    Serial.println("ERROR: aux unit timed out.");
    return false;
}

bool auxUnitMoveTo(int deg) {
    if (!auxUnitSend(AUX_CMD_MOVE_TO, (int16_t)deg)) {
        Serial.println("ERROR: aux MOVE_TO not acknowledged.");
        return false;
    }
    return auxUnitWaitIdle(AUX_UNIT_TIMEOUT);
}

bool auxUnitMoveBy(int deg) {
    if (!auxUnitSend(AUX_CMD_MOVE_BY, (int16_t)deg)) {
        Serial.println("ERROR: aux MOVE_BY not acknowledged.");
        return false;
    }
    return auxUnitWaitIdle(AUX_UNIT_TIMEOUT);
}

bool auxUnitSetZero() {
    if (!auxUnitSend(AUX_CMD_SET_ZERO, 0)) {
        Serial.println("ERROR: aux SET_ZERO not acknowledged.");
        return false;
    }
    return auxUnitWaitIdle(AUX_UNIT_TIMEOUT);
}

bool auxUnitHome() {
    if (!auxUnitSend(AUX_CMD_HOME, 0)) {
        Serial.println("ERROR: aux HOME not acknowledged.");
        return false;
    }
    return auxUnitWaitIdle(AUX_UNIT_TIMEOUT);
}

// ---- vision unit

bool visionUnitConnect() {
    if (WiFi.status() == WL_CONNECTED)
        return true;
    
    #if 0 // only for debugging in case of connection failure
        int n = WiFi.scanNetworks();
        Serial.printf("%d networks found:\n", n);
        for (int i = 0; i < n; i++) {
            Serial.printf("  %-24s ch%-3d %4d dBm  enc=%d\n",
                WiFi.SSID(i).c_str(), WiFi.channel(i), WiFi.RSSI(i), WiFi.encryptionType(i));
        }
    #endif

    WiFi.mode(WIFI_STA);
    WiFi.begin(PI_SSID, PI_PASSWORD);

    Serial.print("Connecting to the Pi access point");
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED) {
        if (millis() - start > WIFI_TIMEOUT) {
            Serial.println("\nERROR: could not join the Pi access point.");
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
    ObjectData data = {0, 0, 0, 0, false};

    String body = visionGet("/object");
    if (body.length() == 0)
        return data;

    // NOT an error: the Pi reports invalid while the scene is still moving or when the sheet is empty.
    if (!jsonBool(body, "valid", false))
        return data;

    data.x = jsonInt(body, "x", 0);
    data.y = jsonInt(body, "y", 0);
    data.w = jsonInt(body, "w", 0);
    data.r = jsonInt(body, "r", 0);
    data.valid = true;

    return data;
}

// ---- motors

void motorStop(Motor motor, SH_Next_Action action) {
    MotorRef ref = motorRef(motor);
    ref.bank->motorStop(ref.id, action);
}

uint8_t motorStatus(Motor motor) {
    MotorRef ref = motorRef(motor);
    return ref.bank->motorGetStatusByte(ref.id);
}

long motorPosition(Motor motor) {
    MotorRef ref = motorRef(motor);
    // converted to joint space (positive means away from the home switch)
    long raw = (long)ref.bank->motorGetEncoderPosition(ref.id) * motorSign(motor);
    return raw - backlashOffset[motor];
}

void motorResetEncoder(Motor motor) {
    MotorRef ref = motorRef(motor);
    ref.bank->motorResetEncoder(ref.id);
    motorResetBacklash(motor);
}

bool motorStartMoveBy(Motor motor, int degrees) {
    if (degrees == 0)
        return true;

    MotorRef ref = motorRef(motor);

    int direction = (degrees > 0) ? 1 : -1;

    // Reversing direction or moving for the first time after homing
    // means the gear train has slack to take up before the joint responds.
    // Drive the extra degrees and remember them so that motorPosition() stays
    // honest about where the joint actually is
    if (direction != lastDirection[motor]) {
        long correction = (long)motorCorrection(motor) * direction;
        degrees += correction;
        backlashOffset[motor] += correction;
    }
    lastDirection[motor] = direction;

    // convert the joint-space delta into a raw motor direction
    long raw = (long)degrees * motorSign(motor);
    SH_Direction dir = (raw > 0) ? SH_Direction_Forward : SH_Direction_Reverse;

    ref.bank->motorRunDegrees(
        ref.id,
        dir,
        motorSpeed(motor),
        labs(raw),
        SH_Completion_Dont_Wait,
        SH_Next_Action_BrakeHold);

    return true;
}

bool motorStartMoveTo(Motor motor, int degrees) {
    long current = motorPosition(motor);
    return motorStartMoveBy(motor, (int)(degrees - current));
}

bool motorWait(Motor motor, unsigned long timeout) {
    unsigned long start = millis();
    bool started = false;

    // Wait for TACHO to appear first, otherwise we would read
    // the pre-command status and conclude the move was already finished.
    while (millis() - start < MOTOR_START_TIMEOUT) {
        if (motorStatus(motor) & SH_STATUS_TACHO) {
            started = true;
            break;
        }
        delay(10);
    }

    if (!started) {
        // either the move was too short to observe or the shield never took
        // the command. anyways treat it as done rather than hanging
        return true;
    }

    // wait for TACHO to clear
    while (millis() - start < timeout) {
        if ((motorStatus(motor) & SH_STATUS_TACHO) == 0)
            return true;    // move complete

        delay(10);
    }

    motorStop(motor, SH_Next_Action_Brake);
    Serial.print("ERROR: timeout waiting for motor ");
    Serial.print(motorName(motor));
    Serial.print(" at ");
    Serial.println(motorPosition(motor));
    return false;
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
        shield.bank_a.motorRunUnlimited(SH_Motor_2, MAIN_HOMING_DIR, MAIN_HOMING_SPEED);

    if (!auxHomed)
        shield.bank_b.motorRunUnlimited(SH_Motor_2, AUX_HOMING_DIR, AUX_HOMING_SPEED);

    startTime = millis();
    while (!(mainHomed && auxHomed)) {
        if (!mainHomed && touchMain.isPressed()) {
            motorStop(MOTOR_MAIN, SH_Next_Action_Brake);
            delay(100);
            motorResetEncoder(MOTOR_MAIN);
            mainHomed = true;
        }

        if (!auxHomed && touchAux.isPressed()) {
            motorStop(MOTOR_AUX, SH_Next_Action_Brake);
            delay(100);
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
        shield.bank_a.motorRunUnlimited(SH_Motor_1, BASE_HOMING_DIR, BASE_HOMING_SPEED);

    if (!clawHomed)
        shield.bank_b.motorRunUnlimited(SH_Motor_1, CLAW_HOMING_DIR, CLAW_HOMING_SPEED);

    startTime = millis();
    while (!(baseHomed && clawHomed)) {
        if (!baseHomed && touchBase.isPressed()) {
            motorStop(MOTOR_BASE, SH_Next_Action_Brake);
            delay(100);
            motorResetEncoder(MOTOR_BASE);
            baseHomed = true;
        }

        if (!clawHomed && touchClaw.isPressed()) {
            motorStop(MOTOR_CLAW, SH_Next_Action_Brake);
            delay(100);
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

// ---- high level

bool moveToPose(int x, int y, int z, int a) {
    MotorAngles target = calculate_motor_angles(x, y, z, (a < 0) ? 0 : a);

    if (!target.valid) {
        Serial.printf("ERROR: unreachable pose (x=%d, y=%d, z=%d)\n", x, y, z);
        return false;
    }

    Serial.printf("Moving to (x=%d, y=%d, z=%d): base=%d, main=%d, aux=%d, claw=%d\n",
        x, y, z,
        (int)(target.base / BASE_ROT_RATIO),
        (int)(target.main / MAIN_ROT_RATIO),
        (int)(target.aux / AUX_ROT_RATIO),
        (int)(target.claw / CLAW_ROT_RATIO));

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

    return ok;
}

bool moveToPosition(int x_mm, int y_mm, int z_mm) {
    return moveToPose(x_mm, y_mm, z_mm, -1);
}

} // namespace rearm
