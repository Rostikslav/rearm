/**
 * AUX UNIT - secondary ESP32 + EVShield, drives the claw grab motor only.
 *
 * WIRING
 * ------
 * This ESP32 has to be an I2C *master* toward its own EVShield and an I2C
 * *slave* toward the main ESP32. One peripheral cannot do both, so it uses
 * both buses:
 *
 *   Wire  (GPIO 21 / 22, default) -> this unit's EVShield   [master]
 *   Wire1 (GPIO 16 / 17)          -> main ESP32             [slave]
 *
 * On the main ESP32 side everything hangs off its single Wire bus:
 *
 *   MAIN GPIO21 (SDA) --+-- main EVShield SDA
 *                       +-- AUX GPIO16 (SDA)
 *   MAIN GPIO22 (SCL) --+-- main EVShield SCL
 *                       +-- AUX GPIO17 (SCL)
 *
 * Common ground between both ESP32s is required.
 */

#include <Arduino.h>
#include <Wire.h>
#include <EVShield.h>

#include "aux_protocol.h"

// ---- link to the main ESP32 ----
#define LINK_SDA 16
#define LINK_SCL 17

// ---- which port the grab motor is plugged into ----
#define GRAB_BANK  shield.bank_b
#define GRAB_MOTOR SH_Motor_1

#define GRAB_SPEED 30
#define GRAB_TIMEOUT 8000   // ms for a single move
#define GRAB_TOLERANCE 5    // motor degrees considered "arrived"
#define GRAB_STALL_MS 400   // no movement for this long = stalled/arrived

EVShield shield;

// ---- state shared with the interrupt handlers ----
volatile uint8_t  pendingCmd     = 0;
volatile int16_t  pendingPayload = 0;
volatile bool     hasPending     = false;
volatile bool     badFrame       = false;

// ---- state owned by loop() ----
volatile uint8_t  status   = AUX_ST_UNHOMED;
volatile int16_t  position = 0;     // cached, so onRequest never touches I2C

bool moving = false;
unsigned long moveStarted = 0;
int16_t moveTarget = 0;
int16_t lastPosition = 0;
unsigned long lastMovement = 0;

/**
 * Interrupt context. Latches the frame and returns immediately.
 */
void onLinkReceive(int count) {
    if (count != AUX_MSG_LEN) {
        while (Wire1.available()) Wire1.read();
        badFrame = true;
        return;
    }

    uint8_t cmd = Wire1.read();
    uint8_t hi  = Wire1.read();
    uint8_t lo  = Wire1.read();
    uint8_t sum = Wire1.read();

    if (sum != auxChecksum(cmd, hi, lo)) {
        badFrame = true;
        return;
    }

    // STOP is the one command that may pre-empt a move in progress.
    if (!hasPending || cmd == AUX_CMD_STOP) {
        pendingCmd     = cmd;
        pendingPayload = (int16_t)(((uint16_t)hi << 8) | lo);
        hasPending     = true;
    }
}

void refreshLinkBuffer() {
    uint8_t st = status;
    uint8_t hi = (uint8_t)(position >> 8);
    uint8_t lo = (uint8_t)(position & 0xFF);

    uint8_t buf[AUX_MSG_LEN] = {st, hi, lo, auxChecksum(st, hi, lo)};
    Wire1.slaveWrite(buf, AUX_MSG_LEN);
}

/**
 * Interrupt context. Serves cached values only.
 * Kept as a fallback for cores where slaveWrite() is not used.
 */
void onLinkRequest() {
    uint8_t st = status;
    uint8_t hi = (uint8_t)(position >> 8);
    uint8_t lo = (uint8_t)(position & 0xFF);

    Wire1.write(st);
    Wire1.write(hi);
    Wire1.write(lo);
    Wire1.write(auxChecksum(st, hi, lo));
}

void startMoveBy(int delta) {
    if (delta == 0) {
        status = AUX_ST_IDLE;
        return;
    }

    SH_Direction dir = (delta > 0) ? SH_Direction_Forward : SH_Direction_Reverse;

    moveTarget = (int16_t)(position + delta);

    GRAB_BANK.motorRunDegrees(
        GRAB_MOTOR,
        dir,
        GRAB_SPEED,
        (long)abs(delta),
        SH_Completion_Dont_Wait,
        SH_Next_Action_BrakeHold);

    moving = true;
    moveStarted = millis();
    lastPosition = position;
    lastMovement = moveStarted;
    status = AUX_ST_BUSY;
}

void handleCommand(uint8_t cmd, int16_t payload) {
    switch (cmd) {

    case AUX_CMD_PING:
        // nothing to do; status is served by onLinkRequest
        break;

    case AUX_CMD_SET_ZERO:
        GRAB_BANK.motorStop(GRAB_MOTOR, SH_Next_Action_Brake);
        GRAB_BANK.motorResetEncoder(GRAB_MOTOR);
        position = 0;
        moving = false;
        status = AUX_ST_IDLE;
        Serial.println("zero set");
        break;

    case AUX_CMD_STOP:
        GRAB_BANK.motorStop(GRAB_MOTOR, SH_Next_Action_Brake);
        moving = false;
        status = (status == AUX_ST_UNHOMED) ? AUX_ST_UNHOMED : AUX_ST_IDLE;
        Serial.println("stopped");
        break;

    case AUX_CMD_MOVE_TO:
        if (status == AUX_ST_UNHOMED) {
            Serial.println("refused MOVE_TO: no zero reference");
            break;
        }
        Serial.print("move to "); Serial.println(payload);
        startMoveBy((int)payload - (int)position);
        break;

    case AUX_CMD_MOVE_BY:
        if (status == AUX_ST_UNHOMED) {
            Serial.println("refused MOVE_BY: no zero reference");
            break;
        }
        Serial.print("move by "); Serial.println(payload);
        startMoveBy((int)payload);
        break;

    case AUX_CMD_HOME:
        if (status == AUX_ST_UNHOMED) {
            Serial.println("refused HOME: no zero reference");
            break;
        }
        Serial.println("homing grip");
        startMoveBy(-(int)position);
        break;

    default:
        Serial.print("unknown command 0x");
        Serial.println(cmd, HEX);
        status = AUX_ST_ERROR;
        break;
    }
}

void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println();
    Serial.println("=== aux unit starting ===");

    // master toward the local EVShield
    shield.init(SH_HardwareI2C);
    GRAB_BANK.motorStop(GRAB_MOTOR, SH_Next_Action_Brake);

    // slave toward the main ESP32
    Wire1.begin((uint8_t)AUX_I2C_ADDR, LINK_SDA, LINK_SCL, 100000);
    Wire1.onReceive(onLinkReceive);
    Wire1.onRequest(onLinkRequest);

    status = AUX_ST_UNHOMED;
    refreshLinkBuffer();

    Serial.print("listening on 0x");
    Serial.println(AUX_I2C_ADDR, HEX);
    Serial.println("waiting for SET_ZERO from the main unit");
}

void loop() {
    // --- report a bad frame once ---
    if (badFrame) {
        badFrame = false;
        Serial.println("bad frame received");
        if (!moving && status != AUX_ST_UNHOMED)
            status = AUX_ST_ERROR;
    }

    // --- pick up a latched command ---
    if (hasPending) {
        uint8_t cmd;
        int16_t payload;

        noInterrupts();
        cmd = pendingCmd;
        payload = pendingPayload;
        hasPending = false;
        interrupts();

        // Clear a previous error now that a fresh command has arrived.
        if (status == AUX_ST_ERROR)
            status = AUX_ST_IDLE;

        handleCommand(cmd, payload);

        refreshLinkBuffer();
    }

    // --- track an in-flight move ---
    if (moving) {
        position = (int16_t)GRAB_BANK.motorGetEncoderPosition(GRAB_MOTOR);

        unsigned long now = millis();

        if (abs((int)position - (int)lastPosition) > 2) {
            lastPosition = position;
            lastMovement = now;
        }

        bool arrived = abs((int)position - (int)moveTarget) <= GRAB_TOLERANCE;
        bool stalled = (now - lastMovement) > GRAB_STALL_MS;

        if (arrived || stalled) {
            // a stall is normal here since the claw closing onto an object
            // stops early and that counts as a successful grip.
            GRAB_BANK.motorStop(GRAB_MOTOR, SH_Next_Action_BrakeHold);
            moving = false;
            status = AUX_ST_IDLE;

            Serial.print(arrived ? "arrived at " : "stalled at ");
            Serial.print(position);
            Serial.print(" (target ");
            Serial.print(moveTarget);
            Serial.println(")");

        } else if (now - moveStarted > GRAB_TIMEOUT) {
            GRAB_BANK.motorStop(GRAB_MOTOR, SH_Next_Action_Brake);
            moving = false;
            status = AUX_ST_ERROR;
            Serial.println("move timed out");
        }
    }

    // Keep the staged reply current so any incoming read is served instantly.
    refreshLinkBuffer();

    delay(5);
}