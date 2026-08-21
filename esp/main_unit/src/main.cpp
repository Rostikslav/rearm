#include <Arduino.h>
#include <EVShield.h>
#include <Wire.h>

#include "rearm.h"
#include "kinematics.h"

// ---- vertical waypoints, mm above the sheet ----
#define Z_APPROACH 60   // hover here before descending onto the object
#define Z_GRIP      0   // claw tips at the sheet surface
#define Z_LIFT     60   // carry height while the object is held

// ---- grip tuning, mm ----
#define GRIP_CLEARANCE 12   // how much wider than the object the claw opens
#define GRIP_SQUEEZE    3   // how far past the object width the claw closes

#define CLAW_ANGLE_OFFSET 90

#define RETRY_DELAY 1500   // ms between vision polls

namespace {

int gripWidth(const rearm::ObjectData &obj) {
    return min(obj.w, obj.h);
}

int gripAngle(const rearm::ObjectData &obj) {
    int a = (obj.r + CLAW_ANGLE_OFFSET) % 180;
    if (a < 0) a += 180;
    return a;
}

/**
 * The grab motor has no limit switch, so its zero has to be established by
 * hand at startup. 
 */
bool establishGripZero() {
    Serial.println();
    Serial.println("=== CLAW SETUP ===");
    Serial.println("Open the claw FULLY by hand, then send 'ok' to continue.");

    while (true) {
        if (Serial.available() > 0) {
            String line = Serial.readStringUntil('\n');
            line.trim();
            if (line.equalsIgnoreCase("ok"))
                break;
            Serial.println("Send 'ok' once the claw is fully open.");
        }
        delay(50);
    }

    if (!rearm::auxUnitSetZero()) {
        Serial.println("ERROR: could not set the grip zero.");
        return false;
    }

    Serial.println("Grip zero set.");
    return true;
}

bool pickAndCenter(const rearm::ObjectData &obj) {
    int width = gripWidth(obj);
    int angle = gripAngle(obj);

    Serial.println();
    Serial.println("--- pickup sequence ---");
    Serial.print("object x="); Serial.print(obj.x);
    Serial.print(" y=");       Serial.print(obj.y);
    Serial.print(" w=");       Serial.print(obj.w);
    Serial.print(" h=");       Serial.print(obj.h);
    Serial.print(" r=");       Serial.println(obj.r);
    Serial.print("grip width="); Serial.print(width);
    Serial.print(" angle=");     Serial.println(angle);

    // 1. hover above the object, claw rotated and opened wide
    Serial.println("[1/8] approach");
    if (!rearm::moveToPose(obj.x, obj.y, Z_APPROACH, angle, width + GRIP_CLEARANCE))
        return false;

    // 2. descend onto it
    Serial.println("[2/8] descend");
    if (!rearm::moveToPose(obj.x, obj.y, Z_GRIP, angle, -1))
        return false;

    // 3. close the claw
    Serial.println("[3/8] grip");
    if (!rearm::moveToPose(obj.x, obj.y, Z_GRIP, angle, width - GRIP_SQUEEZE))
        return false;

    // 4. lift
    Serial.println("[4/8] lift");
    if (!rearm::moveToPose(obj.x, obj.y, Z_LIFT, angle, -1))
        return false;

    // 5. swing to the centre of the sheet, still holding
    Serial.println("[5/8] traverse to centre");
    if (!rearm::moveToPose(ZONE_CENTER_X, ZONE_CENTER_Y, Z_LIFT, angle, -1))
        return false;

    // 6. lower
    Serial.println("[6/8] lower");
    if (!rearm::moveToPose(ZONE_CENTER_X, ZONE_CENTER_Y, Z_GRIP, angle, -1))
        return false;

    // 7. release
    Serial.println("[7/8] release");
    if (!rearm::moveToPose(ZONE_CENTER_X, ZONE_CENTER_Y, Z_GRIP, angle,
                           width + GRIP_CLEARANCE))
        return false;

    // 8. retreat and go home, clearing the camera's view
    Serial.println("[8/8] retreat");
    if (!rearm::moveToPose(ZONE_CENTER_X, ZONE_CENTER_Y, Z_LIFT, angle, -1))
        return false;

    if (!rearm::homeAllMotors())
        return false;

    if (!rearm::auxUnitHome())
        return false;

    Serial.println("--- sequence complete ---");
    return true;
}

} // namespace

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("=== rearm starting ===");

    Wire.begin();
    rearm::shieldInit();

    Serial.println("Waiting for the aux unit...");
    if (!rearm::auxUnitWaitOnline(30000)) {
        Serial.println("ERROR: aux unit never came online. Halting.");
        while (1) delay(1000);
    }
    Serial.println("Aux unit online.");

    Serial.println("Connecting to the vision unit...");
    if (!rearm::visionUnitConnect() || !rearm::visionUnitCheck()) {
        Serial.println("WARNING: vision unit unreachable, will retry in the loop.");
    } else {
        Serial.println("Vision unit online.");
    }

    Serial.println("Homing motors...");
    rearm::stopAllMotors(SH_Next_Action_Brake);
    delay(1000);

    if (!rearm::homeAllMotors()) {
        Serial.println("ERROR: homing failed. Halting.");
        while (1) delay(1000);
    }
    Serial.println("All motors homed.");

    if (!establishGripZero()) {
        Serial.println("ERROR: grip setup failed. Halting.");
        while (1) delay(1000);
    }

    Serial.println();
    Serial.println("Ready. Place an object on the sheet.");
}

void loop() {
    rearm::ObjectData obj = rearm::visionUnitRequestData();

    if (!obj.valid) {
        // Either nothing on the sheet, or the scene is still moving and the
        // Pi is holding back the result. Both cases just mean "wait".
        Serial.printf("No valid object detected, trying again in %f seconds", RETRY_DELAY/1000.0f);
        delay(RETRY_DELAY);
        return;
    }

    if (!pickAndCenter(obj)) {
        Serial.println("Sequence aborted. Re-homing before the next attempt.");
        rearm::stopAllMotors(SH_Next_Action_Brake);
        delay(500);
        rearm::homeAllMotors();
    }

    // Let the operator move the object again before the next cycle.
    delay(3000);
}
