#include <Arduino.h>
#include <EVShield.h>
#include <Wire.h>
#include <HTTPClient.h>

#include "rearm.h"
#include "kinematics.h"

// ---- vertical waypoints, mm above the sheet ----
#define Z_HOVER 30
#define Z_GRIP  20

// ---- drop-off pose ----
#define DROP_X 200
#define DROP_Y 65
#define DROP_ANGLE 90

// grab motor travel from fully open to gripping (in motor degrees).
// Fixed rather than computed from the object width because
// LEGO flex makes a width-derived opening unreliable
#define GRAB_CLOSE_DEG 1600

#define RETRY_DELAY 1500   // ms between vision polls

namespace {


// blocks until given string is typed in
void waitForInput(String s){
    Serial.printf("Type \"%s\" to continue\n", s);
    while (true) {
        if (Serial.available() > 0) {
            String line = Serial.readStringUntil('\n');
            if (line.equalsIgnoreCase(s))
                break;
        }
        delay(50);
    }
}

/**
 * the grab motor has no limit switch so its zero is established by hand
 */
bool establishGripZero() {
    Serial.println("\n=== CLAW SETUP ===");
    Serial.println("Open the claw FULLY by hand, then send 'ok' to continue.");

    waitForInput("ok");

    if (!rearm::auxUnitSetZero()) {
        Serial.println("ERROR: could not set the grip zero.");
        return false;
    }

    Serial.println("Grip zero set.");
    return true;
}

bool pickAndCenter(const rearm::ObjectData &obj) {
    Serial.printf("\n--- pickup: x=%d y=%d w=%d r=%d ---\n",
                  obj.x, obj.y, obj.w, obj.r);

    Serial.println("[1/8] hover");
    if (!rearm::moveToPose(obj.x, obj.y, Z_HOVER, obj.r)) return false;
    delay(500);

    Serial.println("[2/8] lower");
    if (!rearm::moveToPose(obj.x, obj.y, Z_GRIP, -1)) return false;
    delay(500);

    Serial.println("[3/8] grab");
    if (!rearm::auxUnitMoveTo(GRAB_CLOSE_DEG)) return false;
    delay(500);

    Serial.println("[4/8] lift");
    if (!rearm::moveToPose(obj.x, obj.y, Z_HOVER, -1)) return false;
    delay(500);

    Serial.println("[5/8] traverse");
    if (!rearm::moveToPose(DROP_X, DROP_Y, Z_HOVER, DROP_ANGLE)) return false;
    delay(500);

    Serial.println("[6/8] lower");
    if (!rearm::moveToPose(DROP_X, DROP_Y, Z_GRIP, -1)) return false;
   delay(500);

    Serial.println("[7/8] release");
    if (!rearm::auxUnitMoveTo(0)) return false;
    delay(500);

    Serial.println("[8/8] home");
    if (!rearm::homeAllMotors() || !rearm::auxUnitSetZero()) return false;
    
    Serial.println("--- sequence complete ---");
    return true;
}

} // namespace

void setup() {
    Serial.begin(115200);
    Wire.begin();
    delay(1000);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    Serial.println("\n=== rearm starting ===");
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

    Serial.println("\nReady. Place an object on the sheet.");
}

void loop() {
    Serial.println("Requesting object data...");
    rearm::ObjectData obj = rearm::visionUnitRequestData();

    if (!obj.valid) {
        delay(RETRY_DELAY);
        return;
    }

    if (!rearm::auxUnitSetZero()) {
        Serial.println("Aux unit decided to go on vacation mid sequence.");
        Serial.println("Halting execution. Check connection and reset the arm.");
        while(1) delay(1000);
    }
    if (!pickAndCenter(obj)) {
        Serial.println("Sequence aborted. Re-homing before the next attempt.");
        rearm::stopAllMotors(SH_Next_Action_Brake);
        delay(500);
        rearm::homeAllMotors();
    }

    waitForInput(" "); // press space to re-run the sequence
}