#pragma once

// ---- gear ratios: motor degrees per joint degree ----
#define BASE_ROT_RATIO 11.2
#define MAIN_ARM_RATIO 24.0
#define AUX_ARM_RATIO 5.0
#define CLAW_ROT_RATIO 11.67
#define CLAW_GRAB_RATIO 40

// ---- working area geometry (mm) ----
// x: 0..ZONE_W, left to right across the sheet
// y: 0..ZONE_H, ZONE_H is the edge NEAREST the arm base
// the base sits ZONE_GAP beyond the near edge
#define ZONE_W 392.0
#define ZONE_H 200.0
#define ZONE_GAP 109.0

// ---- arm geometry (mm) ----
#define BASE_ELEVATION 119.0
#define MAIN_ARM_LEN 136.0
#define MAIN_ARM_ROT 30.0
#define AUX_ARM_LEN 80.0
#define CLAW_EXTENSION 40.0
#define CLAW_HEIGHT 112.0
#define CLAW_LENGTH 48.0
#define CLAW_BASE_DIST 32.0
#define CLAW_ROT_INIT 45.0

// ---- drop-off target: center of the sheet ----
#define ZONE_CENTER_X ((int)(ZONE_W / 2))
#define ZONE_CENTER_Y ((int)(ZONE_H / 2))

typedef struct {
    int base;
    int main;
    int aux;
    int claw;   // claw rotation, main shield
    int grab;   // claw open/close, AUX SHIELD (sent over I2C)
    bool valid;
} MotorAngles;

/**
 * @brief Calculates absolute rotation angles for each motor.
 *
 * @param x Target x-coordinate in mm.
 * @param y Target y-coordinate in mm.
 * @param z Target z-coordinate in mm (0 = sheet surface).
 * @param a Target rotation angle of the claw in degrees.
 * @param w Opening width of the claw in mm.
 *
 * @return MotorAngles containing the final target rotation for each motor.
 *         `.valid` is false if the pose is unreachable or numerically invalid.
 */
MotorAngles calculate_motor_angles(int x, int y, int z, int a, int w);
