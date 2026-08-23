#pragma once

// ---- gear ratios: motor degrees per joint degree ----
#define BASE_ROT_RATIO 11.2
#define MAIN_ROT_RATIO 24.0
#define AUX_ROT_RATIO 5.0
#define CLAW_ROT_RATIO 43

// ---- working area geometry (mm) ----
// x: 0..ZONE_W, left to right across the sheet
// y: 0..ZONE_H, ZONE_H is the edge NEAREST the arm base
// the base sits ZONE_GAP beyond the near edge
#define ZONE_W 400.0
#define ZONE_H 184.0
#define ZONE_GAP 120.0

// ---- arm geometry (mm) ----
#define BASE_ELEVATION 120.0
#define MAIN_ARM_LEN 136.0
#define AUX_ARM_LEN 96.0
#define CLAW_EXTENSION 40.0
#define CLAW_HEIGHT 104.0

#define MAIN_ARM_INIT 57

typedef struct {
    int base;
    int main;
    int aux;
    int claw;
    bool valid;
} MotorAngles;

/**
 * @brief Calculates absolute motor angles for a pose.
 *
 * @param x,y,z Target position in mm (z = 0 is the sheet surface).
 * @param a     Claw rotation in degrees.
 * @return .valid is false if the pose is unreachable or numerically invalid.
 */
MotorAngles calculate_motor_angles(int x, int y, int z, int a);