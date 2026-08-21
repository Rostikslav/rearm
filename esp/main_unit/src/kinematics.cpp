#include "kinematics.h"
#include <math.h>

// NOTE: arduino core already defines it and the
// redefinition either warns or silently shadows it. Use our own name.
#define KIN_PI 3.14159265358979323846

namespace {

const double EPS = 1e-9;

inline double to_deg(double rad) {
    return rad * 180.0 / KIN_PI;
}

/**
 * acos / asin that return NaN instead of undefined behaviour when the
 * argument leaves [-1, 1]. A small tolerance absorbs floating point error
 * on poses that are geometrically exactly at the limit.
 */
double safe_acos(double v) {
    if (v > 1.0) {
        if (v <= 1.0 + 1e-9) v = 1.0;
        else return NAN;
    }
    if (v < -1.0) {
        if (v >= -1.0 - 1e-9) v = -1.0;
        else return NAN;
    }
    return acos(v);
}

double safe_asin(double v) {
    if (v > 1.0) {
        if (v <= 1.0 + 1e-9) v = 1.0;
        else return NAN;
    }
    if (v < -1.0) {
        if (v >= -1.0 - 1e-9) v = -1.0;
        else return NAN;
    }
    return asin(v);
}

inline double dist_x(double x) {
    return x - ZONE_W / 2.0;
}

inline double dist_y(double y) {
    return ZONE_H + ZONE_GAP - y;
}

double base_rotation(double x, double y) {
    double dx = dist_x(x);
    double dy = dist_y(y);

    if (fabs(dy) < EPS)
        return NAN;

    return 90.0 + to_deg(atan(dx / dy));
}

double dist_obj(double x, double y) {
    double dx = dist_x(x);
    double dy = dist_y(y);

    return sqrt(dx * dx + dy * dy);
}

/**
 * Vertical offset of the claw tip relative to the shoulder joint.
 */
inline double vertical_offset(double z) {
    return z + CLAW_HEIGHT - BASE_ELEVATION;
}

/**
 * Straight-line distance from the shoulder joint to the wrist.
 */
double shoulder_to_wrist(double x, double y, double z) {
    double planar = dist_obj(x, y) - CLAW_EXTENSION;
    double vertical = vertical_offset(z);

    return sqrt(planar * planar + vertical * vertical);
}

double main_arm_rotation(double x, double y, double z) {
    double diagonal = shoulder_to_wrist(x, y, z);

    if (diagonal < EPS)
        return NAN;

    double angle1 = safe_acos(
        (MAIN_ARM_LEN * MAIN_ARM_LEN + diagonal * diagonal - AUX_ARM_LEN * AUX_ARM_LEN)
        / (2.0 * MAIN_ARM_LEN * diagonal));

    double angle2 = safe_asin(vertical_offset(z) / diagonal);

    return 90.0 - to_deg(angle1 + angle2) - MAIN_ARM_ROT;
}

double aux_arm_rotation(double x, double y, double z) {
    double diagonal = shoulder_to_wrist(x, y, z);

    double angle = safe_acos(
        (MAIN_ARM_LEN * MAIN_ARM_LEN + AUX_ARM_LEN * AUX_ARM_LEN - diagonal * diagonal)
        / (2.0 * MAIN_ARM_LEN * AUX_ARM_LEN));

    return 180.0 - to_deg(angle);
}

double claw_grab_rotation(double w) {
    double angle = safe_asin(((w - CLAW_BASE_DIST) / 2.0) / CLAW_LENGTH);

    return 90.0 - (CLAW_ROT_INIT + to_deg(angle));
}

/**
 * Normalizes an angle into [0, 180).
 */
int normalize_claw_angle(int a) {
    int r = a % 180;
    if (r < 0) r += 180;
    return r;
}

bool isValid(int x, int y, int z, int a, int w) {
    // TODO
    return true;
}

} // namespace

MotorAngles calculate_motor_angles(int x, int y, int z, int a, int w) {
    MotorAngles angles = {0, 0, 0, 0, 0, false};

    if (!isValid(x, y, z, a, w))
        return angles;

    double base = base_rotation(x, y);
    double main = main_arm_rotation(x, y, z);
    double aux  = aux_arm_rotation(x, y, z);
    double grab = claw_grab_rotation(w);

    // Hard safety net. Casting a NaN or an infinity to int is undefined
    // behavior - in practice it produces an arbitrary value, which would
    // command the joints to a random position at full travel.
    if (!isfinite(base) || !isfinite(main) || !isfinite(aux) || !isfinite(grab))
        return angles;

    angles.base = (int)(base * BASE_ROT_RATIO);
    angles.main = (int)(main * MAIN_ARM_RATIO);
    angles.aux  = (int)(aux  * AUX_ARM_RATIO);
    angles.claw = (int)(normalize_claw_angle(a) * CLAW_ROT_RATIO);
    angles.grab = (int)(grab * CLAW_GRAB_RATIO);
    angles.valid = true;

    return angles;
}
