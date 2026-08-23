#include "kinematics.h"
#include <math.h>

// The Arduino core already defines PI, so use our own name.
#define KIN_PI 3.14159265358979323846

namespace {

const double EPS = 1e-9;

inline int max(int a, int b) {
    return a > b ? a : b;
}

inline double to_deg(double rad) {
    return rad * 180.0 / KIN_PI;
}

// acos / asin returning NaN instead of undefined behavior outside [-1, 1].
// The tolerance absorbs float error on poses exactly at the limit.
double safe_acos(double v) {
    if (v > 1.0)  return (v <= 1.0 + 1e-9) ? 0.0 : NAN;
    if (v < -1.0) return (v >= -1.0 - 1e-9) ? KIN_PI : NAN;
    return acos(v);
}

double safe_asin(double v) {
    if (v > 1.0)  return (v <= 1.0 + 1e-9) ? KIN_PI / 2 : NAN;
    if (v < -1.0) return (v >= -1.0 - 1e-9) ? -KIN_PI / 2 : NAN;
    return asin(v);
}

// x-distance to the object
inline double dist_x(double x) {
    return x - ZONE_W / 2.0;
}

// y-distance to the object
inline double dist_y(double y) {
    return ZONE_H + ZONE_GAP - y;
}

// euclidean distance to the object
double dist_obj(double x, double y) {
    double dx = dist_x(x);
    double dy = dist_y(y);

    return sqrt(dx * dx + dy * dy);
}

// claw tip height relative to the shoulder joint
inline double vertical_offset(double z) {
    return z + CLAW_HEIGHT - BASE_ELEVATION;
}

// straight-line distance from the shoulder joint to the wrist
double shoulder_to_wrist(double x, double y, double z) {
    double planar = dist_obj(x, y) - CLAW_EXTENSION;
    double vertical = vertical_offset(z);

    return sqrt(planar * planar + vertical * vertical);
}

double base_rotation(double x, double y) {
    double dx = dist_x(x);
    double dy = dist_y(y);

    if (fabs(dy) < EPS)
        return NAN;

    return 90.0 + to_deg(atan(dx / dy));
}

double main_arm_rotation(double x, double y, double z) {
    double diagonal = shoulder_to_wrist(x, y, z);

    if (diagonal < EPS)
        return NAN;

    double angle1 = safe_acos(
        (MAIN_ARM_LEN * MAIN_ARM_LEN + diagonal * diagonal - AUX_ARM_LEN * AUX_ARM_LEN)
        / (2.0 * MAIN_ARM_LEN * diagonal));

    double angle2 = safe_asin(vertical_offset(z) / diagonal);

    return 90.0 - to_deg(angle1 + angle2) - MAIN_ARM_INIT;
}

double aux_arm_rotation(double x, double y, double z) {
    double diagonal = shoulder_to_wrist(x, y, z);

    double angle = safe_acos(
        (MAIN_ARM_LEN * MAIN_ARM_LEN + AUX_ARM_LEN * AUX_ARM_LEN - diagonal * diagonal)
        / (2.0 * MAIN_ARM_LEN * AUX_ARM_LEN));

    return 180.0 - to_deg(angle);
}

// normalizes into [0, 180) with small error tolerance
int normalize_claw_angle(int a) {
    int r = a % 180;
    if (r < -5) r += 180;
    else if (r < 0) r = 0;
    return r;
}

// the claw angle is relative to the arm, so the base rotation has to come out
double claw_rotation(double a, double base) {
    return normalize_claw_angle((int)(a + base - 90));
}

} // namespace

MotorAngles calculate_motor_angles(int x, int y, int z, int a) {
    MotorAngles angles = {0, 0, 0, 0, false};

    double base = base_rotation(x, y);
    double main = main_arm_rotation(x, y, z);
    double aux  = aux_arm_rotation(x, y, z);
    double claw = claw_rotation(a, base);

    // casting NaN or infinity to int is undefined behavior and in practice
    // yields an arbitrary value that would send a joint to full travel
    if (!isfinite(base) || !isfinite(main) || !isfinite(aux) || !isfinite(claw))
        return angles;

    angles.base = max(0, (int)(base * BASE_ROT_RATIO));
    angles.main = max(0, (int)(main * MAIN_ROT_RATIO));
    angles.aux  = max(0, (int)(aux  * AUX_ROT_RATIO));
    angles.claw = max(0, (int)(claw * CLAW_ROT_RATIO));
    angles.valid = true;

    return angles;
}
