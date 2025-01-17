#include <Arduino.h>
#include <Odometry>

using namespace rudra;

// Odom primitives
Pose initPose;
Arena arena(
    8000.0f,
    15000.0f,
    initPose);

// Encoder ob
ThreeXYEncoder xyEnc(
    400.0f,
    48.0f,
    360,
    initPose,
    arena);

// Use this object to call functions
Odometry<ThreeXYEncoder> odom(xyEnc);

void setup()
{
}
