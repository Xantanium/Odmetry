#include "ThreeXYEncoder"

namespace rudra {
Pose ThreeXYEncoder::update(long countX, long count_Y_R, long count_Y_L)
{
    // Convert counts to distances
    rawX = countX * mmsPerPulse;
    rawYR = count_Y_R * mmsPerPulse;
    rawYL = count_Y_L * mmsPerPulse;

    // calculate yaw movement

    /**
     * This comes from:
     *
     * Theta = arcLength / radius.
     *
     * arclength is calculated by the simultaneous movements of opposite y encoders.
     * arcLength = (y1movement - y2movement) / 2.
     * e.g.: y1 moves 10, hence y2 moves -10. therefore net movement is
     * ((10) - (-10)) / 2 = 10.
     *
     * radius = wheelSeperation / 2
     *
     * combining arclength and radius, 2s in denominators cancel out to give below expression.*/
    deltaYaw = (rawYR - rawYL) / wheelSeperation;

    // Calculate X correction

    /**
     * The X correction works like this:
     *
     * While the robot is rotating about its axis, x encoder also traces an arc.
     * Manipulating the above formula, the arc traced by X encoder can be calculated by:
     *
     * xArcLength = theta * radius.
     *
     * X encoder readings obtained while rotation are garbage for net x movement.
     * Hence, we subtract the x arc length from the raw x to obtain net x movement. */
    xCorrection = deltaYaw * (wheelSeperation / 2);
    xCorrected = rawX - xCorrection;

    // calculate local changes in positions
    dXLocal = xCorrected;
    dYLocal = (rawYR + rawYR) / 2.0f; // average of both Y encs

    // Globalization
    dXGlobal = (dXLocal * std::cos(deltaYaw))
        - (dYLocal * std::sin(deltaYaw));

    dYGlobal = (dXLocal * std::sin(deltaYaw))
        + (dYLocal * std::cos(deltaYaw));

    // Append to pose
    odomPose.x += dXGlobal;
    odomPose.y += dYGlobal;

    odomPose.w += deltaYaw;

    // Normalize the angle
    odomPose.w = fmod(odomPose.w, 2 * M_PI);
    if (odomPose.w >= M_PI) {
        odomPose.w -= 2 * M_PI;
    } else if (odomPose.w < -M_PI) {
        odomPose.w += 2 * M_PI;
    }

    return odomPose;
}

} // namespace rudra
