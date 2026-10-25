package org.firstinspires.ftc.teamcode.blockheads.robot2;

/*
 * HoldMyGround: the "anti-push" brain for a mecanum robot.
 *
 * Idea in plain words: when the driver lets go of the sticks, we remember exactly where the robot
 * is and which way it faces ("lock" that spot). If another robot shoves us, this works out how
 * hard to drive back to the locked spot, and how hard to turn back to the locked heading.
 * Mecanum wheels can push back in ANY direction, so the robot fights to stay put.
 *
 * This class is only math: it doesn't touch motors or sensors, so it can be tested on a computer.
 * The OpMode reads the position (from goBILDA Pinpoint odometry) and heading, calls correction(),
 * and sends the answer to the wheels.
 *
 * Safety / rules built in:
 *  - It only pushes back up to MAX power (it resists; it doesn't ram anyone, rule G419).
 *  - If we get shoved really far (2 ft), it stops fighting and locks the NEW spot instead of
 *    driving across the field back to the old one.
 *  - Tiny errors are ignored (a "deadband") so the robot doesn't jitter in place.
 *
 * Directions: x, y in inches on the field (like Pinpoint: x = forward from where it started,
 * y = to the LEFT). Heading in degrees, counter-clockwise = positive (like the gyro).
 * The answer uses the same names as the FTC mecanum samples: axial (forward), lateral (right),
 * yaw (turn RIGHT = positive).
 */
public class HoldMyGround {

    // ---- How strong the push-back is (tune these on the robot) ----
    public double kPosition = 0.06;          // drive power per inch we are away from the spot
    public double kPositionDamping = 0.004;  // calms the robot down so it doesn't overshoot (per in/s)
    public double kHeading = 0.02;           // turn power per degree we are turned away
    public double kHeadingDamping = 0.0008;  // calms turning (per deg/s)
    public double minPower = 0.06;           // smallest push that actually moves the robot
    public double maxDrivePower = 0.6;       // never push back harder than this
    public double maxTurnPower = 0.5;

    // ---- When to ignore or give up ----
    public double positionDeadbandIn = 0.5;  // closer than this = good enough
    public double headingDeadbandDeg = 1.0;
    public double giveUpDistanceIn = 24;     // shoved further than 2 ft = stop fighting, new spot

    // The locked spot and heading.
    private boolean locked = false;
    private double lockX, lockY, lockHeadingDeg;

    // Last readings, to work out how fast we're moving (for the damping).
    private double lastX, lastY, lastHeadingDeg;
    private boolean haveLast = false;

    /** Remember this spot and heading as the one to hold. */
    public void lock(double xIn, double yIn, double headingDeg) {
        locked = true;
        lockX = xIn;
        lockY = yIn;
        lockHeadingDeg = headingDeg;
        haveLast = false;
    }

    /** Stop holding (the driver is driving again). */
    public void release() {
        locked = false;
        haveLast = false;
    }

    public boolean isLocked() {
        return locked;
    }

    /** How far (inches) we are from the locked spot right now. */
    public double distanceFromLockIn(double xIn, double yIn) {
        return locked ? Math.hypot(lockX - xIn, lockY - yIn) : 0;
    }

    /** How far (degrees) we are turned away from the locked heading right now. */
    public double headingErrorDeg(double headingDeg) {
        return locked ? wrapDegrees(lockHeadingDeg - headingDeg) : 0;
    }

    /**
     * Work out how to push back.
     *
     * @param xIn, yIn      where we are now (inches). Ignored if knowPosition is false.
     * @param headingDeg    which way we face now (degrees, counter-clockwise +).
     * @param knowPosition  true if we have odometry (Pinpoint). Without it we can only hold heading,
     *                      because mecanum wheel encoders can't tell when we're shoved sideways.
     * @param dtSeconds     time since the last call (for the damping).
     * @return {axial, lateral, yaw} wheel commands, each between -1 and 1. All 0 if not locked.
     */
    public double[] correction(double xIn, double yIn, double headingDeg, boolean knowPosition, double dtSeconds) {
        if (!locked) return new double[] {0, 0, 0};

        // How fast are we moving / turning? (Used to stop the robot overshooting the spot.)
        double vx = 0, vy = 0, turnRate = 0;
        if (haveLast && dtSeconds > 0.001) {
            vx = (xIn - lastX) / dtSeconds;
            vy = (yIn - lastY) / dtSeconds;
            turnRate = wrapDegrees(headingDeg - lastHeadingDeg) / dtSeconds;
        }
        lastX = xIn; lastY = yIn; lastHeadingDeg = headingDeg; haveLast = true;

        double axial = 0, lateral = 0;
        if (knowPosition) {
            double ex = lockX - xIn, ey = lockY - yIn;    // field error: how far to go back
            double dist = Math.hypot(ex, ey);

            // Shoved really far? Don't chase across the field: hold this new spot instead.
            if (dist > giveUpDistanceIn) {
                lock(xIn, yIn, headingDeg);
                return new double[] {0, 0, 0};
            }

            if (dist > positionDeadbandIn) {
                // Push toward the spot, minus a bit for how fast we're already moving back.
                double fx = kPosition * ex - kPositionDamping * vx;
                double fy = kPosition * ey - kPositionDamping * vy;
                // Make sure the push is big enough to move, but never bigger than the maximum.
                double mag = Math.hypot(fx, fy);
                if (mag > 1e-6) {
                    double scaled = clip(Math.max(mag, minPower), 0, maxDrivePower);
                    fx = fx / mag * scaled;
                    fy = fy / mag * scaled;
                }
                // Turn the field push into robot directions (forward / sideways) using our heading.
                double h = Math.toRadians(headingDeg);
                double forward = fx * Math.cos(h) + fy * Math.sin(h);
                double left = -fx * Math.sin(h) + fy * Math.cos(h);
                axial = forward;
                lateral = -left;   // the samples use "lateral = right"
            }
        }

        // Turn back to the locked heading.
        double yaw = 0;
        double eh = wrapDegrees(lockHeadingDeg - headingDeg);   // + = we need to turn LEFT
        if (Math.abs(eh) > headingDeadbandDeg) {
            double turnLeft = kHeading * eh - kHeadingDamping * turnRate;
            if (Math.abs(turnLeft) < minPower) turnLeft = Math.signum(eh) * minPower;
            turnLeft = clip(turnLeft, -maxTurnPower, maxTurnPower);
            yaw = -turnLeft;   // the samples use "yaw = turn RIGHT"
        }
        return new double[] {axial, lateral, yaw};
    }

    // Keep an angle between -180 and 180 (so 350 degrees off becomes 10 degrees off).
    static double wrapDegrees(double deg) {
        double d = deg % 360;
        if (d > 180) d -= 360;
        if (d < -180) d += 360;
        return d;
    }

    static double clip(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
