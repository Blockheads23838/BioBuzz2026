package org.firstinspires.ftc.teamcode.blockheads.robot2;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/*
 * R2: Hold Ground Drive. A TeleOp for Robot 2 (goBILDA Strafer mecanum chassis).
 *
 * DRIVING (gamepad 1):
 *   Left stick  = move (field-centric: pushing UP always moves AWAY from the driver, even if the
 *                 robot got spun around).
 *   Right stick X = turn.
 *   Back button = "this way is forward now" (resets field-centric; do it with the robot facing
 *                 away from the drivers).
 *   Y           = turn HOLD on/off (starts ON).
 *
 * HOLD MY GROUND: when you let go of the sticks, the robot locks its spot and heading. If another
 * robot shoves it, it pushes back to the spot (with goBILDA Pinpoint odometry) and turns back to
 * its heading. Without a working Pinpoint (missing, unplugged, or not saying READY) it uses the
 * Control Hub gyro and can only hold its HEADING (mecanum wheel encoders can't tell when the robot
 * is shoved sideways). Touch a stick and you're driving normally again.
 *
 * RULES: holding still is legal defense. Don't use it to block opponents from game pieces (G411),
 * and remember pins: 3 seconds max (G421). The push-back is capped so it resists, not rams (G419).
 *
 * SETUP (check before the first run, robot on blocks):
 *   - Robot Configuration names: front_left_drive, front_right_drive, back_left_drive,
 *     back_right_drive, imu, and (if you have it) pinpoint.
 *   - Push the left stick up: all four wheels should spin so the robot goes FORWARD.
 *     If not, fix the directions below.
 *   - HAND CHECK of the sensor signs. Pits or practice field only: on the match field the robot
 *     must stay still after INIT (rule G304.H). Robot on the floor, press INIT (not Play), and
 *     watch the INIT screen while you move the robot by hand:
 *       1. Push it FORWARD (the way it drives when the left stick is pushed up) -> Pinpoint x UP.
 *       2. Push it to ITS LEFT -> Pinpoint y goes UP.
 *       3. Turn it counter-clockwise (to the left, seen from above) -> BOTH headings go UP.
 *       4. Spin it one full turn on the same spot -> x and y come back to about 0 (within 1 in).
 *     Pinpoint should also say READY. If x or y goes the wrong way, flip that pod in
 *     setEncoderDirections (first = x pod, second = y pod). If the Pinpoint heading goes the
 *     wrong way, check it is mounted flat and the right way up (see goBILDA's Pinpoint guide).
 *     If the gyro heading goes the wrong way, fix HUB_LOGO / HUB_USB. Check the gyro even on a
 *     Pinpoint robot: it takes over if the Pinpoint stops working. If step 4 fails, measure
 *     PINPOINT_X/Y_OFFSET_MM. A wrong sign makes hold push the robot AWAY from its spot, or spin
 *     it around until it faces backwards.
 *     Finish with the robot straight, then press INIT again (that resets x, y and heading to 0).
 */
@TeleOp(name = "R2: Hold Ground Drive", group = "Robot 2")
public class R2HoldGroundTeleOp extends OpMode {

    // ---- Settings to check on Robot 2 ----
    // Motor directions (same as the FTC mecanum samples; flip if a wheel spins backwards).
    static final DcMotor.Direction FRONT_LEFT_DIR = DcMotor.Direction.REVERSE;
    static final DcMotor.Direction BACK_LEFT_DIR = DcMotor.Direction.REVERSE;
    static final DcMotor.Direction FRONT_RIGHT_DIR = DcMotor.Direction.FORWARD;
    static final DcMotor.Direction BACK_RIGHT_DIR = DcMotor.Direction.FORWARD;
    // Control Hub mounting, for the backup gyro. TODO(robot 2): set for how it's mounted.
    static final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    static final RevHubOrientationOnRobot.UsbFacingDirection HUB_USB = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
    // Pinpoint pod positions (mm from the robot's center). TODO(robot 2): measure your pods.
    static final double PINPOINT_X_OFFSET_MM = -84.0, PINPOINT_Y_OFFSET_MM = -168.0;

    static final double STICK_DEADZONE = 0.05;   // ignore tiny stick wobble
    static final double LOCK_AFTER_S = 0.15;     // sticks still for this long = lock the spot
    static final double PINPOINT_DROPOUT_S = 0.2; // Pinpoint not READY this long = stop trusting it

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private GoBildaPinpointDriver pinpoint = null;   // odometry computer (optional)
    private IMU imu = null;                          // Control Hub gyro (backup for heading)
    private final HoldMyGround hold = new HoldMyGround();

    // Which sensor we're using this loop. PINPOINT = spot + heading, GYRO = heading only.
    private enum Source { PINPOINT, GYRO, NONE }
    private Source lastSource = Source.NONE;
    private double lastHeading = 0;           // last loop's heading (from last loop's sensor)
    private double pinpointMinusGyroDeg = 0;  // how far apart the two sensors' "0 degrees" are
    // Pinpoint health: when it last said READY, and its last good reading.
    private boolean pinpointEverReady = false;
    private final ElapsedTime sincePinpointReady = new ElapsedTime();
    private double goodX = 0, goodY = 0, goodHeading = 0;

    private final ElapsedTime idleTimer = new ElapsedTime();   // how long the sticks have been still
    private final ElapsedTime holdTimer = new ElapsedTime();   // time since the last push-back calculation
    private boolean holdEnabled = true;
    private boolean prevY = false;
    private double fieldForwardOffsetDeg = 0;   // set by the Back button

    // INIT: find the motors and sensors. Nothing moves here (rule G304.H).
    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeft = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRight = hardwareMap.get(DcMotor.class, "back_right_drive");
        frontLeft.setDirection(FRONT_LEFT_DIR);
        backLeft.setDirection(BACK_LEFT_DIR);
        frontRight.setDirection(FRONT_RIGHT_DIR);
        backRight.setDirection(BACK_RIGHT_DIR);

        // BRAKE = when power is 0 the motors resist turning, which already helps against pushes.
        for (DcMotor m : new DcMotor[] {frontLeft, frontRight, backLeft, backRight}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Pinpoint odometry, if the robot has one. It tracks our exact spot even when shoved.
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            pinpoint.setOffsets(PINPOINT_X_OFFSET_MM, PINPOINT_Y_OFFSET_MM, DistanceUnit.MM);
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                    GoBildaPinpointDriver.EncoderDirection.FORWARD);
            pinpoint.resetPosAndIMU();   // robot must be still while this calibrates
        } catch (Exception e) {
            pinpoint = null;
        }

        // Always set up the Control Hub's gyro too. It's the backup for heading if the Pinpoint
        // is missing, unplugged, or stops working in the middle of a match.
        try {
            imu = hardwareMap.get(IMU.class, "imu");
            if (!imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(HUB_LOGO, HUB_USB)))) imu = null;
            else imu.resetYaw();
        } catch (Exception e) {
            imu = null;
        }
    }

    // INIT screen: live sensor readings, so you can do the HAND CHECK (see SETUP at the top).
    // Reading sensors doesn't move anything, so this is fine before Play (rule G304.H).
    @Override
    public void init_loop() {
        if (pinpoint != null) {
            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();
            telemetry.addData("Pinpoint", "%s | x %.1f  y %.1f in | heading %.1f deg", pinpoint.getDeviceStatus(),
                    pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH), pose.getHeading(AngleUnit.DEGREES));
            // CALIBRATING for a moment right after INIT is normal. Anything else that isn't READY
            // (NOT_READY or FAULT_... the whole time) means the Pinpoint isn't working.
            GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
            if (status != GoBildaPinpointDriver.DeviceStatus.READY && status != GoBildaPinpointDriver.DeviceStatus.CALIBRATING) {
                telemetry.addLine("Pinpoint not working: check its cable and that the config name is 'pinpoint'.");
            }
        } else {
            telemetry.addData("Pinpoint", "not found - heading hold only");
        }
        telemetry.addData("Gyro heading", imu != null
                ? String.format("%.1f deg", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES))
                : "not found");
        telemetry.addLine("Hand check: push forward = x UP, push left = y UP, turn left = headings UP.");
        telemetry.addLine("Y = hold on/off, Back = reset 'forward'. Hold starts ON.");
    }

    @Override
    public void start() {
        idleTimer.reset();
        holdTimer.reset();
    }

    @Override
    public void loop() {
        // ---- Read where we are ----
        // Only trust the Pinpoint when it says READY. An unplugged Pinpoint just reads 0, 0, 0,
        // which would look like "not moving" and the hold would silently do nothing.
        // One corrupt reading makes it say FAULT_BAD_READ for that loop (it keeps the last good
        // numbers); a missed reading says NOT_READY. For a short blip like that we keep the
        // Pinpoint but pause the push-back. Only if it stays not-READY for PINPOINT_DROPOUT_S
        // (unplugged, broken) do we switch to the gyro.
        boolean pinpointReadyNow = false;
        if (pinpoint != null) {
            pinpoint.update();
            pinpointReadyNow = pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
            if (pinpointReadyNow) {
                pinpointEverReady = true;
                sincePinpointReady.reset();
                Pose2D pose = pinpoint.getPosition();
                goodX = pose.getX(DistanceUnit.INCH);
                goodY = pose.getY(DistanceUnit.INCH);
                goodHeading = pose.getHeading(AngleUnit.DEGREES);
            }
        }
        boolean pinpointBlip = !pinpointReadyNow && pinpointEverReady
                && sincePinpointReady.seconds() < PINPOINT_DROPOUT_S;

        double x = 0, y = 0, heading = 0;
        boolean knowPosition = false, knowHeading = false;
        Source source;
        if (pinpointReadyNow || pinpointBlip) {
            // During a blip these are the last good numbers (at most 0.2 s old).
            x = goodX;
            y = goodY;
            heading = goodHeading;
            knowPosition = true;
            knowHeading = true;
            source = Source.PINPOINT;
        } else if (imu != null) {
            heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            knowHeading = true;
            source = Source.GYRO;
        } else {
            source = Source.NONE;
        }
        // If we switched sensors (for example the Pinpoint dropped out), forget the old locked spot:
        // it was measured by the other sensor. The hold re-locks on the new sensor's numbers.
        // The two sensors can disagree about which way is 0 degrees (both are 0 at INIT, but a
        // Pinpoint that restarts comes back at 0). So also shift the field-centric "forward" by the
        // difference, and stick-up still points the same way on the field.
        if (source != lastSource) {
            hold.release();
            if (lastSource == Source.GYRO && source == Source.PINPOINT) {
                // Both readings are fresh here (gyro last loop, Pinpoint now), so measure the gap.
                pinpointMinusGyroDeg = heading - lastHeading;
                fieldForwardOffsetDeg += pinpointMinusGyroDeg;
            } else if (lastSource == Source.PINPOINT && source == Source.GYRO) {
                // The Pinpoint's last reading may be 0.2 s old, so use the gap we already know.
                fieldForwardOffsetDeg -= pinpointMinusGyroDeg;
            }
        }
        lastSource = source;
        lastHeading = heading;

        // ---- Buttons ----
        // Y toggles hold on/off (one press = one change).
        if (gamepad1.y && !prevY) {
            holdEnabled = !holdEnabled;
            hold.release();
        }
        prevY = gamepad1.y;
        // Back: "the robot is facing away from us now", for field-centric driving.
        if (gamepad1.back) fieldForwardOffsetDeg = heading;

        // ---- Driver sticks ----
        double stickForward = -gamepad1.left_stick_y;   // pushing up gives a negative number
        double stickRight = gamepad1.left_stick_x;
        double stickTurn = gamepad1.right_stick_x;
        boolean driverActive = Math.abs(stickForward) > STICK_DEADZONE || Math.abs(stickRight) > STICK_DEADZONE
                || Math.abs(stickTurn) > STICK_DEADZONE;

        double axial, lateral, yaw;
        if (driverActive) {
            // The driver is driving: stop holding, and drive field-centric.
            hold.release();
            idleTimer.reset();
            // Field-centric: turn the stick direction by how far the robot has rotated, so "up" on
            // the stick always means "away from the drivers".
            // (Example: robot turned 90 deg left, stick up -> robot must slide to ITS right.)
            // No heading sensor at all = plain robot-centric driving (stick up = robot's forward).
            double a = knowHeading ? Math.toRadians(heading - fieldForwardOffsetDeg) : 0;
            axial = stickForward * Math.cos(a) - stickRight * Math.sin(a);
            lateral = stickForward * Math.sin(a) + stickRight * Math.cos(a);
            yaw = stickTurn;
        } else if (holdEnabled && knowHeading && idleTimer.seconds() > LOCK_AFTER_S && !pinpointBlip) {
            // Sticks have been still: lock this spot (once), then push back if shoved.
            if (!hold.isLocked()) hold.lock(x, y, heading);
            // dt = time since the last push-back calculation (not just this loop), so the speed
            // estimate stays right even after a paused loop.
            double dt = holdTimer.seconds();
            holdTimer.reset();
            double[] c = hold.correction(x, y, heading, knowPosition, dt);
            axial = c[0];
            lateral = c[1];
            yaw = c[2];
        } else {
            // Driver isn't driving and we're not pushing back (hold off, no sensor, just let go,
            // or a Pinpoint blip): zero power, and BRAKE resists on its own.
            axial = 0;
            lateral = 0;
            yaw = 0;
        }

        // ---- Mecanum wheel math (same as the FTC samples) ----
        double fl = axial + lateral + yaw;
        double fr = axial - lateral - yaw;
        double bl = axial - lateral + yaw;
        double br = axial + lateral - yaw;
        // If any wheel would get more than 100%, scale them all down together.
        double biggest = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br))));
        frontLeft.setPower(fl / biggest);
        frontRight.setPower(fr / biggest);
        backLeft.setPower(bl / biggest);
        backRight.setPower(br / biggest);

        // ---- Screen ----
        telemetry.addData("Hold (Y)", !holdEnabled ? "OFF"
                : !hold.isLocked() ? "ON - let go of sticks to lock"
                : pinpointBlip ? "LOCKED - paused for a moment" : "LOCKED - pushing back if shoved");
        telemetry.addData("Sensor", source == Source.PINPOINT
                ? (pinpointBlip ? "Pinpoint missed a reading - push-back paused" : "Pinpoint READY - full hold (spot + heading)")
                : source == Source.GYRO ? "gyro only - heading hold (Pinpoint " + (pinpoint != null ? pinpoint.getDeviceStatus() : "not found") + ")"
                : "NONE - hold can't work");
        if (hold.isLocked()) {
            // "turned" uses the same sign as Heading below: + = shoved counter-clockwise.
            // (0.0 - error instead of -error, so it never prints "-0.0".)
            telemetry.addData("Shoved by", "%.1f in, turned %.1f deg", hold.distanceFromLockIn(x, y), 0.0 - hold.headingErrorDeg(heading));
        }
        telemetry.addData("Position", knowPosition ? String.format("x %.1f  y %.1f in", x, y)
                : pinpoint != null ? "unknown (Pinpoint not READY)" : "unknown (no Pinpoint)");
        telemetry.addData("Heading", knowHeading ? String.format("%.1f deg", heading) : "unknown");
    }

    @Override
    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
