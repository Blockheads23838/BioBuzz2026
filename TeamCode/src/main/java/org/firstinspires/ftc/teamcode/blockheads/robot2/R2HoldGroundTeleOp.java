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
 * its heading. Without Pinpoint it can only hold its HEADING (mecanum wheel encoders can't tell
 * when the robot is shoved sideways). Touch a stick and you're driving normally again.
 *
 * RULES: holding still is legal defense. Don't use it to block opponents from game pieces (G411),
 * and remember pins: 3 seconds max (G421). The push-back is capped so it resists, not rams (G419).
 *
 * SETUP (check before the first run, robot on blocks):
 *   - Robot Configuration names: front_left_drive, front_right_drive, back_left_drive,
 *     back_right_drive, imu, and (if you have it) pinpoint.
 *   - Push the left stick up: all four wheels should spin so the robot goes FORWARD.
 *     If not, fix the directions below.
 */
@TeleOp(name = "R2: Hold Ground Drive", group = "Robot 2")
public class R2HoldGroundTeleOp extends OpMode {

    // ---- Settings to check on Robot 2 ----
    // Motor directions (same as the FTC mecanum samples; flip if a wheel spins backwards).
    static final DcMotor.Direction FRONT_LEFT_DIR = DcMotor.Direction.REVERSE;
    static final DcMotor.Direction BACK_LEFT_DIR = DcMotor.Direction.REVERSE;
    static final DcMotor.Direction FRONT_RIGHT_DIR = DcMotor.Direction.FORWARD;
    static final DcMotor.Direction BACK_RIGHT_DIR = DcMotor.Direction.FORWARD;
    // Control Hub mounting, used only if there's no Pinpoint. TODO(robot 2): set for how it's mounted.
    static final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    static final RevHubOrientationOnRobot.UsbFacingDirection HUB_USB = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
    // Pinpoint pod positions (mm from the robot's center). TODO(robot 2): measure your pods.
    static final double PINPOINT_X_OFFSET_MM = -84.0, PINPOINT_Y_OFFSET_MM = -168.0;

    static final double STICK_DEADZONE = 0.05;   // ignore tiny stick wobble
    static final double LOCK_AFTER_S = 0.15;     // sticks still for this long = lock the spot

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private GoBildaPinpointDriver pinpoint = null;   // odometry computer (optional)
    private IMU imu = null;                          // Control Hub gyro (backup for heading)
    private final HoldMyGround hold = new HoldMyGround();

    private final ElapsedTime idleTimer = new ElapsedTime();   // how long the sticks have been still
    private final ElapsedTime loopTimer = new ElapsedTime();
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

        // Without Pinpoint, use the Control Hub's gyro for heading.
        if (pinpoint == null) {
            try {
                imu = hardwareMap.get(IMU.class, "imu");
                if (!imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(HUB_LOGO, HUB_USB)))) imu = null;
                else imu.resetYaw();
            } catch (Exception e) {
                imu = null;
            }
        }
    }

    @Override
    public void init_loop() {
        telemetry.addData("Position sensor", pinpoint != null ? "Pinpoint OK - full hold (spot + heading)" : "no Pinpoint - heading hold only");
        telemetry.addData("Heading sensor", pinpoint != null ? "Pinpoint" : (imu != null ? "Control Hub gyro" : "NONE - hold disabled"));
        telemetry.addLine("Y = hold on/off, Back = reset 'forward'. Hold starts ON.");
    }

    @Override
    public void start() {
        idleTimer.reset();
        loopTimer.reset();
    }

    @Override
    public void loop() {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        // ---- Read where we are ----
        double x = 0, y = 0, heading = 0;
        boolean knowPosition = false, knowHeading = false;
        if (pinpoint != null) {
            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();
            x = pose.getX(DistanceUnit.INCH);
            y = pose.getY(DistanceUnit.INCH);
            heading = pose.getHeading(AngleUnit.DEGREES);
            knowPosition = true;
            knowHeading = true;
        } else if (imu != null) {
            heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            knowHeading = true;
        }

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
            double a = Math.toRadians(heading - fieldForwardOffsetDeg);
            axial = stickForward * Math.cos(a) - stickRight * Math.sin(a);
            lateral = stickForward * Math.sin(a) + stickRight * Math.cos(a);
            yaw = stickTurn;
        } else if (holdEnabled && knowHeading && idleTimer.seconds() > LOCK_AFTER_S) {
            // Sticks have been still: lock this spot (once), then push back if shoved.
            if (!hold.isLocked()) hold.lock(x, y, heading);
            double[] c = hold.correction(x, y, heading, knowPosition, dt);
            axial = c[0];
            lateral = c[1];
            yaw = c[2];
        } else {
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
        telemetry.addData("Hold (Y)", holdEnabled ? (hold.isLocked() ? "LOCKED - pushing back if shoved" : "ON - lets go of sticks to lock") : "OFF");
        if (hold.isLocked()) {
            telemetry.addData("Shoved by", "%.1f in, %.1f deg", hold.distanceFromLockIn(x, y), hold.headingErrorDeg(heading));
        }
        telemetry.addData("Position", knowPosition ? String.format("x %.1f  y %.1f in", x, y) : "unknown (no Pinpoint)");
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
