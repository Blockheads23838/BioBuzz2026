package org.firstinspires.ftc.teamcode.blockheads;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * BH Bench Check: the "Phase 0" robot check from Autonomous-Plan.md, as one test program.
 *
 * It answers the questions we must know BEFORE tuning any autonomous:
 *   1. Are all 7 devices in the Robot Configuration, with the right names and types?
 *   2. Does anything move after INIT? (It shouldn't. A servo jumping = still in position mode.)
 *   3. When the robot drives "forward", do BOTH wheel encoders count up?
 *   4. Pushing the robot 24 inches gives about 1087 encoder ticks per side?
 *   5. Does the launcher's speed read POSITIVE when it spins up? How fast does it get there?
 *   6. When the robot turns left, does the gyro (IMU) angle go up?
 *   7. Which end does the launcher shoot out of? (You watch and write it down.)
 *
 * It uses the SAME names and directions as BioBuzzStarterBotTeleop, so what you see here
 * is what the real programs do. It is a NEW file; it changes nothing else.
 *
 * SAFETY: put the robot on blocks (wheels off the ground) for the first try.
 * Every motor only runs while you HOLD its button, and stops when you let go.
 */
@TeleOp(name = "BH Bench Check", group = "BH Dev")
public class BHBenchCheck extends OpMode {

    // ---- Numbers we test against (same values as the team code) ----
    static final double TICKS_PER_INCH = 537.7 / (Math.PI * 96.0 / 25.4); // about 45.3 ticks per inch of travel
    static final double DRIVE_TEST_POWER = 0.3;     // slow, for safety
    static final double INTAKE_TEST_POWER = 0.5;
    static final double LAUNCHER_TARGET = 1250;     // ticks per second, same as the team code
    static final double LAUNCHER_READY = 1200;      // "fast enough to shoot", same as the team code

    // ---- The robot's parts. Any of these can be null if it's missing from the configuration. ----
    private DcMotor leftDrive, rightDrive, intake;
    private DcMotorEx launcher;
    private CRServo windmill, leftIntakeServo, rightIntakeServo;
    private IMU imu;
    private final StringBuilder missing = new StringBuilder();

    // ---- Stopwatches and remembered values ----
    private final ElapsedTime initWatch = new ElapsedTime();   // counts the 10 s "nothing moves" watch
    private final ElapsedTime spinUpTimer = new ElapsedTime(); // times the launcher spin-up
    private int leftZero = 0, rightZero = 0;                   // encoder readings when you last pressed X
    private int leftAtPress = 0, rightAtPress = 0;             // encoder readings when you started holding a dpad button
    private double yawAtPress = 0;
    private boolean wasDriving = false, wasSpinningUp = false;
    private double spinUpSeconds = -1;                         // -1 = not measured yet
    private String driveResult = "not tested yet", turnResult = "not tested yet", launcherResult = "not tested yet";

    // INIT: find every part and set it up exactly like BioBuzzStarterBotTeleop does.
    // Nothing is told to move here (rule G304.H: the robot must be still after INIT).
    @Override
    public void init() {
        // Look up each part by its configuration name. tryGet() returns null instead of crashing,
        // so we can list EVERY missing name at once instead of stopping at the first one.
        leftDrive = find(DcMotor.class, "left_drive");
        rightDrive = find(DcMotor.class, "right_drive");
        intake = find(DcMotor.class, "intake");
        launcher = find(DcMotorEx.class, "launcher");
        windmill = find(CRServo.class, "windmill");
        leftIntakeServo = find(CRServo.class, "left_intake_servo");
        rightIntakeServo = find(CRServo.class, "right_intake_servo");

        // Same motor directions as the team TeleOp: one side is reversed so both sides drive forward together.
        if (leftDrive != null) { leftDrive.setDirection(DcMotor.Direction.FORWARD); leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); }
        if (rightDrive != null) { rightDrive.setDirection(DcMotor.Direction.REVERSE); rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); }
        if (intake != null) intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Launcher: speed control (RUN_USING_ENCODER = the motor uses its encoder to hold a speed),
        // with the same PIDF tuning numbers as the team code.
        if (launcher != null) {
            launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));
        }

        // Continuous servos: power 0, same as the team code. If a servo JUMPS right now,
        // it is still in "position" mode and must be switched with the goBILDA servo programmer.
        if (leftIntakeServo != null) leftIntakeServo.setPower(0);
        if (rightIntakeServo != null) { rightIntakeServo.setPower(0); rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE); }
        if (windmill != null) { windmill.setPower(0); windmill.setDirection(DcMotorSimple.Direction.REVERSE); }

        // The gyro (IMU) inside the Control Hub. It's optional for now, so a missing "imu" is not an error.
        // The orientation here is a guess; the yaw (turning) check works for any flat mounting.
        imu = hardwareMap.tryGet(IMU.class, "imu");
        if (imu != null) {
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
            imu.resetYaw();
        }
        initWatch.reset();
    }

    // While waiting for START: show the config check and the "watch for movement" countdown.
    // Encoders are shown too, so you can already do the 24-inch push test here.
    @Override
    public void init_loop() {
        telemetry.addLine("=== CHECK 1: configuration names ===");
        telemetry.addLine(missing.length() == 0 ? "OK: all 7 devices found" : "MISSING or wrong type: " + missing);

        telemetry.addLine("=== CHECK 2: nothing moves after INIT ===");
        double left = Math.max(0, 10 - initWatch.seconds());
        telemetry.addLine(left > 0
                ? String.format("Watch the robot for %.0f more seconds. NOTHING should move or twitch.", left)
                : "Done. Did anything move? If a servo jumped, switch it to continuous mode.");

        showEncoders();
        telemetry.addLine("Press START for the motor tests.");
    }

    // START pressed: begin with encoders zeroed at the robot's current spot.
    @Override
    public void start() {
        zeroEncoders();
    }

    // Runs over and over after START. Each test runs only while its button is HELD.
    @Override
    public void loop() {
        // X: set the encoder distance back to 0 (use before the 24-inch push test).
        if (gamepad1.x) zeroEncoders();

        // ---- CHECK 3: drive forward / backward (dpad up / down) ----
        // Both sides get the same power. With the team directions this is "TeleOp forward".
        boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down;
        boolean spinLeft = gamepad1.dpad_left, spinRight = gamepad1.dpad_right;
        boolean driving = up || down || spinLeft || spinRight;

        // When a dpad button is first pressed, remember where the encoders and gyro started.
        if (driving && !wasDriving) {
            leftAtPress = position(leftDrive);
            rightAtPress = position(rightDrive);
            yawAtPress = yaw();
        }

        double leftPower = 0, rightPower = 0;
        if (up)        { leftPower = DRIVE_TEST_POWER;  rightPower = DRIVE_TEST_POWER; }
        else if (down) { leftPower = -DRIVE_TEST_POWER; rightPower = -DRIVE_TEST_POWER; }
        // ---- CHECK 6: spin in place (dpad left / right) to test the gyro ----
        else if (spinLeft)  { leftPower = -DRIVE_TEST_POWER; rightPower = DRIVE_TEST_POWER; }
        else if (spinRight) { leftPower = DRIVE_TEST_POWER;  rightPower = -DRIVE_TEST_POWER; }
        if (leftDrive != null) leftDrive.setPower(leftPower);
        if (rightDrive != null) rightDrive.setPower(rightPower);

        // While driving forward, decide automatically whether both encoders count up.
        int dl = position(leftDrive) - leftAtPress, dr = position(rightDrive) - rightAtPress;
        if (up && (Math.abs(dl) > 100 || Math.abs(dr) > 100)) {
            if (dl > 0 && dr > 0) driveResult = "OK: both encoders count UP going forward";
            else if (Math.abs(dl) < 20 || Math.abs(dr) < 20) driveResult = "PROBLEM: one encoder isn't counting (cable unplugged?)";
            else driveResult = "PROBLEM: an encoder counts DOWN going forward (note which side)";
        }
        // While spinning left (counter-clockwise from above), the gyro yaw should go UP.
        if (spinLeft && imu != null) {
            double turned = yaw() - yawAtPress;
            if (Math.abs(turned) > 15) turnResult = turned > 0 ? "OK: yaw goes UP turning left" : "CHECK: yaw goes DOWN turning left (hub mounted upside down?)";
        }
        wasDriving = driving;

        // ---- CHECK 5: launcher (hold right bumper) ----
        // Spin up to the team's target speed, time how long it takes to be "ready",
        // and check that the speed reads positive.
        boolean spinning = gamepad1.right_bumper;
        if (launcher != null) {
            if (spinning && !wasSpinningUp) { spinUpTimer.reset(); spinUpSeconds = -1; }
            launcher.setVelocity(spinning ? LAUNCHER_TARGET : 0);
            double v = launcher.getVelocity();
            if (spinning && spinUpSeconds < 0 && v >= LAUNCHER_READY) spinUpSeconds = spinUpTimer.seconds();
            if (spinning && spinUpTimer.seconds() > 1.0) {
                if (v < -100) launcherResult = "PROBLEM: speed reads NEGATIVE (launcher direction must be reversed)";
                else if (v < 100) launcherResult = "PROBLEM: speed reads ~0 (encoder cable unplugged?)";
                else if (spinUpSeconds > 0) launcherResult = String.format("OK: positive, ready in %.2f s", spinUpSeconds);
                else launcherResult = "SLOW: still below 1200 after 1 s";
            }
        }
        wasSpinningUp = spinning;

        // ---- Feeders: hold A = windmill feeds the launcher, hold B = intake roller + corner servos pull in ----
        if (windmill != null) windmill.setPower(gamepad1.a ? 1.0 : 0);
        double intakePower = gamepad1.b ? INTAKE_TEST_POWER : 0;
        if (intake != null) intake.setPower(intakePower);
        if (leftIntakeServo != null) leftIntakeServo.setPower(intakePower);
        if (rightIntakeServo != null) rightIntakeServo.setPower(intakePower);

        // ---- Show everything on the Driver Station ----
        telemetry.addLine("HOLD: dpad up/down = drive, dpad left/right = spin, RB = launcher, A = windmill, B = intake. X = zero encoders");
        telemetry.addData("Config", missing.length() == 0 ? "OK: all 7 devices found" : "MISSING: " + missing);
        showEncoders();
        telemetry.addData("CHECK 3 drive", driveResult);
        telemetry.addData("Launcher speed", launcher == null ? "n/a" : String.format("%.0f ticks/s (target %.0f)", launcher.getVelocity(), LAUNCHER_TARGET));
        telemetry.addData("CHECK 5 launcher", launcherResult);
        telemetry.addData("CHECK 6 gyro", imu == null ? "no \"imu\" in config (OK for now)" : String.format("yaw %.1f deg | %s", yaw(), turnResult));
        telemetry.addData("Battery", "%.2f V", batteryVoltage());
        telemetry.addLine("CHECK 7: while holding RB + A on blocks, WHICH END do balls come out? Write it down.");
    }

    // OpMode ending: make sure every motor and servo is off.
    @Override
    public void stop() {
        if (leftDrive != null) leftDrive.setPower(0);
        if (rightDrive != null) rightDrive.setPower(0);
        if (intake != null) intake.setPower(0);
        if (launcher != null) launcher.setPower(0);
        if (windmill != null) windmill.setPower(0);
        if (leftIntakeServo != null) leftIntakeServo.setPower(0);
        if (rightIntakeServo != null) rightIntakeServo.setPower(0);
    }

    // ---------- Small helpers ----------

    // Find a part by name; if it's missing (or the wrong type), remember the name to show later.
    private <T> T find(Class<T> type, String name) {
        T device = hardwareMap.tryGet(type, name);
        if (device == null) missing.append(name).append(" (").append(type.getSimpleName()).append(") ");
        return device;
    }

    // CHECK 4: show how far each side has moved since X was pressed, in ticks and inches.
    // Push the robot 24 inches by hand: each side should read about 1087 ticks (24.0 in).
    private void showEncoders() {
        int l = position(leftDrive) - leftZero, r = position(rightDrive) - rightZero;
        telemetry.addData("CHECK 4 encoders", "left %d (%.1f in) | right %d (%.1f in) | 24 in push = ~1087",
                l, l / TICKS_PER_INCH, r, r / TICKS_PER_INCH);
    }

    private void zeroEncoders() {
        leftZero = position(leftDrive);
        rightZero = position(rightDrive);
    }

    private int position(DcMotor motor) {
        return motor == null ? 0 : motor.getCurrentPosition();
    }

    private double yaw() {
        return imu == null ? 0 : imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    // The lowest voltage reported by any hub = the real battery level under load.
    private double batteryVoltage() {
        double lowest = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double v = sensor.getVoltage();
            if (v > 0) lowest = Math.min(lowest, v);
        }
        return lowest == Double.POSITIVE_INFINITY ? 0 : lowest;
    }
}
