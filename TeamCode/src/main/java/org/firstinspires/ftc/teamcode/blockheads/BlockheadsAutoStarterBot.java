package org.firstinspires.ftc.teamcode.blockheads;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * BH Auto: StarterBot. The rebuilt autonomous for the goBILDA 6WD StarterBot.
 * (AUTO = the first 30 seconds of a match, when the robot drives itself.)
 *
 * WHERE TO PUT THE ROBOT (red; blue is the same thing on the opposite side, see below):
 *   - Against the AUDIENCE wall, in column C (robot center about 17 in left of the middle line).
 *   - The INTAKE end touches the wall. The LAUNCHER end points at the HIVE.
 *   BLUE: against the FAR wall in column D (17 in right of the middle line), same way round.
 *
 * BEFORE START: press A on gamepad 1 to CONFIRM the robot is placed right (launcher pointing at
 * the HIVE). If nobody presses A, the robot ONLY backs 6 in off the wall (LEAVE, 3 pts) and does
 * nothing else. A robot placed backwards would otherwise shoot at the audience and drive the
 * whole path in the wrong direction (we saw this in the simulator).
 *
 * WHAT IT DOES:
 *   1. BACK OFF   - drive 6 in away from the wall, launcher end first (this earns LEAVE, 3 pts).
 *                   If the robot can't move away, it was probably placed the WRONG WAY ROUND
 *                   (pushing into the wall), so it stops everything instead of driving off wildly.
 *   2. AIM        - turn a few degrees so the launcher points at the opening of our upward
 *                   HIVE cell. It's a small turn (about 6 degrees) because we start almost lined up.
 *   3. SHOOT      - launcher spins up from START; feed the 4 balls only while it's at speed.
 *   4. SPILL WAIT - wait 1 s so balls falling out of a tipping HIVE land first (rule G409).
 *   5. PARK PATH  - turn and drive along our side: west 30 in, north to the LOADING ZONE,
 *                   then turn and slide into the middle of the LOADING ZONE (PARK, 5 pts).
 *                   It ends 6 in from the wall so it still counts as LEAVE (not touching the wall).
 *
 * WHY THE SAME CODE WORKS FOR RED AND BLUE: the BIOBUZZ field looks the same after a half-turn
 * (180 degrees), so every move and turn is identical. The alliance choice only changes the screen.
 *
 * Every move is measured from where the robot is NOW (wheel encoders for distance, the Control
 * Hub's gyro, called the IMU, for turning). Every step has a time limit so it can never get stuck.
 */
@Autonomous(name = "BH Auto: StarterBot", group = "Dev", preselectTeleOp = "BioBuzz StarterBot Teleop")
@Disabled // hidden on the real Driver Station until it has been tested on the robot
public class BlockheadsAutoStarterBot extends OpMode {

    // ================= Numbers to check on the real robot (see Bench-Check-Sheet.md) =================

    // Which end shoots? true = the launcher is at the BACK (the TeleOp drives intake-first as "forward").
    // TODO(robot): confirm with BH Bench Check, check 7.
    static final boolean LAUNCHER_ON_BACK = true;

    // How the Control Hub is mounted (only matters for the gyro). TODO(robot): check with SensorIMUOrthogonal.
    static final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    static final RevHubOrientationOnRobot.UsbFacingDirection HUB_USB = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    // Encoder ticks per inch of travel (537.7 ticks per wheel turn, 96 mm wheels): about 45.3.
    static final double TICKS_PER_INCH = StarterBotHardware.DRIVE_TICKS_PER_MM * 25.4;

    // ================= Field spots (inches, RED side; origin = field center) =================
    // +x points toward the BLUE wall, +y points away from the audience.
    static final double START_X = -17, START_Y = -63;        // robot center at the start
    static final double BACK_OFF_IN = 6;                     // how far to back away from the wall
    static final double OPENING_X = -12.75, OPENING_Y = -19.5; // center of our upward cell's opening
    static final double LANE_X = -47;                        // the "road" we drive north on, clear of the HIVE
    static final double PARK_X = -57;                        // robot center when parked (6 in off the side wall)
    // Where along the LOADING ZONE to park (pick in INIT with dpad left/right):
    static final String[] PARK_NAMES = {"MIDDLE", "HIVE END", "CORNER END"};
    static final double[] PARK_YS = {36, 20, 50};

    // ================= Speeds, accuracy and time limits =================
    static final double DRIVE_POWER = 0.5;        // normal driving speed (0 to 1)
    static final double SLOW_POWER = 0.25;        // speed for the last few inches of a move
    static final double SLOW_ZONE_IN = 8;         // start slowing down this many inches before the end
    static final double HOLD_KP = 0.03;           // how hard to steer back to straight while driving
    static final double HOLD_MAX = 0.2;
    static final double TURN_KP = 0.02;           // how hard to turn per degree still to go
    static final double TURN_MIN_POWER = 0.12;    // enough power to actually move (6 wheels scrub when turning)
    static final double TURN_MAX_POWER = 0.45;
    static final double TURN_TOLERANCE_DEG = 1.5; // close enough
    static final double SETTLE_S = 0.15;          // must stay close enough this long to count as done
    static final double MOVE_TIMEOUT_S = 6.0;     // any single drive gives up after this
    static final double TURN_TIMEOUT_S = 2.5;     // any single turn gives up after this
    static final double FEED_SECONDS = 4.0;       // feed time (only counted while the launcher is at speed)
    static final double SHOOT_TIMEOUT_S = 8.0;
    static final double SPILL_WAIT_S = 1.0;
    static final double PARK_LATEST_START_S = 15.0;  // if the park path can't start by now, skip it
    static final double SAFETY_STOP_S = 28.5;        // stop everything, whatever is happening

    // ================= The steps, in order =================
    private enum Step { BACK_OFF, AIM, SHOOT, SPILL_WAIT, PARK_WAIT, TURN_WEST, DRIVE_WEST, TURN_NORTH, DRIVE_NORTH, TURN_TO_WALL, SLIDE_IN, DONE }

    private final StarterBotHardware robot = new StarterBotHardware();
    private IMU imu = null;
    private final ElapsedTime autoTimer = new ElapsedTime();   // time since START
    private final ElapsedTime stepTimer = new ElapsedTime();   // time in the current step
    private final ElapsedTime settleTimer = new ElapsedTime();

    private Step step = Step.BACK_OFF;
    private String note = "";                 // shown on screen: why the auto stopped early, if it did
    private boolean blue = false;             // alliance (display only)
    private int parkChoice = 0;               // index into PARK_NAMES
    private double parkWaitS = 0;             // optional wait before driving to park
    private boolean prevLeft, prevRight, prevUp, prevDown;
    private boolean placementConfirmed = false; // set by pressing A during INIT

    private boolean launcherOn = false;
    private double fedSeconds = 0, lastLoopS = 0;
    private double aimDeg = 0;                // the aim turn we calculated (shown on screen)

    // For moves: where each wheel started, and how far we must go.
    private int leftStart, rightStart;
    private double moveInches, moveHoldYaw;
    private double lastProgressIn, lastProgressS; // for noticing when the robot is stuck
    private double turnTargetYaw;

    // Our best guess of where we are (RED frame, for the screen only).
    private double poseX = START_X, poseY = START_Y;
    private int lastLeftTicks, lastRightTicks;

    // INIT: set up the hardware. Nothing may move after INIT (rule G304.H).
    @Override
    public void init() {
        robot.init(hardwareMap);

        // The gyro (IMU) is needed for the turns. If it's missing, we only shoot and back off.
        try {
            imu = hardwareMap.get(IMU.class, "imu");
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(HUB_LOGO, HUB_USB)));
        } catch (Exception e) {
            imu = null;
        }
    }

    // init_loop runs while waiting for START: pick options and show where to put the robot.
    @Override
    public void init_loop() {
        // X = blue, B = red (the moves are the same either way).
        if (gamepad1.x) blue = true;
        if (gamepad1.b) blue = false;

        // A = "I checked: the launcher points at the HIVE." Without it we only back off the wall.
        if (gamepad1.a) placementConfirmed = true;

        // Dpad left/right: where in the LOADING ZONE to park. Dpad up/down: wait before parking.
        // (We compare with last time so one press = one change.)
        if (gamepad1.dpad_right && !prevRight) parkChoice = (parkChoice + 1) % PARK_NAMES.length;
        if (gamepad1.dpad_left && !prevLeft) parkChoice = (parkChoice + PARK_NAMES.length - 1) % PARK_NAMES.length;
        if (gamepad1.dpad_up && !prevUp) parkWaitS = Math.min(parkWaitS + 1, 6);
        if (gamepad1.dpad_down && !prevDown) parkWaitS = Math.max(parkWaitS - 1, 0);
        prevLeft = gamepad1.dpad_left; prevRight = gamepad1.dpad_right;
        prevUp = gamepad1.dpad_up; prevDown = gamepad1.dpad_down;

        telemetry.addData("Placement (press A to confirm)", placementConfirmed
                ? "CONFIRMED - full auto"
                : "NOT CONFIRMED - will ONLY back off the wall");
        telemetry.addData("Alliance (X blue / B red)", blue ? "BLUE" : "RED");
        telemetry.addData("Park spot (dpad L/R)", PARK_NAMES[parkChoice]);
        telemetry.addData("Wait before parking (dpad U/D)", "%.0f s", parkWaitS);
        telemetry.addData("Gyro", imu == null ? "MISSING - will only shoot + back off" : "OK");
        telemetry.addLine(blue
                ? "PLACE: FAR wall, column D (17 in right of middle). INTAKE touching the wall, LAUNCHER pointing at the HIVE."
                : "PLACE: AUDIENCE wall, column C (17 in left of middle). INTAKE touching the wall, LAUNCHER pointing at the HIVE.");
        telemetry.addLine("CHECK: 30 s timer ON, 4 balls loaded, nothing moving.");
    }

    // START: reset the clocks and the gyro, set the wheels to speed control, spin up the launcher.
    @Override
    public void start() {
        autoTimer.reset();
        if (imu != null) imu.resetYaw();     // the gyro now reads 0 = the way we started facing
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lastLeftTicks = robot.leftDrive.getCurrentPosition();
        lastRightTicks = robot.rightDrive.getCurrentPosition();

        // Work out the aim turn: from the backed-off spot, which way is the cell opening?
        // atan2 gives the angle; negative = turn right (clockwise).
        double fromY = START_Y + BACK_OFF_IN;
        aimDeg = -Math.toDegrees(Math.atan2(OPENING_X - START_X, OPENING_Y - fromY));

        // Not confirmed? Then don't spin the launcher; we'll only back off the wall.
        if (!placementConfirmed) note = "placement not confirmed (A): only backing off the wall";
        launcherOn = placementConfirmed;     // spin up now so it's ready when we finish aiming
        startMove(BACK_OFF_IN, 0);
        enter(Step.BACK_OFF);
    }

    // loop() runs about 50 times a second until AUTO ends.
    @Override
    public void loop() {
        double t = autoTimer.seconds();
        double windmillPower = 0, intakePower = 0;   // feeders start OFF every loop
        updatePose();

        // Safety net: near the end of AUTO, stop no matter what.
        if (step != Step.DONE && t >= SAFETY_STOP_S) { note = "safety stop at 28.5 s"; enter(Step.DONE); }

        switch (step) {
            case BACK_OFF:
                // If we haven't moved at least 1 inch after 0.6 s, we're pushing into the wall:
                // the robot was placed the wrong way round. Stop and don't shoot.
                if (stepTimer.seconds() > 0.6 && Math.abs(travelledIn()) < 1.0) {
                    note = "COULDN'T LEAVE THE WALL - robot placed the wrong way round? Stopped.";
                    enter(Step.DONE);
                } else if (driveStep()) {
                    if (!placementConfirmed) enter(Step.DONE);              // LEAVE only
                    else if (imu != null) { startTurn(aimDeg); enter(Step.AIM); }
                    else enter(Step.SHOOT);
                }
                break;

            case AIM:
                // Small turn so the launcher points at the cell opening.
                if (turnStep()) { stopDrive(); enter(Step.SHOOT); }
                break;

            case SHOOT:
                // Feed balls only while the launcher is fast enough; count only that feeding time.
                if (robot.launcher.getVelocity() >= StarterBotHardware.LAUNCHER_MIN_VELOCITY) {
                    windmillPower = StarterBotHardware.WINDMILL_FEED_POWER;
                    intakePower = StarterBotHardware.INTAKE_FEED_POWER;
                    fedSeconds += t - lastLoopS;
                }
                if (fedSeconds >= FEED_SECONDS || stepTimer.seconds() > SHOOT_TIMEOUT_S) {
                    launcherOn = false;
                    enter(Step.SPILL_WAIT);
                }
                break;

            case SPILL_WAIT:
                // Let any balls falling from the HIVE land before we move (rule G409).
                if (stepTimer.seconds() >= SPILL_WAIT_S) {
                    if (imu == null) { note = "no gyro: skipped park (LEAVE already earned)"; enter(Step.DONE); }
                    else enter(Step.PARK_WAIT);
                }
                break;

            case PARK_WAIT:
                // Optional wait (lets our partner get to the LOADING ZONE first), then check the clock.
                if (stepTimer.seconds() >= parkWaitS) {
                    if (t > PARK_LATEST_START_S) { note = "too late to park safely; staying put"; enter(Step.DONE); }
                    else { startTurn(90); enter(Step.TURN_WEST); }   // +90 = launcher points west
                }
                break;

            case TURN_WEST:
                if (turnStep()) { startMove(START_X - LANE_X, 90); enter(Step.DRIVE_WEST); }
                break;

            case DRIVE_WEST:
                // Launcher-first along the wall to the north-south "road" (LANE_X).
                if (driveStep()) { startTurn(0); enter(Step.TURN_NORTH); }
                break;

            case TURN_NORTH:
                if (turnStep()) { startMove(PARK_YS[parkChoice] - (START_Y + BACK_OFF_IN), 0); enter(Step.DRIVE_NORTH); }
                break;

            case DRIVE_NORTH:
                // Up the road, beside the HIVE, to the LOADING ZONE.
                if (driveStep()) { startTurn(-90); enter(Step.TURN_TO_WALL); }
                break;

            case TURN_TO_WALL:
                // Turn so the INTAKE end faces our side wall.
                if (turnStep()) { startMove(-(LANE_X - PARK_X), -90); enter(Step.SLIDE_IN); }
                break;

            case SLIDE_IN:
                // Intake-first into the LOADING ZONE, stopping 6 in from the wall.
                if (driveStep()) { note = "parked"; enter(Step.DONE); }
                break;

            default:
                break;   // DONE: everything is off
        }

        // Send the motor commands for this loop (launcher speed and feeders).
        if (step != Step.DONE) {
            robot.launcher.setVelocity(launcherOn ? StarterBotHardware.LAUNCHER_TARGET_VELOCITY : 0);
            robot.windmill.setPower(windmillPower);
            robot.intake.setPower(intakePower);
        }
        lastLoopS = t;

        // Show what's happening on the Driver Station.
        telemetry.addData("Step", step);
        telemetry.addData("AUTO time", "%.1f s", t);
        telemetry.addData("Aim turn", "%.1f deg (%s)", aimDeg, aimDeg < 0 ? "right" : "left");
        telemetry.addData("Launcher", "%.0f ticks/s, fed %.1f s", robot.launcher.getVelocity(), fedSeconds);
        telemetry.addData("Gyro yaw", imu == null ? "no gyro" : String.format("%.1f deg", yaw()));
        double shownX = blue ? -poseX : poseX, shownY = blue ? -poseY : poseY;
        telemetry.addData("Where we think we are", "x %.1f  y %.1f", shownX, shownY);
        if (!note.isEmpty()) telemetry.addData("Note", note);
    }

    // AUTO is over (timer ran out or STOP pressed): make sure everything is off.
    @Override
    public void stop() {
        robot.stopAll();
    }

    // ================= Helpers =================

    // Move on to a new step and restart its timer.
    private void enter(Step next) {
        step = next;
        stepTimer.reset();
        settleTimer.reset();
        if (next == Step.DONE) {
            robot.stopAll();
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  // no "holding" in the transition (G403)
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    // ---- Driving straight ----
    // inches > 0 = launcher end first; inches < 0 = intake end first.
    // holdYaw = the gyro heading to keep while driving (so we go straight).
    private void startMove(double inches, double holdYaw) {
        leftStart = robot.leftDrive.getCurrentPosition();
        rightStart = robot.rightDrive.getCurrentPosition();
        moveInches = inches;
        moveHoldYaw = holdYaw;
        lastProgressIn = 0;
        lastProgressS = autoTimer.seconds();
    }

    // How far we've gone in this move, in inches (+ = launcher first). Average of both wheels.
    private double travelledIn() {
        int dl = robot.leftDrive.getCurrentPosition() - leftStart;
        int dr = robot.rightDrive.getCurrentPosition() - rightStart;
        double forwardIn = (dl + dr) / 2.0 / TICKS_PER_INCH;     // + = TeleOp "forward" (intake first)
        return LAUNCHER_ON_BACK ? -forwardIn : forwardIn;
    }

    // Run one loop of a straight move. Returns true when finished (arrived, stuck, or timed out).
    private boolean driveStep() {
        double done = travelledIn();
        double left = Math.abs(moveInches) - Math.abs(done);
        if (left <= 0.25) { stopDrive(); return true; }                 // arrived

        // Stuck check: if we haven't gained half an inch in 0.6 s, stop this move.
        if (Math.abs(done) - lastProgressIn > 0.5) { lastProgressIn = Math.abs(done); lastProgressS = autoTimer.seconds(); }
        if (autoTimer.seconds() - lastProgressS > 0.6 && stepTimer.seconds() > 0.6) {
            note = "stopped a move early: robot seems blocked (" + step + ")";
            stopDrive(); return true;
        }
        if (stepTimer.seconds() > MOVE_TIMEOUT_S) { note = "move timed out (" + step + ")"; stopDrive(); return true; }

        // Speed: normal, then slow near the end. Direction: launcher-first or intake-first.
        double power = left < SLOW_ZONE_IN ? SLOW_POWER : DRIVE_POWER;
        double sign = Math.signum(moveInches) * (LAUNCHER_ON_BACK ? -1 : 1);   // motor direction
        // Steer back to straight using the gyro (more power on one side turns the robot).
        double correction = 0;
        if (imu != null) correction = Range.clip(HOLD_KP * angleDiff(moveHoldYaw, yaw()), -HOLD_MAX, HOLD_MAX);
        setDrive(sign * power - correction, sign * power + correction);
        return false;
    }

    // ---- Turning in place with the gyro ----
    // targetYaw is measured from the START heading: + = left (counter-clockwise), - = right.
    private void startTurn(double targetYaw) {
        turnTargetYaw = targetYaw;
    }

    // Run one loop of a turn. Returns true when finished (on target, or timed out).
    private boolean turnStep() {
        double error = angleDiff(turnTargetYaw, yaw());          // degrees still to turn
        if (Math.abs(error) > TURN_TOLERANCE_DEG) settleTimer.reset();
        if (settleTimer.seconds() >= SETTLE_S) { stopDrive(); return true; }
        if (stepTimer.seconds() > TURN_TIMEOUT_S) {
            stopDrive();
            // Way off target = something is wrong (blocked?). Stop the whole auto rather than guess.
            if (Math.abs(error) > 10) { note = "turn failed (" + step + "), stopped"; enter(Step.DONE); return false; }
            return true;
        }
        // Turn power: bigger when far away, never below the minimum that actually moves the robot.
        double power = Range.clip(TURN_KP * error, -TURN_MAX_POWER, TURN_MAX_POWER);
        if (Math.abs(error) > TURN_TOLERANCE_DEG && Math.abs(power) < TURN_MIN_POWER) power = Math.signum(error) * TURN_MIN_POWER;
        if (Math.abs(error) <= TURN_TOLERANCE_DEG) power = 0;
        setDrive(-power, power);                                   // left back, right forward = turn left
        return false;
    }

    private void setDrive(double leftPower, double rightPower) {
        robot.leftDrive.setPower(leftPower);
        robot.rightDrive.setPower(rightPower);
    }

    private void stopDrive() {
        setDrive(0, 0);
    }

    // The gyro heading in degrees since START (0 = start heading, + = turned left).
    private double yaw() {
        return imu == null ? 0 : imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    // Smallest difference between two headings (-180..180), e.g. from 170 to -170 is +20, not -340.
    private double angleDiff(double target, double current) {
        return AngleUnit.normalizeDegrees(target - current);
    }

    // Update our position guess from the wheels and the gyro (for the screen only).
    // At yaw 0 the launcher points along +y (toward the HIVE), so launcher-first motion is
    // toward the angle 90 + yaw degrees.
    private void updatePose() {
        int l = robot.leftDrive.getCurrentPosition(), r = robot.rightDrive.getCurrentPosition();
        double forwardIn = ((l - lastLeftTicks) + (r - lastRightTicks)) / 2.0 / TICKS_PER_INCH;
        double launcherFirstIn = LAUNCHER_ON_BACK ? -forwardIn : forwardIn;
        double angle = Math.toRadians(90 + yaw());
        poseX += launcherFirstIn * Math.cos(angle);
        poseY += launcherFirstIn * Math.sin(angle);
        lastLeftTicks = l;
        lastRightTicks = r;
    }
}
