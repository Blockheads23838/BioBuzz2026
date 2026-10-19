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
 * BH Auto: StarterBot (version 3, set up for OUR robot: drive motors reversed, hub on its side). The autonomous for the goBILDA 6WD StarterBot.
 * (AUTO = the first 30 seconds of a match, when the robot drives itself.)
 *
 * WHERE TO PUT THE ROBOT (red; blue is the same thing on the opposite side, see below):
 *   - Against the AUDIENCE wall, in column C (robot center about 17 in left of the middle line).
 *   - The INTAKE end touches the wall. The LAUNCHER end points at the HIVE.
 *   BLUE: against the FAR wall in column D (17 in right of the middle line), same way round.
 *
 * BEFORE START: press A on gamepad 1 to CONFIRM the robot is placed right (Y cancels).
 * Without A, the robot ONLY backs off the wall (LEAVE, 3 pts) and does nothing else.
 *
 * WHAT IT DOES:
 *   0. LINE UP    - push gently back against the wall (under a second) so a slightly crooked
 *                   placement straightens out. On the real robot it also checks that the wall is
 *                   really behind the intake: if the robot rolls freely instead, it was placed the
 *                   wrong way round, so it stops (no shooting).
 *   1. BACK OFF   - drive 7 in away from the wall, launcher end first (LEAVE, 3 pts).
 *   2. AIM        - small turn (about 6 degrees) so the launcher points at the cell opening.
 *   3. SHOOT      - feed the 4 balls while the launcher is at speed. The robot STANDS STILL
 *                   for about 5 s here (the simulator has no balls: watch the "NOW:" line).
 *   4. SPILL WAIT - wait 1 s so balls falling out of a tipping HIVE land first (rule G409).
 *   5. PARK PATH  - turn and drive along our side: toward our side wall, north past the HIVE,
 *                   then slide into the LOADING ZONE, ending 6 in from the wall (PARK, 5 pts).
 *
 * SAFETY RULE OF THIS PROGRAM: if anything goes wrong (a move gets blocked, a turn fails, the
 * gyro doesn't respond), the robot STOPS where it is. It never "carries on" from a wrong spot,
 * because driving on from the wrong place could hit the HIVE or cross to the other side.
 *
 * WHY THE SAME CODE WORKS FOR RED AND BLUE: the field looks the same after a half-turn, so every
 * move and turn is identical. The alliance choice only changes what's shown on the screen.
 */
@Autonomous(name = "BH Auto: StarterBot", group = "Dev", preselectTeleOp = "BioBuzz StarterBot Teleop")
@Disabled // hidden on the real Driver Station until it has been tested on the robot
public class BlockheadsAutoStarterBot extends OpMode {

    // ================= Numbers to check on the real robot (see Bench-Check-Sheet.md) =================

    // Which end shoots? true = launcher at the BACK (the TeleOp drives intake-first as "forward").
    // TODO(robot): confirm with BH Bench Check, check 7.
    static final boolean LAUNCHER_ON_BACK = true;

    // How the Control Hub is mounted (for the gyro) and the drive motor directions both live in
    // StarterBotHardware (HUB_LOGO, HUB_USB, DRIVE_MOTORS_REVERSED), so every program agrees.
    static final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO = StarterBotHardware.HUB_LOGO;
    static final RevHubOrientationOnRobot.UsbFacingDirection HUB_USB = StarterBotHardware.HUB_USB;

    // Encoder ticks per inch of travel (537.7 ticks per wheel turn, 96 mm wheels): about 45.3.
    // TODO(robot): push the robot 48 in and check the real number.
    static final double TICKS_PER_INCH = StarterBotHardware.DRIVE_TICKS_PER_MM * 25.4;

    // ================= Field spots (inches, RED side; origin = field center) =================
    // +x points toward the BLUE wall, +y points away from the audience.
    static final double START_X = -17, START_Y = -63;          // robot center at the start
    static final double BACK_OFF_IN = 7;                       // how far to back away from the wall
    static final double OPENING_X = -12.75, OPENING_Y = -19.5; // center of our upward cell's opening
    static final double LANE_X = -47;                          // the "road" north, clear of the HIVE
    static final double PARK_X = -57;                          // robot center when parked (6 in off the side wall)
    // Where along the LOADING ZONE to park (pick in INIT with dpad left/right):
    static final String[] PARK_NAMES = {"MIDDLE", "HIVE END", "CORNER END"};
    static final double[] PARK_YS = {36, 20, 50};

    // ================= Speeds, accuracy and time limits =================
    static final double DRIVE_POWER = 0.5;        // normal driving speed (0 to 1)
    static final double SLOW_POWER = 0.25;        // speed for the last few inches of a move
    static final double SLOW_ZONE_IN = 8;         // start slowing this many inches before the end
    static final double HOLD_KP = 0.03;           // how hard to steer back to straight while driving
    static final double HOLD_MAX = 0.2;
    static final double TURN_KP = 0.02;           // how hard to turn per degree still to go
    static final double TURN_MIN_POWER = 0.15;    // enough power to actually turn (6 wheels scrub)
    static final double TURN_MAX_POWER = 0.45;
    static final double TURN_TOLERANCE_DEG = 1.5; // close enough
    static final double TURN_GIVE_UP_DEG = 15;    // after the turn time limit, more than this off = stop
    static final double SETTLE_S = 0.15;          // must stay on target this long to count as done
    static final double MOVE_TIMEOUT_S = 6.0;     // any single drive gives up after this
    static final double TURN_TIMEOUT_S = 2.5;     // any single turn gives up after this
    static final double LINE_UP_POWER = 0.2;      // gentle push back against the wall
    static final double LINE_UP_S = 0.6;          // how long to push
    static final double LINE_UP_SETTLE_S = 0.25;  // sit still this long before resetting the gyro
    static final double LINE_UP_MAX_ROLL_IN = 4;  // rolling further than this = no wall behind us
    static final double FEED_SECONDS = 4.0;       // feed time (only counted while the launcher is at speed)
    static final double FEED_LOWER_AFTER_S = 3.0; // if never at full speed by then, accept 90% speed
    static final double SHOOT_TIMEOUT_S = 8.0;
    static final double SPILL_WAIT_S = 1.0;
    static final double MAX_PARK_WAIT_S = 4;      // longest wait you can pick before parking
    static final double PARK_LATEST_START_S = 15.0;  // the park path must start by now
    static final double SAFETY_STOP_S = 28.5;        // stop everything, whatever is happening

    // In the simulator, wheels pushed against a wall just spin (real wheels at low power stall),
    // so the "is the wall really behind us?" check can't work there. We detect the simulator by
    // looking for one of its classes; on the real robot that class doesn't exist.
    static final boolean IN_SIMULATOR = classExists("virtual_robot.controller.VirtualRobotController");

    // ================= The steps, in order =================
    private enum Step { LINE_UP, BACK_OFF, AIM, SHOOT, SPILL_WAIT, PARK_WAIT, TURN_WEST, DRIVE_WEST, TURN_NORTH, DRIVE_NORTH, TURN_TO_WALL, SLIDE_IN, DONE }

    // What a drive or turn reports each loop.
    private enum Result { WORKING, ARRIVED, FAILED }

    private final StarterBotHardware robot = new StarterBotHardware();
    private boolean hardwareReady = false;     // true once robot.init() worked
    private IMU imu = null;
    private final ElapsedTime autoTimer = new ElapsedTime();   // time since START
    private final ElapsedTime stepTimer = new ElapsedTime();   // time in the current step
    private final ElapsedTime settleTimer = new ElapsedTime();

    private Step step = Step.LINE_UP;
    private String result = "";               // final outcome, shown at the end
    private String warnings = "";             // problems noticed along the way (all kept)
    private boolean blue = false;             // alliance (display only)
    private int parkChoice = 0;               // index into PARK_NAMES
    private double parkWaitS = 0;             // wait picked in INIT
    private double waitThisTime = 0;          // the wait actually used (may be shortened)
    private boolean prevLeft, prevRight, prevUp, prevDown, prevA;
    private boolean placementConfirmed = false;

    private boolean launcherOn = false;
    private double fedSeconds = 0, lastLoopS = 0;
    private double aimDeg = 0;

    // For moves: where each wheel started, and how far we must go.
    private int leftStart, rightStart;
    private double moveInches, moveHoldYaw;
    private double bestProgressIn, bestProgressS;  // for noticing when the robot is stuck
    // For turns:
    private double turnTargetYaw, turnStartYaw, turnStartError;

    // Our best guess of where we are (RED frame, for the screen only).
    private double poseX = START_X, poseY = START_Y;
    private int lastLeftTicks, lastRightTicks;

    // INIT: set up the hardware. Nothing may move after INIT (rule G304.H).
    @Override
    public void init() {
        robot.init(hardwareMap);   // if a device name is wrong, this stops here with a clear message
        hardwareReady = true;

        // The gyro (IMU) is needed for the turns. If it's missing or won't start, we only shoot + back off.
        try {
            imu = hardwareMap.get(IMU.class, "imu");
            if (!imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(HUB_LOGO, HUB_USB)))) imu = null;
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

        // A = "I checked: the launcher points at the HIVE." Y = cancel that.
        // We only count a fresh press of A WITHOUT Start held, because Start+A is how a gamepad
        // gets connected to the Driver Station, and that must not confirm by accident.
        boolean aPressed = gamepad1.a && !gamepad1.start;
        if (aPressed && !prevA) placementConfirmed = true;
        if (gamepad1.y) placementConfirmed = false;
        prevA = aPressed;

        // Dpad left/right: where in the LOADING ZONE to park. Dpad up/down: wait before parking.
        // (We compare with last time so one press = one change.)
        if (gamepad1.dpad_right && !prevRight) parkChoice = (parkChoice + 1) % PARK_NAMES.length;
        if (gamepad1.dpad_left && !prevLeft) parkChoice = (parkChoice + PARK_NAMES.length - 1) % PARK_NAMES.length;
        if (gamepad1.dpad_up && !prevUp) parkWaitS = Math.min(parkWaitS + 1, MAX_PARK_WAIT_S);
        if (gamepad1.dpad_down && !prevDown) parkWaitS = Math.max(parkWaitS - 1, 0);
        prevLeft = gamepad1.dpad_left; prevRight = gamepad1.dpad_right;
        prevUp = gamepad1.dpad_up; prevDown = gamepad1.dpad_down;

        telemetry.addData("Placement (A confirm, Y cancel)", placementConfirmed
                ? "CONFIRMED - full auto"
                : "NOT CONFIRMED - will ONLY back off the wall");
        telemetry.addData("Alliance (X blue / B red)", blue ? "BLUE" : "RED");
        telemetry.addData("Park spot (dpad L/R)", PARK_NAMES[parkChoice]);
        telemetry.addData("Wait before parking (dpad U/D)", "%.0f s (max %.0f)", parkWaitS, MAX_PARK_WAIT_S);
        telemetry.addData("Gyro", imu == null ? "MISSING - will only shoot + back off" : String.format("OK (yaw %.1f)", yaw()));
        if (IN_SIMULATOR) telemetry.addLine("Simulator: wall check skipped (sim wheels slip against walls).");
        telemetry.addLine(blue
                ? "PLACE: FAR wall, column D (17 in right of middle). INTAKE touching the wall, LAUNCHER pointing at the HIVE."
                : "PLACE: AUDIENCE wall, column C (17 in left of middle). INTAKE touching the wall, LAUNCHER pointing at the HIVE.");
        telemetry.addLine("CHECK: 30 s timer ON, 4 balls loaded, nothing moving.");
    }

    // START: reset the clock, spin up the launcher (if confirmed), and begin.
    @Override
    public void start() {
        autoTimer.reset();

        // Work out the aim turn: from the backed-off spot, which way is the cell opening?
        // atan2 gives the angle; negative = turn right (clockwise).
        double fromY = START_Y + BACK_OFF_IN;
        aimDeg = -Math.toDegrees(Math.atan2(OPENING_X - START_X, OPENING_Y - fromY));

        if (!placementConfirmed) {
            // Not confirmed: no launcher, just leave the wall.
            setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (imu != null) imu.resetYaw();
            resetPoseHere();
            startMove(BACK_OFF_IN, 0);
            enter(Step.BACK_OFF);
            return;
        }
        launcherOn = true;                    // spin up now so it's ready when we finish aiming
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);   // plain power for the gentle push
        leftStart = robot.leftDrive.getCurrentPosition();
        rightStart = robot.rightDrive.getCurrentPosition();
        enter(Step.LINE_UP);
    }

    // loop() runs about 50 times a second until AUTO ends.
    @Override
    public void loop() {
        double t = autoTimer.seconds();
        double windmillPower = 0, intakePower = 0;   // feeders start OFF every loop
        updatePose();

        // Safety net: near the end of AUTO, stop no matter what.
        if (step != Step.DONE && t >= SAFETY_STOP_S) finish("safety stop at 28.5 s");

        switch (step) {
            case LINE_UP:
                // Push gently INTAKE-first into the wall, then stop and let the robot settle.
                if (stepTimer.seconds() < LINE_UP_S) {
                    double intakeFirst = LAUNCHER_ON_BACK ? LINE_UP_POWER : -LINE_UP_POWER;
                    setDrive(intakeFirst, intakeFirst);
                } else if (stepTimer.seconds() < LINE_UP_S + LINE_UP_SETTLE_S) {
                    stopDrive();
                } else {
                    // Did we roll freely? Against a wall the wheels can't turn much. If they rolled
                    // more than a few inches, there's no wall behind the intake: placed the wrong
                    // way round (or far from the wall). Stop without shooting.
                    double rolledIn = Math.abs(travelledFromStartIn());
                    if (!IN_SIMULATOR && rolledIn > LINE_UP_MAX_ROLL_IN) {
                        launcherOn = false;
                        finish(String.format("STOPPED: rolled %.0f in while lining up - no wall behind the intake. Placed the wrong way round?", rolledIn));
                        break;
                    }
                    // Flat on the wall now: this is our true starting heading and spot.
                    setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    if (imu != null) imu.resetYaw();
                    resetPoseHere();
                    startMove(BACK_OFF_IN, 0);
                    enter(Step.BACK_OFF);
                }
                break;

            case BACK_OFF: {
                Result r = driveStep();
                if (r == Result.FAILED) { launcherOn = false; finish("STOPPED: couldn't back away from the wall (blocked or wrong way round)"); }
                else if (r == Result.ARRIVED) {
                    if (!placementConfirmed) finish("placement not confirmed (A): backed off the wall only (LEAVE)");
                    else if (imu != null) { startTurn(aimDeg); enter(Step.AIM); }
                    else enter(Step.SHOOT);
                }
                break;
            }

            case AIM: {
                // Small turn so the launcher points at the cell opening.
                Result r = turnStep();
                if (r == Result.FAILED) { launcherOn = false; finish("STOPPED during aim: " + lastTurnProblem + " (LEAVE already earned)"); }
                else if (r == Result.ARRIVED) enter(Step.SHOOT);
                break;
            }

            case SHOOT: {
                // Feed balls only while the launcher is fast enough; count only that feeding time.
                // If it never reaches full speed (low battery), accept 90% speed after 3 s.
                double needed = stepTimer.seconds() < FEED_LOWER_AFTER_S
                        ? StarterBotHardware.LAUNCHER_MIN_VELOCITY
                        : 0.9 * StarterBotHardware.LAUNCHER_TARGET_VELOCITY;
                if (robot.launcher.getVelocity() >= needed) {
                    windmillPower = StarterBotHardware.WINDMILL_FEED_POWER;
                    intakePower = StarterBotHardware.INTAKE_FEED_POWER;
                    fedSeconds += t - lastLoopS;
                }
                if (fedSeconds >= FEED_SECONDS) { launcherOn = false; enter(Step.SPILL_WAIT); }
                else if (stepTimer.seconds() > SHOOT_TIMEOUT_S) {
                    warn(String.format("launcher only fed %.1f of %.1f s (never fast enough?)", fedSeconds, FEED_SECONDS));
                    launcherOn = false;
                    enter(Step.SPILL_WAIT);
                }
                break;
            }

            case SPILL_WAIT:
                // Let any balls falling from the HIVE land before we move (rule G409).
                if (stepTimer.seconds() >= SPILL_WAIT_S) {
                    if (imu == null) { finish("no gyro: shot and backed off only (LEAVE)"); break; }
                    // Use the picked wait, but shorten it if it would make us too late to park.
                    // (0.3 s spare so a shortened wait ends safely before the cutoff.)
                    waitThisTime = Math.min(parkWaitS, Math.max(0, PARK_LATEST_START_S - 0.3 - t));
                    if (waitThisTime < parkWaitS) warn(String.format("park wait shortened to %.1f s so we can still park", waitThisTime));
                    enter(Step.PARK_WAIT);
                }
                break;

            case PARK_WAIT:
                if (stepTimer.seconds() >= waitThisTime) {
                    if (t > PARK_LATEST_START_S) finish("too late to park safely; stayed put (LEAVE)");
                    else { startTurn(90); enter(Step.TURN_WEST); }   // +90 = launcher points west
                }
                break;

            case TURN_WEST: {
                Result r = turnStep();
                if (r == Result.FAILED) finish("STOPPED turning toward the side wall: " + lastTurnProblem);
                else if (r == Result.ARRIVED) { startMove(START_X - LANE_X, 90); enter(Step.DRIVE_WEST); }
                break;
            }

            case DRIVE_WEST: {
                // Launcher-first toward our side wall, to the north-south "road" (LANE_X).
                // If this gets blocked, we must NOT turn north from here: we'd hit the HIVE.
                Result r = driveStep();
                if (r == Result.FAILED) finish("STOPPED: blocked on the way to the side wall (not safe to go north from here)");
                else if (r == Result.ARRIVED) { startTurn(0); enter(Step.TURN_NORTH); }
                break;
            }

            case TURN_NORTH: {
                Result r = turnStep();
                if (r == Result.FAILED) finish("STOPPED turning north: " + lastTurnProblem);
                else if (r == Result.ARRIVED) { startMove(PARK_YS[parkChoice] - (START_Y + BACK_OFF_IN), 0); enter(Step.DRIVE_NORTH); }
                break;
            }

            case DRIVE_NORTH: {
                // Up the road, beside the HIVE, to the LOADING ZONE.
                Result r = driveStep();
                if (r == Result.FAILED) finish("STOPPED: blocked driving past the HIVE");
                else if (r == Result.ARRIVED) { startTurn(-90); enter(Step.TURN_TO_WALL); }
                break;
            }

            case TURN_TO_WALL: {
                // Turn so the INTAKE end faces our side wall.
                Result r = turnStep();
                if (r == Result.FAILED) finish("STOPPED turning toward the LOADING ZONE: " + lastTurnProblem);
                else if (r == Result.ARRIVED) { startMove(-(LANE_X - PARK_X), -90); enter(Step.SLIDE_IN); }
                break;
            }

            case SLIDE_IN: {
                // Intake-first into the LOADING ZONE, stopping 6 in from the wall.
                Result r = driveStep();
                if (r == Result.FAILED) finish("STOPPED: blocked sliding into the LOADING ZONE (may not count as PARK)");
                else if (r == Result.ARRIVED) finish("PARKED in the LOADING ZONE");
                break;
            }

            default:
                break;   // DONE: everything is off
        }

        // Send the launcher and feeder commands for this loop.
        if (step != Step.DONE) {
            robot.launcher.setVelocity(launcherOn ? StarterBotHardware.LAUNCHER_TARGET_VELOCITY : 0);
            robot.windmill.setPower(windmillPower);
            robot.intake.setPower(intakePower);
        }
        lastLoopS = t;

        // Show what's happening on the Driver Station.
        telemetry.addData("NOW", nowText());
        telemetry.addData("AUTO time", "%.1f s", t);
        telemetry.addData("Launcher", "%.0f ticks/s (needs %.0f), fed %.1f s",
                robot.launcher.getVelocity(), StarterBotHardware.LAUNCHER_MIN_VELOCITY, fedSeconds);
        telemetry.addData("Gyro yaw", imu == null ? "no gyro" : String.format("%.1f deg", yaw()));
        double shownX = blue ? -poseX : poseX, shownY = blue ? -poseY : poseY;
        telemetry.addData("Where we think we are", "x %.1f  y %.1f", shownX, shownY);
        if (!result.isEmpty()) telemetry.addData("Result", result);
        if (!warnings.isEmpty()) telemetry.addData("Warnings", warnings);
    }

    // AUTO is over (timer ran out or STOP pressed): make sure everything is off.
    @Override
    public void stop() {
        if (hardwareReady) robot.stopAll();   // skip if init failed (devices may be missing)
    }

    // ================= Helpers =================

    // A plain-words description of the current step for the screen.
    private String nowText() {
        switch (step) {
            case LINE_UP:      return "lining up against the wall";
            case BACK_OFF:     return "backing off the wall";
            case AIM:          return String.format("aiming: small turn of %.1f deg", aimDeg);
            case SHOOT:        return String.format("SHOOTING - standing still while feeding balls (%.1f of %.1f s)", fedSeconds, FEED_SECONDS);
            case SPILL_WAIT:   return "waiting 1 s for balls to land";
            case PARK_WAIT:    return String.format("waiting %.0f s before parking", waitThisTime);
            case TURN_WEST:    return "turning toward our side wall";
            case DRIVE_WEST:   return "driving toward our side wall";
            case TURN_NORTH:   return "turning to drive past the HIVE";
            case DRIVE_NORTH:  return "driving past the HIVE to the LOADING ZONE";
            case TURN_TO_WALL: return "turning intake toward the side wall";
            case SLIDE_IN:     return "sliding into the LOADING ZONE";
            default:           return "done";
        }
    }

    // Move on to a new step and restart its timers.
    private void enter(Step next) {
        step = next;
        stepTimer.reset();
        settleTimer.reset();
    }

    // End the auto: everything off, wheels back to plain power mode (no "holding" in the
    // AUTO-to-TELEOP transition, rule G403), and remember why we ended.
    private void finish(String why) {
        result = why;
        robot.stopAll();
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        enter(Step.DONE);
    }

    // Remember a problem without stopping (all warnings are kept and shown).
    private void warn(String problem) {
        warnings = warnings.isEmpty() ? problem : warnings + " | " + problem;
    }

    // ---- Driving straight ----
    // inches > 0 = launcher end first; inches < 0 = intake end first.
    // holdYaw = the gyro heading to keep while driving (so we go straight).
    private void startMove(double inches, double holdYaw) {
        leftStart = robot.leftDrive.getCurrentPosition();
        rightStart = robot.rightDrive.getCurrentPosition();
        moveInches = inches;
        moveHoldYaw = holdYaw;
        bestProgressIn = 0;
        bestProgressS = autoTimer.seconds();
    }

    // How far we've gone since startMove, in inches (+ = launcher first). Average of both wheels.
    private double travelledFromStartIn() {
        int dl = robot.leftDrive.getCurrentPosition() - leftStart;
        int dr = robot.rightDrive.getCurrentPosition() - rightStart;
        double forwardIn = (dl + dr) / 2.0 / TICKS_PER_INCH;     // + = TeleOp "forward" (intake first)
        return LAUNCHER_ON_BACK ? -forwardIn : forwardIn;
    }

    // Run one loop of a straight move.
    // ARRIVED = got there. FAILED = blocked, going the wrong way, or out of time.
    private Result driveStep() {
        // "progress" counts only movement in the direction we asked for (wrong-way = negative).
        double progress = Math.signum(moveInches) * travelledFromStartIn();
        double toGo = Math.abs(moveInches) - progress;
        if (toGo <= 0.25) { stopDrive(); return Result.ARRIVED; }

        // Wrong direction by more than an inch = something is badly wrong. Stop.
        if (progress < -1.0) { stopDrive(); warn("moved the WRONG way during " + step); return Result.FAILED; }
        // Stuck: no new half-inch of progress for 0.6 s.
        if (progress - bestProgressIn > 0.5) { bestProgressIn = progress; bestProgressS = autoTimer.seconds(); }
        if (autoTimer.seconds() - bestProgressS > 0.6) {
            stopDrive();
            warn(String.format("blocked during %s after %.1f of %.1f in", step, progress, Math.abs(moveInches)));
            return Result.FAILED;
        }
        if (stepTimer.seconds() > MOVE_TIMEOUT_S) {
            stopDrive();
            warn(String.format("%s ran out of time after %.1f of %.1f in", step, progress, Math.abs(moveInches)));
            return Result.FAILED;
        }

        // Speed: normal, then slow near the end. Direction: launcher-first or intake-first.
        double power = toGo < SLOW_ZONE_IN ? SLOW_POWER : DRIVE_POWER;
        double sign = Math.signum(moveInches) * (LAUNCHER_ON_BACK ? -1 : 1);   // motor direction
        // Steer back to straight using the gyro (more power on one side turns the robot).
        double correction = 0;
        if (imu != null) correction = Range.clip(HOLD_KP * angleDiff(moveHoldYaw, yaw()), -HOLD_MAX, HOLD_MAX);
        setDrive(sign * power - correction, sign * power + correction);
        return Result.WORKING;
    }

    // ---- Turning in place with the gyro ----
    // targetYaw is measured from the START heading: + = left (counter-clockwise), - = right.
    private String lastTurnProblem = "";

    private void startTurn(double targetYaw) {
        turnTargetYaw = targetYaw;
        turnStartYaw = yaw();
        turnStartError = angleDiff(targetYaw, turnStartYaw);
    }

    // Run one loop of a turn. ARRIVED = on target. FAILED = the gyro or the turn isn't working.
    private Result turnStep() {
        double error = angleDiff(turnTargetYaw, yaw());          // degrees still to turn
        double s = stepTimer.seconds();

        // Gyro check 1: after 0.6 s of turning, the heading should have changed by a few degrees.
        // If it didn't, the gyro isn't working (or the robot is stuck). Don't guess - stop.
        if (s > 0.6 && Math.abs(turnStartError) > 3 && Math.abs(angleDiff(yaw(), turnStartYaw)) < 2) {
            stopDrive(); lastTurnProblem = "gyro heading didn't change (gyro setup? robot stuck?)"; return Result.FAILED;
        }
        // Gyro check 2: if the error is getting much BIGGER, we're turning the wrong way
        // (usually the hub-mounting setting is wrong). Stop.
        if (Math.abs(error) > Math.abs(turnStartError) + 10) {
            stopDrive(); lastTurnProblem = "turning the WRONG way (check HUB_LOGO / HUB_USB)"; return Result.FAILED;
        }

        if (Math.abs(error) > TURN_TOLERANCE_DEG) settleTimer.reset();
        if (settleTimer.seconds() >= SETTLE_S) { stopDrive(); return Result.ARRIVED; }
        if (s > TURN_TIMEOUT_S) {
            stopDrive();
            if (Math.abs(error) > TURN_GIVE_UP_DEG) { lastTurnProblem = String.format("turn ended %.0f deg off", error); return Result.FAILED; }
            warn(String.format("%s ended %.1f deg off (close enough, carried on)", step, error));
            return Result.ARRIVED;
        }
        // Turn power: bigger when far away, never below the minimum that actually turns the robot.
        double power = Range.clip(TURN_KP * error, -TURN_MAX_POWER, TURN_MAX_POWER);
        if (Math.abs(error) > TURN_TOLERANCE_DEG && Math.abs(power) < TURN_MIN_POWER) power = Math.signum(error) * TURN_MIN_POWER;
        if (Math.abs(error) <= TURN_TOLERANCE_DEG) power = 0;
        setDrive(-power, power);                                   // left back, right forward = turn left
        return Result.WORKING;
    }

    private void setDrive(double leftPower, double rightPower) {
        robot.leftDrive.setPower(leftPower);
        robot.rightDrive.setPower(rightPower);
    }

    private void stopDrive() {
        setDrive(0, 0);
    }

    private void setDriveMode(DcMotor.RunMode mode) {
        robot.leftDrive.setMode(mode);
        robot.rightDrive.setMode(mode);
    }

    // The gyro heading in degrees since the last reset (0 = start heading, + = turned left).
    private double yaw() {
        return imu == null ? 0 : imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    // Smallest difference between two headings (-180..180), e.g. from 170 to -170 is +20, not -340.
    private double angleDiff(double target, double current) {
        return AngleUnit.normalizeDegrees(target - current);
    }

    // We're at the known starting spot now (on the wall): reset the position guess.
    private void resetPoseHere() {
        poseX = START_X;
        poseY = START_Y;
        lastLeftTicks = robot.leftDrive.getCurrentPosition();
        lastRightTicks = robot.rightDrive.getCurrentPosition();
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

    // Is a class with this name available? (Used to tell if we're running in the simulator.)
    private static boolean classExists(String name) {
        try {
            Class.forName(name);
            return true;
        } catch (ClassNotFoundException e) {
            return false;
        }
    }
}
