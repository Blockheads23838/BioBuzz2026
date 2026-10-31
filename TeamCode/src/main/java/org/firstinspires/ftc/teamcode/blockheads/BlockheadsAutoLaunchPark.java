package org.firstinspires.ftc.teamcode.blockheads;

// Bring in the FTC SDK pieces this file uses (the SDK is the code library FIRST gives us to run
// the robot). DcMotor is the SDK's motor type; we only use its RunMode list, which names the
// different ways a motor can run (each mode is explained where we use it).
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * BH Auto: Launch + Park.
 *
 * WHAT THIS IS: an autonomous OpMode. An "OpMode" is one program you can pick and run from the
 * Driver Station phone. "Autonomous" means the robot drives itself, with no gamepads, during
 * the 30-second AUTO period at the start of a match.
 *
 * THE PLAN, one step (called a "state") at a time:
 *   1. LAUNCH: spin up the launcher and feed the pre-loaded POLLEN into it.
 *   2. DRIVE_OFF_WALL: back away from the wall. This earns LEAVE (3 pts).
 *   3. DRIVE_TO_PARK: drive to the LOADING ZONE for AUTO PARK (5 pts). The park distances are
 *      still 0 (not measured yet), so right now this step does not actually drive anywhere.
 *   4. DONE: stop everything and do nothing until AUTO ends.
 *
 * HOW IT IS BUILT: a "state machine". The robot is always in exactly ONE state. loop() runs
 * over and over, many times per second, and each time it only runs the code for the current
 * state. When that step is finished (or takes too long), we switch to the next state. This
 * way the program never has to pause or wait, and code at the bottom of loop() (like
 * telemetry) still runs every time.
 *
 * Based on goBILDA's StarterBotAuto (which is left untouched), with these changes:
 *   - intakePower and windmillPower are reset at the top of EVERY loop. In StarterBotAuto,
 *     "intakePower += 0.5" runs every loop once the launcher is up to speed, and it is not
 *     reset while LAUNCH is running, so the intake ends up at full power.
 *   - Every state has its own timer and a timeout, so a blocked robot can't get stuck forever.
 *   - Moves are RELATIVE: the drive encoders are reset when a move starts, and the move only
 *     counts as finished when BOTH encoders are within tolerance.
 *   - LAUNCH ends once the windmill has fed for LAUNCH_FEED_SECONDS, instead of a fixed 10 s.
 *   - Alliance is picked in init_loop (X = blue, B = red). Nothing moves before START (G304.H:
 *     after INIT, the robot must stay completely still until the match starts).
 *   - After parking, everything stops, and the TeleOp is pre-selected for the driver.
 *
 * States: LAUNCH -> DRIVE_OFF_WALL (earns LEAVE, 3 pts) -> DRIVE_TO_PARK (AUTO PARK, 5 pts) -> DONE
 *
 * MATCH RULES (the G-numbers are rule numbers from the game manual):
 *   - G305: run with the Driver Station's 30-second AUTO timer turned on.
 *   - G401: there is no gamepad input after START, so the robot must do everything by itself.
 *   - G402: stay on our alliance's side.
 *   - G417: don't drive into the HIVE.
 *
 * DRAFT: compiled only, never run on the robot. Remove @Disabled only after the distances below
 * have been measured and tested on a field. preselectTeleOp should change to "BH TeleOp" once
 * that OpMode exists.
 */

/*
 * The lines starting with @ are "annotations": labels that tell the FTC SDK about this program.
 *   - @Autonomous: list this OpMode in the Driver Station's AUTO menu, using this name. "group"
 *     only helps sort the list. preselectTeleOp: when this AUTO ends, the Driver Station
 *     automatically picks that TeleOp next, so the driver doesn't have to find it.
 *   - @Disabled: hide this OpMode from the Driver Station menu until we delete that line.
 * "extends OpMode" makes this an iterative OpMode: the SDK calls our init(), init_loop(),
 * start(), loop() and stop() methods for us at the right times (each one is explained below).
 */
@Autonomous(name = "BH Auto: Launch + Park", group = "Dev", preselectTeleOp = "BioBuzz StarterBot Teleop")
@Disabled // TODO(test): remove once this has been tested on the robot.
public class BlockheadsAutoLaunchPark extends OpMode {

    /*
     * Distances (mm) for each relative move. Positive = forward, negative = backward.
     * Every value here is a placeholder until it is measured on a field.
     *
     * "Relative" means measured from wherever the robot is when that move starts.
     * "static final" means a constant: a fixed number that never changes while the program runs.
     * Keeping all the numbers up here makes them easy to find and tune.
     */

    // TODO(tune): -120 mm is copied from StarterBotAuto. Check on a field that it gets fully off
    // the wall from our real starting spot and that the sign (backwards) is right.
    static final double OFF_WALL_MM = -120;

    /*
     * TODO(tune): the park move, measured from where DRIVE_OFF_WALL stops to the LOADING ZONE.
     * Left and right wheels are set separately: equal = straight, opposite = spin in place,
     * different = arc. A spin in place of D degrees is about D * PI / 180 * (TRACK_WIDTH_MM / 2)
     * mm per wheel (TRACK_WIDTH_MM, in StarterBotHardware, is the distance between the left and
     * right wheels). They are 0 ON PURPOSE for now, so the robot does NOT drive toward an
     * unmeasured spot: with a target of 0, each wheel is already "there", so DRIVE_TO_PARK just
     * finishes after MOVE_HOLD_SECONDS without driving anywhere.
     * If the path needs a turn AND a straight, split DRIVE_TO_PARK into two states with the
     * same pattern.
     */
    static final double BLUE_PARK_LEFT_MM = 0;
    static final double BLUE_PARK_RIGHT_MM = 0;
    static final double RED_PARK_LEFT_MM = 0;
    static final double RED_PARK_RIGHT_MM = 0;

    // Drive settings. DRIVE_SPEED and the 10 mm tolerance are copied from StarterBotAuto.
    //   - DRIVE_SPEED: motor power from 0 (stopped) to 1 (full). 0.5 = half power.
    //   - MOVE_TOLERANCE_MM: how close each wheel must get to its target to count as "there".
    //   - MOVE_HOLD_SECONDS: how long BOTH wheels must stay that close before the move is done,
    //     so we don't call it finished while the robot is still rolling past the target.
    static final double DRIVE_SPEED = 0.5;
    static final double MOVE_TOLERANCE_MM = 10;
    static final double MOVE_HOLD_SECONDS = 0.3; // TODO(tune): StarterBotAuto waited 2 s here.

    // How many seconds the windmill must run (feeding POLLEN into the launcher) before LAUNCH
    // is done. Only time while the launcher is up to speed counts toward this.
    // TODO(tune): time how long the windmill needs to clear all 4 POLLEN, then shrink this.
    static final double LAUNCH_FEED_SECONDS = 8.0;

    // Timeouts: if a state takes longer than this, give up on it and move on.
    // This way one problem (like a blocked wheel) can't use up the whole AUTO period.
    static final double LAUNCH_TIMEOUT_S = 10.0;       // same as StarterBotAuto's fixed launch time
    static final double DRIVE_OFF_WALL_TIMEOUT_S = 3.0;
    static final double DRIVE_TO_PARK_TIMEOUT_S = 8.0;

    // Whatever happens, stop everything before the 30 s AUTO period ends.
    // (Counted in seconds from when the driver presses START.)
    static final double AUTO_SAFETY_STOP_S = 28.5;

    // An "enum" is a fixed list of named choices. State lists every step of our plan, in order.
    // The variable "state" (below) holds which one we are on right now.
    private enum State {
        LAUNCH,
        DRIVE_OFF_WALL,
        DRIVE_TO_PARK,
        DONE
    }

    // Which alliance we are on. It stays NOT_SET until the driver presses X or B in init_loop().
    private enum Alliance {
        NOT_SET,
        BLUE,
        RED
    }

    // StarterBotHardware holds all of our motors and servos, with their names and settings,
    // so our new OpModes can share the same setup. "robot" is our copy of it.
    private final StarterBotHardware robot = new StarterBotHardware();

    // Which step we are on (we start at LAUNCH), and which alliance the driver picked.
    private State state = State.LAUNCH;
    private Alliance alliance = Alliance.NOT_SET;

    // ElapsedTime is a stopwatch: reset() starts it again from 0, and seconds() reads how many
    // seconds have passed since then.
    // One timer per state, reset when that state starts, plus one for the whole AUTO period.
    private final ElapsedTime launchTimer = new ElapsedTime();
    private final ElapsedTime offWallTimer = new ElapsedTime();
    private final ElapsedTime parkTimer = new ElapsedTime();
    private final ElapsedTime autoTimer = new ElapsedTime();

    // Counts how long a move has been within tolerance (both wheels).
    private final ElapsedTime holdTimer = new ElapsedTime();

    /*
     * These are recalculated every loop: zeroed at the top of loop(), set by the current state,
     * then written to the hardware exactly once at the bottom of loop().
     * (Starting from 0 every loop is the intake bug fix described at the top of this file.)
     */
    private double intakePower = 0;
    private double windmillPower = 0;

    // LAUNCH bookkeeping: how long the windmill has actually been feeding.
    // feedSeconds adds up only the time the launcher was fast enough and the windmill was
    // feeding. lastLaunchLoopTime remembers the launch timer reading from the last loop, so we
    // can work out how much time passed since then.
    private double feedSeconds = 0;
    private double lastLaunchLoopTime = 0;

    // Current relative move targets, in encoder ticks from where the move started.
    // An "encoder" counts "ticks" as the motor shaft turns, so ticks tell us how far a wheel
    // has rolled. DRIVE_TICKS_PER_MM (in StarterBotHardware) turns mm into ticks.
    private int leftTargetTicks = 0;
    private int rightTargetTicks = 0;

    // For telemetry: how long each state took, and which ones timed out.
    // -1 means "that state has not finished yet". "timeouts" is a line of text with state
    // names in it, like "LAUNCH DRIVE_OFF_WALL ", that grows each time a state gives up ("AUTO"
    // means the AUTO_SAFETY_STOP_S safety stop kicked in).
    private double launchSeconds = -1;
    private double offWallSeconds = -1;
    private double parkSeconds = -1;
    private String timeouts = "";

    /*
     * Code to run ONCE when the driver hits INIT. Configures hardware only; nothing moves (G304.H).
     * (@Override means we are filling in a method that the SDK's OpMode class already has.)
     */
    @Override
    public void init() {
        // hardwareMap is the SDK's list of every device in the robot's configuration.
        // robot.init() uses it to find each motor and servo by name and set it up.
        robot.init(hardwareMap);
        // telemetry is the text shown on the Driver Station screen. addData adds one line.
        telemetry.addData("Status", "Initialized. Pick alliance: X = BLUE, B = RED");
    }

    /*
     * Code to run REPEATEDLY after INIT, before START. Only reads buttons and shows telemetry;
     * it must never move anything (G304.H).
     */
    @Override
    public void init_loop() {
        // Alliance selection. gamepad1 is the first driver's controller, and gamepad1.x is true
        // while the X button is held down. X = BLUE, B = RED. The last button pressed wins, so
        // the driver can change their mind any time before START. We only remember the choice
        // here; nothing moves until START.
        if (gamepad1.x) {
            alliance = Alliance.BLUE;
        } else if (gamepad1.b) {
            alliance = Alliance.RED;
        }

        // Show the choice and a pre-match checklist on the Driver Station.
        // "a ? b : c" means: if a is true use b, otherwise use c. toString() turns BLUE or RED
        // into text, and addLine shows one plain line of text.
        telemetry.addData("Alliance", alliance == Alliance.NOT_SET
                ? "NOT SET (press X = BLUE, B = RED)" : alliance.toString());
        if (alliance == Alliance.NOT_SET) {
            telemetry.addLine("Without an alliance, the robot launches and leaves but skips PARK.");
        }
        telemetry.addLine("Check: 30 s AUTO timer ON. Robot touching the wall, 4 POLLEN loaded.");
    }

    /*
     * Code to run ONCE when the driver hits START.
     */
    @Override
    public void start() {
        // Start the stopwatch for the whole AUTO period, then begin the first step: LAUNCH.
        autoTimer.reset();
        enterState(State.LAUNCH);
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP.
     * Each run is one "loop", and there are many loops every second.
     */
    @Override
    public void loop() {
        // The bug fix: start every loop from zero, so nothing accumulates across loops.
        // The current state below sets these again only if it needs them. In StarterBotAuto,
        // intakePower was only ever added to ("+= 0.5") while launching, so it kept growing
        // loop after loop.
        // Starting from 0 each time means the intake and windmill only run while the current
        // state asks for them.
        intakePower = 0;
        windmillPower = 0;

        // Safety stop: if AUTO has been running longer than AUTO_SAFETY_STOP_S and we are not
        // done yet, note it and jump straight to DONE, which stops everything.
        if (state != State.DONE && autoTimer.seconds() > AUTO_SAFETY_STOP_S) {
            timeouts += "AUTO ";
            enterState(State.DONE);
        }

        // The state machine. "switch" runs only the "case" that matches the current state,
        // and "break" ends that case.
        switch (state) {
            // LAUNCH: shoot the pre-loaded POLLEN. The launcher was told to spin up when this
            // state started (see enterState). We only feed POLLEN in while it is spinning fast
            // enough (this is checked again every loop, so feeding pauses if it slows down).
            // getVelocity() reads the launcher's speed in encoder ticks per second.
            case LAUNCH: {
                double now = launchTimer.seconds();
                boolean atSpeed = robot.launcher.getVelocity() > StarterBotHardware.LAUNCHER_MIN_VELOCITY;
                // Fast enough: run the windmill and intake to push POLLEN into the launcher, and
                // add the time since the last loop to our feeding total. Time spent waiting
                // for the launcher to speed up is not counted.
                if (atSpeed) {
                    windmillPower = StarterBotHardware.WINDMILL_FEED_POWER;
                    intakePower = StarterBotHardware.INTAKE_FEED_POWER;
                    feedSeconds += now - lastLaunchLoopTime;
                }
                // Remember this loop's time for next loop's math.
                lastLaunchLoopTime = now;

                // Done once we have fed long enough. If it takes too long (for example, the
                // launcher never gets up to speed), give up, note the timeout, and move on
                // anyway so we can still earn LEAVE.
                if (feedSeconds >= LAUNCH_FEED_SECONDS) {
                    launchSeconds = now;
                    enterState(State.DRIVE_OFF_WALL);
                } else if (now > LAUNCH_TIMEOUT_S) {
                    launchSeconds = now;
                    timeouts += "LAUNCH ";
                    enterState(State.DRIVE_OFF_WALL);
                }
                break;
            }
            // DRIVE_OFF_WALL: the backwards move was started in enterState. Each loop we just
            // check if it is finished. If it takes too long (for example, the robot is blocked),
            // give up, note the timeout, and go on to the next state anyway.
            case DRIVE_OFF_WALL:
                if (moveFinished()) {
                    offWallSeconds = offWallTimer.seconds();
                    enterState(State.DRIVE_TO_PARK);
                } else if (offWallTimer.seconds() > DRIVE_OFF_WALL_TIMEOUT_S) {
                    offWallSeconds = offWallTimer.seconds();
                    timeouts += "DRIVE_OFF_WALL ";
                    enterState(State.DRIVE_TO_PARK);
                }
                break;
            // DRIVE_TO_PARK: same pattern as DRIVE_OFF_WALL, but for the park move. When it is
            // finished (or times out), we go to DONE.
            case DRIVE_TO_PARK:
                if (moveFinished()) {
                    parkSeconds = parkTimer.seconds();
                    enterState(State.DONE);
                } else if (parkTimer.seconds() > DRIVE_TO_PARK_TIMEOUT_S) {
                    parkSeconds = parkTimer.seconds();
                    timeouts += "DRIVE_TO_PARK ";
                    enterState(State.DONE);
                }
                break;
            case DONE:
                // Everything was stopped on entry. Nothing left to do.
                break;
        }

        // Write the intake and windmill once per loop. Only the intake MOTOR runs, as in
        // StarterBotAuto; the corner servos stay off so we don't pull in a 5th element (G407:
        // a robot may control at most 4 game elements at a time).
        // The windmill is a CRServo (a "continuous rotation" servo that spins all the way
        // around like a small motor), so setPower makes it spin.
        robot.intake.setPower(intakePower);
        robot.windmill.setPower(windmillPower);

        // Show what the robot is doing on the Driver Station, to help us test and debug.
        // In the text, "%.1f" shows a number with 1 digit after the decimal point, "%.0f" with
        // none, and "%d" shows a whole number.
        // The iterative OpMode sends telemetry automatically after each loop().
        telemetry.addData("State", state);
        telemetry.addData("Alliance", alliance);
        telemetry.addData("AUTO time", "%.1f s", autoTimer.seconds());
        telemetry.addData("Launcher", "%.0f ticks/s, fed %.1f s", robot.launcher.getVelocity(), feedSeconds);
        telemetry.addData("Left", "%d / %d ticks", robot.leftDrive.getCurrentPosition(), leftTargetTicks);
        telemetry.addData("Right", "%d / %d ticks", robot.rightDrive.getCurrentPosition(), rightTargetTicks);
        telemetry.addData("State times", "launch %.1f, off wall %.1f, park %.1f s",
                launchSeconds, offWallSeconds, parkSeconds);
        telemetry.addData("Timed out", timeouts.isEmpty() ? "none" : timeouts);
        if (state == State.DONE) {
            telemetry.addLine("Auto complete.");
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP (or the 30 s timer ends AUTO).
     */
    @Override
    public void stop() {
        // Turn off every motor and servo.
        robot.stopAll();
    }

    /*
     * Leave the current state and start the next one. Each state's entry work happens here,
     * exactly once, instead of being repeated every loop.
     */
    private void enterState(State next) {
        // Leaving any state: the drive stops before the next move is set up.
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        state = next;

        switch (next) {
            // Entering LAUNCH: start this state's stopwatch, clear the feeding total, and tell
            // the launcher to spin up. setVelocity asks the motor to hold a target speed (in
            // encoder ticks per second). The motor controller keeps adjusting the power to hold
            // that speed, using the launcher's PIDF settings (a speed-correction formula set up
            // in StarterBotHardware).
            case LAUNCH:
                launchTimer.reset();
                feedSeconds = 0;
                lastLaunchLoopTime = 0;
                robot.launcher.setVelocity(StarterBotHardware.LAUNCHER_TARGET_VELOCITY);
                break;
            // Entering DRIVE_OFF_WALL: turn the launcher off and start backing away from the
            // wall. Both wheels get the same distance, so the robot drives straight.
            case DRIVE_OFF_WALL:
                offWallTimer.reset();
                robot.launcher.setVelocity(0);
                startRelativeMove(OFF_WALL_MM, OFF_WALL_MM);
                break;
            // Entering DRIVE_TO_PARK: start the park move for the alliance the driver picked
            // in init_loop().
            case DRIVE_TO_PARK:
                parkTimer.reset();
                if (alliance == Alliance.BLUE) {
                    startRelativeMove(BLUE_PARK_LEFT_MM, BLUE_PARK_RIGHT_MM);
                } else if (alliance == Alliance.RED) {
                    startRelativeMove(RED_PARK_LEFT_MM, RED_PARK_RIGHT_MM);
                } else {
                    // No alliance picked: don't guess a direction. Stay where LEAVE put us.
                    // (Skip straight to DONE.)
                    parkSeconds = 0;
                    enterState(State.DONE);
                }
                break;
            // Entering DONE: stop every motor and servo.
            case DONE:
                robot.stopAll();
                // Leave the drive in plain power mode, ready for TeleOp.
                // RUN_WITHOUT_ENCODER means setPower just sets the motor's power directly,
                // with no position target.
                robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                break;
        }
    }

    /*
     * Start a move RELATIVE to where the robot is now: zero both encoders, then run to the
     * targets. setTargetPosition must come before RUN_TO_POSITION, or the SDK throws an error
     * and the OpMode stops.
     *
     * Why reset the encoders at the start of each move? Then every target simply means "this
     * many ticks from HERE". We never have to add up where the earlier moves ended, and a move
     * that timed out early can't throw off the next one.
     */
    private void startRelativeMove(double leftMm, double rightMm) {
        // Turn mm into encoder ticks. Math.round picks the nearest whole number, because
        // motor targets must be whole ticks ("int" means a whole number).
        leftTargetTicks = (int) Math.round(leftMm * StarterBotHardware.DRIVE_TICKS_PER_MM);
        rightTargetTicks = (int) Math.round(rightMm * StarterBotHardware.DRIVE_TICKS_PER_MM);

        // STOP_AND_RESET_ENCODER stops the motor and sets its tick count back to 0.
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Tell each motor where to go, in ticks from the new 0.
        robot.leftDrive.setTargetPosition(leftTargetTicks);
        robot.rightDrive.setTargetPosition(rightTargetTicks);

        // RUN_TO_POSITION: the motor controller now drives each motor to its target by itself,
        // and then holds it there. We don't have to steer it every loop.
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // In RUN_TO_POSITION, this power is the top speed the motor may use to get there.
        robot.leftDrive.setPower(DRIVE_SPEED);
        robot.rightDrive.setPower(DRIVE_SPEED);

        // Restart the "close enough" stopwatch for this new move.
        holdTimer.reset();
    }

    /*
     * True once BOTH wheels have stayed within MOVE_TOLERANCE_MM of their targets for
     * MOVE_HOLD_SECONDS. (StarterBotAuto only checked the left wheel.)
     */
    private boolean moveFinished() {
        // Turn the tolerance from mm into ticks, so we can compare it with the encoders.
        double toleranceTicks = MOVE_TOLERANCE_MM * StarterBotHardware.DRIVE_TICKS_PER_MM;
        // How far (in ticks) each wheel still is from its target. getCurrentPosition() reads
        // how many ticks that wheel's encoder has counted since it was reset to 0. Math.abs
        // makes the number positive, so going past the target counts the same as stopping
        // short of it.
        int leftError = Math.abs(leftTargetTicks - robot.leftDrive.getCurrentPosition());
        int rightError = Math.abs(rightTargetTicks - robot.rightDrive.getCurrentPosition());

        // Why check BOTH wheels? If we only checked the left one, the right wheel could still
        // be moving (or stuck) when we move on. If EITHER wheel is too far away, restart the
        // hold stopwatch.
        if (leftError > toleranceTicks || rightError > toleranceTicks) {
            holdTimer.reset();
        }
        // Finished only after both wheels have stayed close enough for MOVE_HOLD_SECONDS in
        // a row.
        return holdTimer.seconds() > MOVE_HOLD_SECONDS;
    }
}
