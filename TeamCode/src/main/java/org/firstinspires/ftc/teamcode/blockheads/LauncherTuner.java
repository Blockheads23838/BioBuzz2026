package org.firstinspires.ftc.teamcode.blockheads;

// Pull in the FTC SDK pieces this file uses: the OpMode base class, the @TeleOp label, motor
// modes, PIDF numbers and a stopwatch. (The SDK is the code library FIRST gives us.)
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Dev: Launcher Tuner. A practice-only TeleOp for tuning the launcher's velocity PIDF live,
 * without redeploying code (re-uploading it to the robot). The drivetrain never moves in this
 * OpMode.
 *
 * Words used in this file:
 *   - OpMode: a robot program you pick and run from the Driver Station (the drivers' app).
 *     A TeleOp is an OpMode driven with gamepads.
 *   - Velocity (speed) is measured in encoder ticks per second. An encoder counts small steps
 *     ("ticks") as the motor turns, so more ticks per second = the flywheel spins faster.
 *   - Hub: the robot's control box that the motors plug into (the Control Hub).
 *   - PIDF: the speed controller inside the hub. It changes the motor power many times a
 *     second to hold the target speed. We only tune two of its numbers here:
 *       P adds extra power based on how far the speed is from the target (bigger P = harder
 *         push back toward the target, but too big makes the speed bounce up and down).
 *       F gives a base amount of power that grows with the target speed, so the wheel gets
 *         close to the target on its own. (I and D stay at the shared values, which are 0.)
 *
 * Controls (gamepad1, the first driver's controller):
 *   A            toggle launcher spin-up on/off
 *   Dpad up/down target velocity +/- 50 ticks/s
 *   Dpad right/left  F +/- 0.5
 *   Right/left bumper  P +/- 5
 *   Hold B       feed (windmill + intake), but ONLY while velocity is within +/-3% of target
 *
 * Why wait for +/-3%? A ball fed while the flywheel is too slow (or too fast) flies a different
 * distance and can miss. Feeding only when the speed is close to the target makes every shot
 * leave at about the same speed, so the test shots tell you something useful.
 *
 * "Time to speed" = how many seconds the flywheel takes, after you press A to turn it on (or
 * change the target while it is spinning), to first get within +/-2% of the target speed.
 *
 * How to tune (see gm0 "Control Loops" and ftc-docs "PIDF coefficients"; gm0 = Game Manual 0,
 * a free community FTC guide):
 *   1. Set P to 0. Raise F until the wheel settles just under the target on its own.
 *   2. Raise P until it reaches the target quickly without oscillating (bouncing above and
 *      below the target).
 *   3. Goal (handbook task 2): settles within +/-2% in under 1.5 s, and 4 of 4 shots score.
 *   4. Copy the final numbers into StarterBotHardware (LAUNCHER_P / LAUNCHER_F / target).
 *
 * Results log. Write down battery voltage with every result:
 *   distance / spot | target ticks/s |  P  |  F   | volts | time-to-speed | shots scored
 *   ----------------|----------------|-----|------|-------|---------------|-------------
 *   (TODO)          |                |     |      |       |               |
 *
 * DRAFT: compiled only, not yet run on the robot.
 */
// @TeleOp puts this OpMode in the Driver Station's TeleOp list as "Dev: Launcher Tuner", grouped
// with other "Dev" tools.
// "extends OpMode" makes this an iterative OpMode: the SDK calls init() once at INIT, then calls
// loop() over and over after START, then stop() once at STOP. (This file does not use the
// optional init_loop() or start() steps, so they do nothing here.)
@TeleOp(name = "Dev: Launcher Tuner", group = "Dev")
public class LauncherTuner extends OpMode {

    // Step sizes for each button press.
    static final double TARGET_STEP = 50;
    static final double F_STEP = 0.5;
    static final double P_STEP = 5;

    // ~2800 ticks/s is the encoder's free-speed rate on a 5203 motor, so never ask for more.
    // ("Free speed" = top speed with nothing slowing the motor down.)
    static final double MAX_TARGET_VELOCITY = 2800;

    // B only feeds inside this band (fraction of target). 0.03 = 3%.
    static final double FEED_TOLERANCE = 0.03;

    // "Time-to-speed" stops counting the first time velocity is inside this band. 0.02 = 2%.
    static final double SETTLE_TOLERANCE = 0.02;

    // Our shared hardware helper (see StarterBotHardware.java). It holds all 7 devices.
    private final StarterBotHardware robot = new StarterBotHardware();

    // Values being tuned. They start at the shared defaults.
    private double targetVelocity = StarterBotHardware.LAUNCHER_TARGET_VELOCITY;
    private double p = StarterBotHardware.LAUNCHER_P;
    private double f = StarterBotHardware.LAUNCHER_F;

    // What the hub reports back (read at INIT and after each PIDF change). We show it on screen
    // to check that the hub really got our new numbers.
    private PIDFCoefficients hubPidf = null;

    // true while the launcher has been switched on with A.
    private boolean spinning = false;

    /*
     * Time-to-speed: measured from spin-up (or a target change while spinning) until the velocity
     * first gets within SETTLE_TOLERANCE. -1 means "nothing measured yet".
     * ElapsedTime is the SDK's stopwatch: reset() sets it back to 0, seconds() reads it.
     * "measuring" is true while we are still waiting for the wheel to reach speed.
     */
    private final ElapsedTime spinUpTimer = new ElapsedTime();
    private boolean measuring = false;
    private double timeToSpeed = -1;

    /*
     * Edge detection: remember each button from the last loop so one press = one step, no matter
     * how many loops the button stays down. (SDK 12 also has gamepad1.aWasPressed() and friends;
     * see the ConceptGamepadEdgeDetection sample.)
     * A button "was just pressed" when it is down now AND was up last loop. loop() runs many
     * times a second, so without this one tap would count as many presses.
     */
    private boolean lastA = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;

    /*
     * Code to run ONCE when the driver hits INIT. Configures hardware only; nothing moves (G304.H).
     * (Rule G304.H: after INIT the robot must stay completely still while it waits for START.)
     * @Override means this is our own version of a method that OpMode already names.
     */
    @Override
    public void init() {
        // Find and set up all 7 devices. hardwareMap is the SDK's list of the robot's devices.
        robot.init(hardwareMap);
        // Read the PIDF numbers the hub is using, so the screen can show them.
        hubPidf = robot.launcher.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // Telemetry = text the robot shows on the Driver Station screen.
        telemetry.addData("Status", "Initialized. Nothing moves until START, then A.");
        telemetry.addLine("A spin | Dpad U/D target | Dpad L/R F | Bumpers P | hold B feed");
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP.
     */
    @Override
    public void loop() {
        // Read each button once per loop. Each one is true while it is held down.
        boolean a = gamepad1.a;
        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;
        boolean dpadLeft = gamepad1.dpad_left;
        boolean dpadRight = gamepad1.dpad_right;
        boolean leftBumper = gamepad1.left_bumper;
        boolean rightBumper = gamepad1.right_bumper;

        // A: toggle spin-up. Each new press flips the launcher between on and off.
        // setVelocity() asks the hub to hold that speed (in ticks per second) using the PIDF
        // controller. Asking for 0 means "stop", so the flywheel slows down to a stop.
        // Turning on also starts the time-to-speed stopwatch; turning off stops measuring.
        if (a && !lastA) {
            spinning = !spinning;
            if (spinning) {
                robot.launcher.setVelocity(targetVelocity);
                startTimeToSpeed();
            } else {
                robot.launcher.setVelocity(0);
                measuring = false;
            }
        }

        // Dpad up/down: change the target. While spinning, send it now and time the new step.
        // Math.min / Math.max keep the target between 0 and MAX_TARGET_VELOCITY.
        boolean targetChanged = false;
        if (dpadUp && !lastDpadUp) {
            targetVelocity = Math.min(targetVelocity + TARGET_STEP, MAX_TARGET_VELOCITY);
            targetChanged = true;
        }
        if (dpadDown && !lastDpadDown) {
            targetVelocity = Math.max(targetVelocity - TARGET_STEP, 0);
            targetChanged = true;
        }
        if (targetChanged && spinning) {
            robot.launcher.setVelocity(targetVelocity);
            startTimeToSpeed();
        }

        // Dpad left/right: F. Bumpers: P. Only talk to the hub when something actually changed.
        // Math.max(..., 0) stops P and F from going below 0.
        boolean pidfChanged = false;
        if (dpadRight && !lastDpadRight) {
            f += F_STEP;
            pidfChanged = true;
        }
        if (dpadLeft && !lastDpadLeft) {
            f = Math.max(f - F_STEP, 0);
            pidfChanged = true;
        }
        if (rightBumper && !lastRightBumper) {
            p += P_STEP;
            pidfChanged = true;
        }
        if (leftBumper && !lastLeftBumper) {
            p = Math.max(p - P_STEP, 0);
            pidfChanged = true;
        }
        // Send the new P and F to the hub (I and D stay at the shared values), then read back
        // what the hub now holds so the screen shows it.
        if (pidfChanged) {
            robot.setLauncherPIDF(p, StarterBotHardware.LAUNCHER_I, StarterBotHardware.LAUNCHER_D, f);
            hubPidf = robot.launcher.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Remember this loop's buttons, so next loop can tell which ones were just pressed.
        lastA = a;
        lastDpadUp = dpadUp;
        lastDpadDown = dpadDown;
        lastDpadLeft = dpadLeft;
        lastDpadRight = dpadRight;
        lastLeftBumper = leftBumper;
        lastRightBumper = rightBumper;

        // Read the launcher's real speed in ticks per second.
        // Error = target minus actual: positive means the wheel is too slow, negative too fast.
        // errorPercent is the same error as a % of the target (0 when the target is 0, so we
        // never divide by zero).
        double velocity = robot.launcher.getVelocity();
        double error = targetVelocity - velocity;
        double errorPercent = (targetVelocity > 0) ? 100.0 * error / targetVelocity : 0;

        // Time to speed: the first loop the speed is within 2% of the target, save the
        // stopwatch time and stop measuring.
        if (spinning && measuring && targetVelocity > 0
                && Math.abs(error) <= SETTLE_TOLERANCE * targetVelocity) {
            timeToSpeed = spinUpTimer.seconds();
            measuring = false;
        }

        // B: feed only while spinning and inside the +/-3% band. Otherwise make sure feed is off.
        // (A ball fed at the wrong speed flies the wrong distance, so we wait until the wheel is
        // close to the target.) Feeding = the windmill servo pushes balls into the flywheel and
        // the intake roller helps push them up.
        boolean atSpeed = spinning && targetVelocity > 0
                && Math.abs(error) <= FEED_TOLERANCE * targetVelocity;
        boolean feeding = gamepad1.b && atSpeed;
        if (feeding) {
            robot.windmill.setPower(StarterBotHardware.WINDMILL_FEED_POWER);
            robot.intake.setPower(StarterBotHardware.INTAKE_FEED_POWER);
        } else {
            robot.windmill.setPower(0);
            robot.intake.setPower(0);
        }

        // Pick a short message for the screen that says what the feeder is doing and why.
        String feedStatus;
        if (feeding) {
            feedStatus = "FEEDING";
        } else if (gamepad1.b) {
            feedStatus = "blocked: not within 3% of target";
        } else {
            feedStatus = atSpeed ? "ready (hold B)" : "not ready";
        }

        // Time-to-speed text: the running stopwatch while measuring, the saved result once there
        // is one, or "-" when there is no result (nothing measured yet, or the launcher was
        // turned off before it reached speed).
        // ("%.2f" means "show the number with 2 digits after the decimal point"; "%%" prints a
        // single % sign.)
        String timeToSpeedText;
        if (measuring) {
            timeToSpeedText = String.format("measuring... %.2f s", spinUpTimer.seconds());
        } else if (timeToSpeed >= 0) {
            timeToSpeedText = String.format("%.2f s (to within 2%%)", timeToSpeed);
        } else {
            timeToSpeedText = "-";
        }

        // Show everything on the Driver Station screen: speed, error, time to speed, feed status,
        // the P/F we set, what the hub reports, and the battery voltage.
        // The iterative OpMode sends telemetry automatically after each loop().
        telemetry.addData("Launcher", spinning ? "ON" : "off");
        telemetry.addData("Target", "%.0f ticks/s", targetVelocity);
        telemetry.addData("Actual", "%.0f ticks/s", velocity);
        telemetry.addData("Error", "%.0f ticks/s (%.1f%%)", error, errorPercent);
        telemetry.addData("Time to speed", timeToSpeedText);
        telemetry.addData("Feed", feedStatus);
        telemetry.addLine();
        telemetry.addData("P / F (set)", "%.1f / %.2f", p, f);
        if (hubPidf != null) {
            telemetry.addData("Hub PIDF", "P %.2f  I %.2f  D %.2f  F %.2f",
                    hubPidf.p, hubPidf.i, hubPidf.d, hubPidf.f);
        }
        telemetry.addData("Battery", "%.2f V", robot.getBatteryVoltage());
        telemetry.addLine();
        telemetry.addLine("A spin | Dpad U/D target | Dpad L/R F | Bumpers P | hold B feed");
    }

    /*
     * Code to run ONCE after the driver hits STOP.
     * It sets every motor and servo to zero power.
     */
    @Override
    public void stop() {
        robot.stopAll();
    }

    /*
     * Start (or restart) the time-to-speed stopwatch from 0. It also clears the old result, so
     * the screen shows "measuring..." until the wheel reaches speed (or the launcher is turned
     * off).
     */
    private void startTimeToSpeed() {
        spinUpTimer.reset();
        measuring = true;
        timeToSpeed = -1;
    }
}
