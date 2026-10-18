package org.firstinspires.ftc.teamcode.blockheads;

// Tools from the FTC SDK and from Java that this file uses.
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;

/*
 * BH Auto: Full. A complete BIOBUZZ autonomous for the goBILDA StarterBot.
 * (AUTO = the first 30 seconds of a match, when the robot drives itself with no drivers.)
 *
 * From the AUDIENCE_WALL start spot (the default, and the only one that shoots):
 *   1. Back a few inches off the audience wall (LEAVE, 3 points).
 *   2. Turn so the launcher points at the opening of our upward HIVE CELL.
 *   3. Launch the 4 pre-loaded POLLEN (a TIP is worth 20).
 *   4. Drive around the HIVE into our LOADING ZONE (AUTO PARK, 5). LEAVE + PARK from both
 *      partners = SWARM RP (RP = ranking point).
 * The other two start spots (BESIDE_HIVE, BY_LOADING_ZONE) start on our alliance wall and only
 * LEAVE + PARK. They never shoot: from our alliance wall a shot would hit the SIDE of the CELL,
 * and deliberately hitting the HIVE any way except launching into its opening is a foul (G417).
 *
 * WHY THE AUDIENCE WALL (team plan, checked against the Field Setup Guide 11.1 and Field Reset
 * Guide E.i): at the start of every match the red CELL on the audience side is the upward one.
 * Its opening is its outer end, which faces the AUDIENCE wall (tilted up about 30 degrees; that
 * angle is a community estimate). So the shot must come from the audience side, heading away
 * from the audience. For BLUE everything is turned 180 degrees: the blue CELL on the far side is
 * up, and the blue robot shoots from the far wall toward the audience.
 *
 * Choose options in init_loop (nothing moves before START, G304.H):
 *   X = BLUE, B = RED        Dpad left/right = start spot
 *   Dpad up/down = wait before parking (use it so we don't meet our partner near the LOADING ZONE)
 * init_loop() is the code that runs after INIT is pressed and before START. Rule G304.H means the
 * robot must stay completely still in that time. Each dpad up/down press changes the park wait by
 * 1 s (0 to 18 s).
 *
 * HOW THE PATH WORKS
 * The path is a list of Steps using field points, measured for RED. The BIOBUZZ field is the same
 * after a 180-degree turn, so for BLUE every point is rotated 180 degrees about the center: the
 * robot makes exactly the same moves. (A RED point (x, y) becomes (-x, -y) for BLUE, and every
 * heading gets +180 degrees, so we only have to measure one set of points.) Where the robot is
 * comes from the drive encoders (distance) and the Control Hub IMU (heading). If the IMU is
 * missing, heading comes from the encoders. Tracking our own position this way is called
 * odometry, and the robot's position plus heading is called its "pose".
 *
 * FIELD FRAME (inches, same as the virtual_robot sim): origin at field center. +X points from the
 * RED alliance wall toward the BLUE alliance wall. +Y points from the audience wall toward the far
 * wall. Heading 0 = facing +Y, counter-clockwise positive. (FTC's usual frame has +X toward the
 * audience instead; nothing here depends on it.) Layout per Competition Manual TU05, Figs 9-2/9-5:
 * red LOADING ZONE on tile A5, red GARDEN at the A1 corner, red-wall FLOWER on the row 2|3 seam.
 * Example: heading 0 = facing the far wall; heading 180 = facing the audience wall;
 * heading 90 = facing -X (toward the RED wall); heading -90 = facing +X.
 *
 * MATCH RULES: 30 s AUTO timer ON (G305). No gamepad input after START (G401). Stay on our side
 * (G402: the path never crosses x = 0). Only affect the HIVE by launching into the upward CELL's
 * opening (G417: the drive path stays well away from the HIVE). Never control more than 4
 * elements (G407: this auto never intakes; the corner-pull servos stay off).
 *
 * DRAFT: tested only in the virtual_robot sim. The sim has no HIVE, CELLs or balls, so it proves
 * driving, timing and parking only, NOT that the launch scores. Field points come from the sim's
 * BIOBUZZ field image and the team plan, good to about +/-1 in; every TODO must be checked on a
 * real field. Remove @Disabled only after it has run safely on the robot.
 */
// An OpMode is one program you can pick and run from the Driver Station.
// @Autonomous puts this OpMode in the Driver Station's AUTO list with this name; group = "Dev"
// files it with our development OpModes. preselectTeleOp: when this auto finishes, the Driver
// Station automatically picks that TeleOp (driver-controlled) OpMode, ready for the next period.
// @Disabled hides this OpMode from the Driver Station list until that line is removed.
// "extends OpMode" makes this an iterative OpMode: the SDK calls init() once at INIT, init_loop()
// over and over until START, start() once at START, loop() over and over until this OpMode ends
// (the 30 s AUTO timer runs out or STOP is pressed), and stop() once at the end.
@Autonomous(name = "BH Auto: Full", group = "Dev", preselectTeleOp = "BioBuzz StarterBot Teleop")
@Disabled // TODO(test): remove once this has been run on the robot.
public class BlockheadsAutoFull extends OpMode {

    // ----------------------------------------------------------------------------------------
    // Robot facts
    // ----------------------------------------------------------------------------------------

    /*
     * TODO(robot): which end the launcher shoots from. goBILDA's StarterBotAuto launches from the
     * wall and then drives BACKWARDS to leave, which only works if the robot's back faces the
     * HIVE, and goBILDA's match video shows the intake end against the wall with the launch
     * tower at the back. Check on our robot; flip this if it shoots out the front.
     *
     * true = the launcher shoots out of the robot's back, so the front (intake) end goes against
     * the wall. ("static final" means a fixed value, a constant, that never changes while the code
     * runs.)
     */
    static final boolean LAUNCHER_ON_BACK = true;

    // Encoder ticks: each drive motor has an encoder that counts "ticks" as the wheel turns.
    // DRIVE_TICKS_PER_IN = how many ticks the encoder counts for each inch the wheel rolls
    // (25.4 mm = 1 inch). Dividing a tick count by it gives inches traveled.
    // TRACK_WIDTH_IN is the distance between the left and right wheels. It is used to work out
    // how far the robot has turned from the two wheel counts when there is no IMU.
    static final double DRIVE_TICKS_PER_IN = StarterBotHardware.DRIVE_TICKS_PER_MM * 25.4;
    static final double TRACK_WIDTH_IN = StarterBotHardware.TRACK_WIDTH_MM / 25.4;

    // The IMU is a motion sensor inside the Control Hub. We use it to read the robot's heading
    // (which way it faces; the IMU calls this "yaw"). It has to be told how the hub is mounted.
    // TODO(robot): how the Control Hub is mounted. Only matters if the IMU is used.
    static final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    static final RevHubOrientationOnRobot.UsbFacingDirection HUB_USB = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    // TODO(robot): half the robot's length, front bumper to back bumper. When the robot is pushed
    // flat against a wall, its center is this far from the wall.
    static final double ROBOT_HALF_LENGTH = 9.0;

    // ----------------------------------------------------------------------------------------
    // Field points (RED, inches). TODO(field): check each one on a real field.
    // ----------------------------------------------------------------------------------------

    // Where to aim: the middle of the upward red CELL's OPENING (not the middle of the CELL's
    // box). Team plan: the opening is at about x = -12.75 and spans y = -16 to -23, with its lower
    // lip about 53.5 in above the floor and its top about 65.6 in up.
    static final double OPENING_X = -12.75, OPENING_Y = -19.5;

    // Start spot AUDIENCE_WALL ("S1" in the team plan), the default and the only one that shoots.
    // Intake end against the audience wall (y = -72) in tile column C, launcher end toward the
    // HIVE. y = -63 is the wall (-72) plus half the robot's length (9).
    static final double AUDIENCE_WALL_X = -17, AUDIENCE_WALL_Y = -63;
    // How far to back off the audience wall before turning to aim. Turning in place sweeps a
    // 12.7 in radius (half the robot's diagonal), so the center needs to be more than 12.7 in
    // from the wall: -63 + 5 = -58 is 14 in from it. Backing off also earns LEAVE.
    static final double AUDIENCE_WALL_BACKOFF_IN = 5;

    // Start spots on our alliance wall (front against the red wall, x = -72 + 9 = -63).
    // LEAVE + PARK only, no shooting (a shot from here would hit the CELL's side).
    // BESIDE_HIVE: y = -9 keeps the robot 3 in clear of the FLOWER on the red wall (y = -21 to -27).
    // BY_LOADING_ZONE: just below the LOADING ZONE, which starts at y = 24.2.
    static final double BESIDE_HIVE_X = -63, BESIDE_HIVE_Y = -9;
    static final double BY_LZ_X = -63, BY_LZ_Y = 12;

    // Where to stop after backing off our alliance wall: far enough out (18 in from the wall) to
    // turn in place without a corner touching it.
    static final double LEAVE_X = -54;

    // On the way from the audience wall to the LOADING ZONE we pass this point, which keeps the
    // robot well clear of the HIVE (x -24.6..24.6, y -19.5..19.5) and of the red-wall FLOWER.
    static final double ROUTE_X = -54, ROUTE_Y = -9;

    // LOADING ZONE is x -72..-60.4, y 24.2..47.1. PARK only needs part of the robot inside, and
    // LEAVE needs it off the wall, so both park points stop ~5 in from the wall.
    // PARK_LOW is the lower end of the zone (used from AUDIENCE_WALL and BESIDE_HIVE), PARK_HIGH
    // the upper end (used from BY_LOADING_ZONE), so two partner robots can both fit.
    static final double PARK_LOW_X = -57.5, PARK_LOW_Y = 20;
    static final double PARK_HIGH_X = -59, PARK_HIGH_Y = 41;

    // ----------------------------------------------------------------------------------------
    // Tuning
    // ----------------------------------------------------------------------------------------

    // Motor power goes from -1 to 1 (1 = full power forward). The "KP" numbers are proportional
    // gains: power = KP x the error (how far we still have to go, or for HEADING_KP how many
    // degrees off course we are), so the robot pushes hard when it is far from its goal and eases
    // off as it gets close.
    // Minimum power: below DRIVE_MIN_POWER the motors can't beat friction, so without it the robot
    // would stall just short of its target. The ramp: drive power may only RISE by
    // DRIVE_RAMP_PER_S each second (0 to 0.7 takes 0.35 s), so the wheels don't spin or tip us.
    // "Tolerance" = close enough to count as there.
    static final double DRIVE_MAX_POWER = 0.7;
    static final double DRIVE_MIN_POWER = 0.12;       // TODO(tune): least power that still moves the robot
    static final double DRIVE_KP = 0.06;              // power per inch of distance left
    static final double DRIVE_RAMP_PER_S = 2.0;       // max power change per second (no wheelies)
    static final double DRIVE_TOLERANCE_IN = 0.75;
    static final double HEADING_KP = 0.02;            // steering power per degree while driving

    // Turning in place works the same way but with no ramp: power proportional to the degrees
    // left, capped at TURN_MAX_POWER, at least TURN_MIN_POWER until within TURN_TOLERANCE_DEG.
    // A turn or drive is done once it has stayed within tolerance for SETTLE_SECONDS.
    static final double TURN_MAX_POWER = 0.5;
    static final double TURN_MIN_POWER = 0.12;
    static final double TURN_KP = 0.012;              // power per degree of heading error
    static final double TURN_TOLERANCE_DEG = 2.0;

    static final double SETTLE_SECONDS = 0.15;        // must stay in tolerance this long

    // TODO(tune): how long the windmill must run to clear 4 POLLEN. goBILDA's auto used 10 s total.
    // LAUNCH_FEED_SECONDS only counts time while the launcher is up to speed and feeding.
    // LAUNCH_TIMEOUT_S: give up on a launch step after this many seconds, no matter what.
    static final double LAUNCH_FEED_SECONDS = 4.0;
    static final double LAUNCH_TIMEOUT_S = 9.0;

    // TIME GUARDS. AUTO lasts 30 s, and these times are counted from START.
    //  - LATEST_PARK_START_S: at 20 s, if we haven't started parking yet, drop whatever we are
    //    doing and jump to the parking steps. The drive from the audience wall to the LOADING
    //    ZONE takes about 7 s, so this always leaves time to PARK.
    //  - MAX_PARK_WAIT_S: the most the park wait (dpad up/down) can be set to, for the same reason.
    //  - AUTO_SAFETY_STOP_S: at 28.5 s, stop every motor and servo and end the auto, so the robot
    //    is still before AUTO ends.
    static final double LATEST_PARK_START_S = 20.0;   // whatever is happening, start parking by then
    static final double MAX_PARK_WAIT_S = 18;
    static final double AUTO_SAFETY_STOP_S = 28.5;    // stop everything before AUTO ends

    // ----------------------------------------------------------------------------------------
    // Options picked in init_loop
    // ----------------------------------------------------------------------------------------

    // An enum is a type with a short, fixed list of named choices, like RED or BLUE.
    private enum Alliance { RED, BLUE }

    // The start spots, in the order dpad right steps through them (dpad left goes backwards).
    private enum StartSpot { AUDIENCE_WALL, BESIDE_HIVE, BY_LOADING_ZONE }

    // The current choices. Defaults: RED, AUDIENCE_WALL, no park wait.
    // allianceChosen is only used to show "NOT SET" on telemetry (the text on the Driver Station
    // screen) until X or B is pressed.
    // parkWaitUntilS: the final drive into the LOADING ZONE won't start before this AUTO time (s).
    private Alliance alliance = Alliance.RED;
    private boolean allianceChosen = false;
    private StartSpot startSpot = StartSpot.AUDIENCE_WALL;
    private double parkWaitUntilS = 0;

    // Button states from the last init_loop() pass. Comparing "pressed now" with "pressed last
    // time" is called edge detection: it makes one press count once, even though init_loop()
    // runs many times while a button is held down.
    private boolean lastDpadLeft, lastDpadRight, lastDpadUp, lastDpadDown;

    // ----------------------------------------------------------------------------------------
    // Steps
    // ----------------------------------------------------------------------------------------

    // The auto is a list of Steps done one at a time, in order. This is a simple "state machine":
    // the robot is always on exactly one step (its "state") and moves to the next step when the
    // current one finishes or runs out of time (a time check can also jump ahead to parking).
    // Kind says what kind of thing a step does.
    private enum Kind {
        LAUNCH,          // spin up (if needed) and feed until LAUNCH_FEED_SECONDS of feeding
        DRIVE_TO,        // turn toward a point (front or back, whichever turns less), then drive to it
        AIM_AT,          // turn so the launcher points at a point
        WAIT_UNTIL_PARK_TIME, // wait until AUTO time reaches the park wait picked in init_loop
        // Instant step: start spinning the launcher, then go straight to the next step.
        LAUNCHER_ON
    }

    /*
     * One step of the path. "final" means each value is set once and never changes.
     *   kind    = what to do (see Kind above)
     *   label   = name shown on telemetry; the parking time check also finds a step by its label
     *   x, y    = a field point in RED inches (for DRIVE_TO and AIM_AT)
     *   timeout = give up on the step after this many seconds (0 = no limit)
     */
    private static final class Step {
        final Kind kind;
        final String label;
        final double x, y, timeout;

        Step(Kind kind, String label, double x, double y, double timeout) {
            this.kind = kind;
            this.label = label;
            this.x = x;
            this.y = y;
            this.timeout = timeout;
        }
    }

    // Shortcuts for making Steps, so the path below is short and easy to read:
    //   step()  = a step with no field point (LAUNCH, LAUNCHER_ON, WAIT_UNTIL_PARK_TIME)
    //   point() = a step with a field point (DRIVE_TO, AIM_AT)
    private static Step step(Kind kind, String label, double timeout) {
        return new Step(kind, label, 0, 0, timeout);
    }

    private static Step point(Kind kind, String label, double x, double y, double timeout) {
        return new Step(kind, label, x, y, timeout);
    }

    // Every path's parking part starts with a step with this label. The time check in loop()
    // jumps here when we are running late, so we never lose the PARK points.
    private static final String PARK_LABEL = "Start parking";

    /*
     * The path for each start spot, in RED field points. Timeouts are generous; if one trips,
     * the step is abandoned and the next one starts (telemetry lists which ones timed out).
     * Each path.add(...) line below is one step, done in order from top to bottom (the time
     * check may skip ahead to parking).
     */
    private List<Step> buildPath() {
        List<Step> path = new ArrayList<Step>();

        if (startSpot == StartSpot.AUDIENCE_WALL) {
            // Start spinning the launcher now, so it is up to speed by the time we have aimed.
            path.add(step(Kind.LAUNCHER_ON, "Pre-spin launcher", 0));
            // Back straight off the audience wall (this earns LEAVE and gives room to turn).
            path.add(point(Kind.DRIVE_TO, "Back off the wall",
                    AUDIENCE_WALL_X, AUDIENCE_WALL_Y + AUDIENCE_WALL_BACKOFF_IN, 2.0));
            // Turn a little so the launcher points at the CELL opening, then shoot.
            path.add(point(Kind.AIM_AT, "Aim at the CELL opening", OPENING_X, OPENING_Y, 2.0));
            path.add(step(Kind.LAUNCH, "Launch pre-loads", LAUNCH_TIMEOUT_S));
            // Parking: wait for the park time picked in init_loop, go around the HIVE, then into
            // the lower end of the LOADING ZONE.
            path.add(step(Kind.WAIT_UNTIL_PARK_TIME, PARK_LABEL, 30));
            path.add(point(Kind.DRIVE_TO, "Around the HIVE", ROUTE_X, ROUTE_Y, 4.0));
            path.add(point(Kind.DRIVE_TO, "Park in LOADING ZONE", PARK_LOW_X, PARK_LOW_Y, 4.0));
        } else if (startSpot == StartSpot.BESIDE_HIVE) {
            // No shooting from our alliance wall. Back off it (LEAVE), wait, then park low.
            path.add(point(Kind.DRIVE_TO, "Leave the wall", LEAVE_X, BESIDE_HIVE_Y, 2.5));
            path.add(step(Kind.WAIT_UNTIL_PARK_TIME, PARK_LABEL, 30));
            path.add(point(Kind.DRIVE_TO, "Park in LOADING ZONE", PARK_LOW_X, PARK_LOW_Y, 4.0));
        } else {
            // BY_LOADING_ZONE: no shooting. Back off the wall (LEAVE), wait, then park high.
            path.add(point(Kind.DRIVE_TO, "Leave the wall", LEAVE_X, BY_LZ_Y, 2.5));
            path.add(step(Kind.WAIT_UNTIL_PARK_TIME, PARK_LABEL, 30));
            path.add(point(Kind.DRIVE_TO, "Park in LOADING ZONE", PARK_HIGH_X, PARK_HIGH_Y, 4.0));
        }
        return path;
    }

    // ----------------------------------------------------------------------------------------
    // State
    // ----------------------------------------------------------------------------------------

    // The StarterBot's motors and servos (set up in StarterBotHardware), and the IMU if found.
    private final StarterBotHardware robot = new StarterBotHardware();
    private IMU imu = null;

    // The list of steps, which one we are on (0 = the first), and whether the auto is finished.
    private List<Step> path = new ArrayList<Step>();
    private int stepIndex = 0;
    private boolean done = false;

    // ElapsedTime is a stopwatch. autoTimer = time since START. stepTimer = time in the current
    // step. settleTimer = how long we have stayed inside tolerance. loopTimer = time since the
    // last loop() (called dt).
    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime stepTimer = new ElapsedTime();
    private final ElapsedTime settleTimer = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();

    // Pose estimate (field frame, inches/degrees) and encoder bookkeeping.
    // Pose = where the robot thinks it is: x and y in inches, heading in degrees.
    // startHeading = our heading at START; current heading = startHeading + how far we have
    // turned since START. The tick counts remember earlier encoder readings (last loop and
    // START), so we can tell how far each wheel has moved.
    private double poseX, poseY, poseHeading;
    private double startHeading;
    private int lastLeftTicks, lastRightTicks;
    private int startLeftTicks, startRightTicks;

    // Per-step state.
    private boolean turning;            // DRIVE_TO: true while turning, false while driving
    private double targetHeading;       // the heading this step turns to (and holds while driving)
    private double driveDistance;       // inches to drive in this step (negative = backwards)
    private int legLeftTicks, legRightTicks;   // encoder counts when the straight drive started
    private double commandedPower;      // drive power after the ramp (it may only rise slowly)
    private double feedSeconds;         // seconds spent feeding POLLEN in this LAUNCH step

    // Whether the launcher should be spinning, and a text list of steps that timed out or were
    // skipped (shown on telemetry).
    private boolean launcherOn = false;
    private String timeouts = "";

    // ----------------------------------------------------------------------------------------
    // OpMode
    // ----------------------------------------------------------------------------------------

    // init() runs ONCE when the drivers press INIT. It only sets up hardware; nothing moves.
    // (@Override marks a method that OpMode already names, which we fill in with our own code.
    // OpMode requires init() and loop(); init_loop(), start() and stop() are optional.)
    @Override
    public void init() {
        // hardwareMap is the list of devices in the robot's configuration on the Control Hub.
        // robot.init() finds every motor and servo by name and sets them all to zero power.
        robot.init(hardwareMap);

        // The IMU is optional: if the configuration has no "imu", fall back to encoder heading.
        // initialize() tells the IMU how the hub is mounted on the robot.
        try {
            imu = hardwareMap.get(IMU.class, "imu");
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(HUB_LOGO, HUB_USB)));
        } catch (Exception e) {
            imu = null;
        }
        // telemetry = text shown on the Driver Station screen.
        telemetry.addData("Status", "Initialized. Nothing moves until START.");
    }

    // init_loop() runs over and over after INIT until START is pressed. We use it to pick options.
    @Override
    public void init_loop() {
        // Only reads buttons and shows telemetry: nothing may move before START (G304.H).
        // gamepad1 is the first driver's controller. X = BLUE, B = RED.
        if (gamepad1.x) { alliance = Alliance.BLUE; allianceChosen = true; }
        if (gamepad1.b) { alliance = Alliance.RED; allianceChosen = true; }

        // Read the buttons once, then act only on a NEW press (pressed now, not pressed last
        // time). Dpad right: next start spot. Dpad left: previous start spot (both wrap around).
        // Dpad up/down: park wait +1 s / -1 s, kept between 0 and MAX_PARK_WAIT_S.
        boolean dpadLeft = gamepad1.dpad_left, dpadRight = gamepad1.dpad_right;
        boolean dpadUp = gamepad1.dpad_up, dpadDown = gamepad1.dpad_down;
        StartSpot[] spots = StartSpot.values();
        if (dpadRight && !lastDpadRight) startSpot = spots[(startSpot.ordinal() + 1) % spots.length];
        if (dpadLeft && !lastDpadLeft) startSpot = spots[(startSpot.ordinal() + spots.length - 1) % spots.length];
        if (dpadUp && !lastDpadUp) parkWaitUntilS = Math.min(parkWaitUntilS + 1, MAX_PARK_WAIT_S);
        if (dpadDown && !lastDpadDown) parkWaitUntilS = Math.max(parkWaitUntilS - 1, 0);
        // Remember this pass's buttons for the next pass.
        lastDpadLeft = dpadLeft; lastDpadRight = dpadRight; lastDpadUp = dpadUp; lastDpadDown = dpadDown;

        // Show the current choices and setup reminders. Because BLUE is RED turned 180 degrees,
        // the robot's moves are the same for both alliances, so a forgotten alliance choice does
        // not change the moves (it only changes the field positions shown on telemetry).
        telemetry.addData("Alliance (X blue / B red)", allianceChosen ? alliance.toString() : "NOT SET (moves are the same)");
        telemetry.addData("Start spot (dpad L/R)", startSpot + (startSpot == StartSpot.AUDIENCE_WALL ? " (shoots)" : " (leave + park only)"));
        telemetry.addData("Park no earlier than (dpad U/D)", "%.0f s", parkWaitUntilS);
        telemetry.addData("Heading from", imu != null ? "IMU" : "encoders (no \"imu\" in config)");
        telemetry.addLine(placementHint());
        telemetry.addLine("Check: 30 s AUTO timer ON, 4 POLLEN loaded.");
    }

    // Where the drive team should put the robot for the chosen start spot and alliance.
    // For BLUE the field is turned 180 degrees, so the audience wall becomes the far wall and
    // "left of center" (seen from the audience) becomes "right of center".
    private String placementHint() {
        boolean blue = alliance == Alliance.BLUE;
        if (startSpot == StartSpot.AUDIENCE_WALL) {
            return blue
                    ? "Place: intake against the FAR wall, center 17 in right of the middle line (column D), launcher toward the HIVE."
                    : "Place: intake against the AUDIENCE wall, center 17 in left of the middle line (column C), launcher toward the HIVE.";
        } else if (startSpot == StartSpot.BESIDE_HIVE) {
            return "Place: intake against our alliance wall, 3 in clear of the FLOWER, back toward the HIVE.";
        } else {
            return "Place: intake against our alliance wall, just short of the LOADING ZONE tape.";
        }
    }

    // start() runs ONCE when START is pressed. The AUTO clock starts here.
    @Override
    public void start() {
        autoTimer.reset();
        loopTimer.reset();

        // Starting pose (RED frame, converted for BLUE). Every start spot has the intake end
        // against a wall. From AUDIENCE_WALL the launcher end points +y (away from the audience);
        // from our alliance wall it points +x (toward the HIVE).
        double sx, sy, launcherDx, launcherDy;
        if (startSpot == StartSpot.AUDIENCE_WALL) {
            sx = AUDIENCE_WALL_X; sy = AUDIENCE_WALL_Y; launcherDx = 0; launcherDy = 1;
        } else if (startSpot == StartSpot.BESIDE_HIVE) {
            sx = BESIDE_HIVE_X; sy = BESIDE_HIVE_Y; launcherDx = 1; launcherDy = 0;
        } else {
            sx = BY_LZ_X; sy = BY_LZ_Y; launcherDx = 1; launcherDy = 0;
        }
        poseX = fieldX(sx, sy);
        poseY = fieldY(sx, sy);
        startHeading = fieldHeading(launcherHeadingToward(launcherDx, launcherDy));
        poseHeading = startHeading;

        // resetYaw() sets the IMU's heading to 0 right now, so later it reads how far we have
        // turned since START.
        if (imu != null) imu.resetYaw();
        // DcMotor is the SDK's name for a regular motor, and RunMode picks how it uses its
        // encoder. RUN_USING_ENCODER: each drive motor uses its encoder to hold a steady speed
        // for the power we give it. Then remember the starting encoder counts.
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lastLeftTicks = startLeftTicks = robot.leftDrive.getCurrentPosition();
        lastRightTicks = startRightTicks = robot.rightDrive.getCurrentPosition();

        // Build the step list from the options picked in init_loop, then begin the first step.
        path = buildPath();
        stepIndex = 0;
        beginStep();
    }

    // loop() runs over and over (many times a second) from START until AUTO ends.
    @Override
    public void loop() {
        // dt = seconds since the last loop (used by the drive ramp and to count feeding time).
        // Then update where we think we are (odometry).
        double dt = loopTimer.seconds();
        loopTimer.reset();
        updatePose();

        // Time guard: at AUTO_SAFETY_STOP_S (28.5 s) stop everything, so the robot is still
        // before AUTO ends.
        if (!done && autoTimer.seconds() > AUTO_SAFETY_STOP_S) {
            timeouts += "AUTO ";
            finish();
        }
        // Time guard: after LATEST_PARK_START_S (20 s), if the parking steps are still ahead of
        // us, turn off the launcher and jump straight to parking, so we never lose the PARK points.
        int parkIndex = indexOf(PARK_LABEL);
        if (!done && autoTimer.seconds() > LATEST_PARK_START_S && parkIndex > stepIndex) {
            timeouts += "late, parking now; ";
            launcherOn = false;
            stepIndex = parkIndex;
            beginStep();
        }

        // Recomputed every loop, written once at the end (the intakePower bug fix from StarterBotAuto).
        // Every power starts at zero and the current step changes the ones it needs.
        double leftPower = 0, rightPower = 0;
        double windmillPower = 0;
        double intakePower = 0;

        // Do the current step. Each case sets powers and decides if the step is finished.
        if (!done) {
            Step s = path.get(stepIndex);
            boolean finished = false;
            double[] drive = {0, 0};

            switch (s.kind) {
                case LAUNCH: {
                    // Keep the launcher spinning. getVelocity() reads its speed in encoder ticks
                    // per second. Only feed POLLEN (windmill + intake) once it is up to speed,
                    // and only count that feeding time toward LAUNCH_FEED_SECONDS.
                    launcherOn = true;
                    if (robot.launcher.getVelocity() > StarterBotHardware.LAUNCHER_MIN_VELOCITY) {
                        windmillPower = StarterBotHardware.WINDMILL_FEED_POWER;
                        intakePower = StarterBotHardware.INTAKE_FEED_POWER;
                        feedSeconds += dt;
                    }
                    finished = feedSeconds >= LAUNCH_FEED_SECONDS;
                    break;
                }
                // Turning and driving steps are handled by runMove() (further down).
                case DRIVE_TO:
                case AIM_AT:
                    finished = runMove(s, dt, drive);
                    break;
                // Wait until the AUTO clock reaches the park time picked in init_loop.
                case WAIT_UNTIL_PARK_TIME:
                    finished = autoTimer.seconds() >= parkWaitUntilS;
                    break;
                default:
                    finished = true;   // instant steps are handled in beginStep()
                    break;
            }
            leftPower = drive[0];
            rightPower = drive[1];

            // Timeout safety: if a step takes too long, note it on telemetry and move on anyway.
            if (!finished && s.timeout > 0 && stepTimer.seconds() > s.timeout) {
                timeouts += s.label + "; ";
                finished = true;
            }
            if (finished) nextStep();
        }

        // Send this loop's powers to the hardware (skipped once done: finish() already stopped
        // everything).
        if (!done) {
            robot.leftDrive.setPower(leftPower);
            robot.rightDrive.setPower(rightPower);
            // setVelocity() asks the launcher motor for an exact speed in ticks per second. The
            // hub's PIDF speed controller (PIDF = the math that compares the real speed with the
            // target; its numbers are set in StarterBotHardware) keeps adjusting the power to hold
            // that speed.
            robot.launcher.setVelocity(launcherOn ? StarterBotHardware.LAUNCHER_TARGET_VELOCITY : 0);
            // The windmill is a CRServo (a continuous-rotation servo that spins like a small
            // motor, power -1 to 1). The intake motor only nudges POLLEN up while feeding.
            robot.windmill.setPower(windmillPower);
            robot.intake.setPower(intakePower);
            // The corner-pull servos never run in this auto, so we can't grab extra elements (G407).
            robot.leftIntakeServo.setPower(0);
            robot.rightIntakeServo.setPower(0);
        }

        // Show progress on the Driver Station: step, time, pose, launcher speed, options, timeouts.
        telemetry.addData("Step", done ? "DONE" : (stepIndex + 1) + "/" + path.size() + " " + path.get(stepIndex).label);
        telemetry.addData("AUTO time", "%.1f s", autoTimer.seconds());
        telemetry.addData("Pose", "x %.1f  y %.1f  heading %.1f", poseX, poseY, poseHeading);
        telemetry.addData("Launcher", "%.0f ticks/s, fed %.1f s", robot.launcher.getVelocity(), feedSeconds);
        telemetry.addData("Alliance / start", "%s / %s", alliance, startSpot);
        telemetry.addData("Heading from", imu != null ? "IMU" : "encoders");
        telemetry.addData("Timed out", timeouts.isEmpty() ? "none" : timeouts);
    }

    // stop() runs ONCE when AUTO ends or STOP is pressed: zero power to every motor and servo.
    @Override
    public void stop() {
        robot.stopAll();
    }

    // ----------------------------------------------------------------------------------------
    // Steps
    // ----------------------------------------------------------------------------------------

    /*
     * Set up the step at stepIndex. Instant steps (launcher on) are done right here and we move
     * straight past them, until we reach a step that takes time. That step is then prepared
     * below, and loop() runs it.
     */
    private void beginStep() {
        // Instant steps run here and immediately move on.
        while (stepIndex < path.size() && path.get(stepIndex).kind == Kind.LAUNCHER_ON) {
            launcherOn = true;
            stepIndex++;
        }
        // Ran out of steps: the auto is done.
        if (stepIndex >= path.size()) {
            finish();
            return;
        }

        // Fresh start for this step: reset its stopwatches and numbers.
        Step s = path.get(stepIndex);
        stepTimer.reset();
        settleTimer.reset();
        commandedPower = 0;
        feedSeconds = 0;

        switch (s.kind) {
            case DRIVE_TO: {
                // Turn-then-drive: first turn in place to face the target, then drive straight.
                // Work out the target in this alliance's frame, how far away it is, and the
                // heading that points at it (the "bearing").
                double tx = fieldX(s.x, s.y), ty = fieldY(s.x, s.y);
                double dx = tx - poseX, dy = ty - poseY;
                double distance = Math.hypot(dx, dy);
                double bearing = headingOf(dx, dy);
                // Drive backwards if that needs less turning than facing the target.
                boolean reverse = Math.abs(wrap(bearing + 180 - poseHeading)) < Math.abs(wrap(bearing - poseHeading));
                if (distance < 2.0) {
                    // Too close to bother turning: just drive along the current heading.
                    // driveDistance = how far ahead of us the target is (negative = behind us).
                    targetHeading = poseHeading;
                    driveDistance = dx * -Math.sin(Math.toRadians(poseHeading)) + dy * Math.cos(Math.toRadians(poseHeading));
                    turning = false;
                    startLeg();
                } else {
                    // Face the target (or face away from it when reversing), then drive.
                    targetHeading = reverse ? wrap(bearing + 180) : bearing;
                    driveDistance = reverse ? -distance : distance;
                    turning = true;
                }
                break;
            }
            case AIM_AT: {
                // Turn so the launcher points at the target. With the launcher on the back,
                // the robot's front must face the opposite way. No driving.
                double tx = fieldX(s.x, s.y), ty = fieldY(s.x, s.y);
                double bearing = headingOf(tx - poseX, ty - poseY);
                targetHeading = LAUNCHER_ON_BACK ? wrap(bearing + 180) : bearing;
                turning = true;
                driveDistance = 0;
                break;
            }
            case LAUNCH:
                // Start spinning the launcher up right away.
                launcherOn = true;
                break;
            default:
                break;
        }
    }

    // Called when a step finishes: clean up after it, then start the next step.
    // After a LAUNCH, turn the launcher off.
    private void nextStep() {
        Step s = path.get(stepIndex);
        if (s.kind == Kind.LAUNCH) launcherOn = false;
        stepIndex++;
        beginStep();
    }

    // End of the auto: mark it done, stop every motor and servo, and put the drive motors back in
    // plain power mode (RUN_WITHOUT_ENCODER). After this, loop() only updates pose and telemetry.
    private void finish() {
        done = true;
        launcherOn = false;
        robot.stopAll();
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Find the step with this label. Returns its place in the path, or -1 if there is none.
    private int indexOf(String label) {
        for (int i = 0; i < path.size(); i++) {
            if (label.equals(path.get(i).label)) return i;
        }
        return -1;
    }

    /*
     * Turn (and for DRIVE_TO, then drive). Writes left/right power into out[]. Returns true when
     * the step is finished.
     */
    private boolean runMove(Step s, double dt, double[] out) {
        // TURN PART. error = degrees still to turn, kept between -180 and 180
        // (positive = turn left / counter-clockwise).
        if (turning) {
            double error = wrap(targetHeading - poseHeading);
            // Power is proportional to the error, but at least TURN_MIN_POWER so the robot doesn't
            // stall just short. Inside the tolerance, stop and let the settle timer run.
            double power = clamp(TURN_KP * error, -TURN_MAX_POWER, TURN_MAX_POWER);
            if (Math.abs(error) > TURN_TOLERANCE_DEG) {
                power = Math.copySign(Math.max(Math.abs(power), TURN_MIN_POWER), power);
                settleTimer.reset();
            } else {
                power = 0;
            }
            // Left wheels back + right wheels forward spins the robot left (when power > 0).
            out[0] = -power;
            out[1] = power;
            // Wait until we have stayed on heading for SETTLE_SECONDS.
            if (settleTimer.seconds() < SETTLE_SECONDS) return false;

            // AIM_AT is finished now. DRIVE_TO goes on to the drive part.
            if (s.kind != Kind.DRIVE_TO) return true;
            // Re-measure the distance from where the turn actually ended.
            // (Keep the sign: negative means drive backwards.)
            double tx = fieldX(s.x, s.y), ty = fieldY(s.x, s.y);
            double distance = Math.hypot(tx - poseX, ty - poseY);
            driveDistance = driveDistance < 0 ? -distance : distance;
            turning = false;
            startLeg();
            out[0] = out[1] = 0;
            return false;
        }

        // DRIVE PART. How far we have gone since this straight drive started = the average of
        // the two wheel encoders, turned into inches.
        double traveled = ((robot.leftDrive.getCurrentPosition() - legLeftTicks)
                + (robot.rightDrive.getCurrentPosition() - legRightTicks)) / 2.0 / DRIVE_TICKS_PER_IN;
        double remaining = driveDistance - traveled;
        // Power is proportional to the distance left (so we slow down near the end), capped at
        // the max, but at least DRIVE_MIN_POWER until we are within tolerance.
        double wanted = clamp(DRIVE_KP * remaining, -DRIVE_MAX_POWER, DRIVE_MAX_POWER);
        if (Math.abs(remaining) > DRIVE_TOLERANCE_IN) {
            wanted = Math.copySign(Math.max(Math.abs(wanted), DRIVE_MIN_POWER), wanted);
            settleTimer.reset();
        } else {
            wanted = 0;
        }
        // Ramp up gently; slowing down is never limited.
        // Power may only grow by DRIVE_RAMP_PER_S each second (step = the most it can grow now).
        double step = DRIVE_RAMP_PER_S * dt;
        if (Math.abs(wanted) > Math.abs(commandedPower)) {
            commandedPower = clamp(wanted, commandedPower - step, commandedPower + step);
        } else {
            commandedPower = wanted;
        }

        // Heading hold: if the robot drifts off the heading it turned to, speed up one side and
        // slow the other to steer back. No steering while stopped.
        double steer = (commandedPower == 0) ? 0 : HEADING_KP * wrap(targetHeading - poseHeading);
        double left = commandedPower - steer;
        double right = commandedPower + steer;
        // If either side would go past full power (1), shrink both sides in proportion so the
        // steering still works.
        double scale = Math.max(1.0, Math.max(Math.abs(left), Math.abs(right)));
        out[0] = left / scale;
        out[1] = right / scale;
        // Finished once we have stayed within tolerance for SETTLE_SECONDS.
        return settleTimer.seconds() >= SETTLE_SECONDS;
    }

    // Start the straight-drive part: remember the encoder counts here and start from zero power.
    private void startLeg() {
        legLeftTicks = robot.leftDrive.getCurrentPosition();
        legRightTicks = robot.rightDrive.getCurrentPosition();
        commandedPower = 0;
        settleTimer.reset();
    }

    // ----------------------------------------------------------------------------------------
    // Pose and geometry
    // ----------------------------------------------------------------------------------------

    /*
     * Odometry: work out the robot's new pose from how far each wheel moved since the last loop.
     * Heading comes from the IMU if we have one. Otherwise it comes from the difference between
     * the two wheels (if the right wheels went farther than the left, the robot turned left).
     */
    private void updatePose() {
        // How far each side moved since the last loop, in inches.
        int left = robot.leftDrive.getCurrentPosition();
        int right = robot.rightDrive.getCurrentPosition();
        double dLeft = (left - lastLeftTicks) / DRIVE_TICKS_PER_IN;
        double dRight = (right - lastRightTicks) / DRIVE_TICKS_PER_IN;
        lastLeftTicks = left;
        lastRightTicks = right;

        // IMU: heading = start heading + how far the IMU says we have turned (yaw) since START.
        // No IMU: angle turned since START (in radians) = (right distance - left distance) /
        // track width. (Radians are another unit for angles; Math.toDegrees() turns them into
        // degrees.)
        double previousHeading = poseHeading;
        if (imu != null) {
            poseHeading = wrap(startHeading + imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        } else {
            double turnedRad = ((right - startRightTicks) - (left - startLeftTicks)) / DRIVE_TICKS_PER_IN / TRACK_WIDTH_IN;
            poseHeading = wrap(startHeading + Math.toDegrees(turnedRad));
        }
        // Move the pose by the average wheel distance, in the direction halfway between the old
        // and new heading (more accurate when the robot drives and turns at the same time).
        // With heading 0 = +Y, "forward" is the direction (-sin(heading), +cos(heading)).
        double midHeading = Math.toRadians(previousHeading + wrap(poseHeading - previousHeading) / 2.0);
        double d = (dLeft + dRight) / 2.0;
        poseX += -d * Math.sin(midHeading);
        poseY += d * Math.cos(midHeading);
    }

    /**
     * RED-frame heading for the launcher to point along direction (dx, dy). With the launcher on
     * the back, the robot's front faces the opposite way (+180 degrees).
     */
    private static double launcherHeadingToward(double dx, double dy) {
        double h = headingOf(dx, dy);
        return LAUNCHER_ON_BACK ? wrap(h + 180) : h;
    }

    /**
     * Heading (0 = +Y, CCW positive) of the direction (dx, dy). For example +X gives -90 and -X
     * gives +90. atan2 is the math function that turns a direction into an angle.
     */
    private static double headingOf(double dx, double dy) {
        return Math.toDegrees(Math.atan2(-dx, dy));
    }

    // RED-frame values to this match's frame: BLUE is RED rotated 180 degrees about the center.
    // The field looks the same after a half turn around its center, so a RED point (x, y) is
    // (-x, -y) for BLUE, and every heading turns by 180 degrees. For RED nothing changes.
    private double fieldX(double x, double y) { return alliance == Alliance.BLUE ? -x : x; }
    private double fieldY(double x, double y) { return alliance == Alliance.BLUE ? -y : y; }
    private double fieldHeading(double h) { return alliance == Alliance.BLUE ? wrap(h + 180) : h; }

    // Keep an angle between -180 and 180 degrees (for example 270 becomes -90), so the robot
    // always turns the short way around.
    private static double wrap(double degrees) {
        while (degrees > 180) degrees -= 360;
        while (degrees <= -180) degrees += 360;
        return degrees;
    }

    // Keep v between lo and hi.
    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
