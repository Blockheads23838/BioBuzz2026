package org.firstinspires.ftc.teamcode.blockheads;

// This "static import" lets us write just BRAKE instead of DcMotor.ZeroPowerBehavior.BRAKE.
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

// Pull in the FTC SDK pieces this file uses: motors, servos, the hardware list, PIDF numbers and
// the battery voltage sensors. (The SDK is the code library FIRST gives us to run the robot.)
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/*
 * StarterBotHardware: one place for every device name, direction and launcher constant on the
 * goBILDA StarterBot. This is a plain class, NOT an OpMode, so it never shows up on the Driver
 * Station. New OpModes create one and call init(hardwareMap) from their own init().
 *
 * Words used in this file:
 *   - OpMode: a robot program you pick and run from the Driver Station (a TeleOp or an Auto).
 *   - Driver Station: the phone/tablet app the drivers use to pick, start and stop OpModes.
 *   - init(): the OpMode method that runs once when the driver presses INIT (before START).
 *     This class also has its own init(hardwareMap), which an OpMode calls from its init().
 *   - hardwareMap: the SDK's list of every motor, servo and sensor in the robot's
 *     configuration. We look devices up in it by name.
 *
 * The robot has 7 devices: 2 drive motors (left side and right side), the intake roller motor,
 * the launcher motor (it spins a flywheel, a fast-spinning wheel that shoots the balls), the
 * windmill servo that feeds balls into the launcher, and 2 intake servos that help pull balls
 * out of corners and into the intake.
 *
 * The names, directions, BRAKE settings and launcher numbers are copied from goBILDA's
 * StarterBot example (BioBuzzStarterBotTeleop.java / StarterBotAuto.java, Base 10 Assets, LLC).
 * Those two files are left untouched; change a value HERE and every OpMode that uses this class
 * picks it up.
 *
 * RULE G304.H (a game manual rule): after the driver presses INIT, the robot must stay completely
 * still (nothing may move while it waits for START). init() only configures devices and writes
 * zero power. Do not add anything here that moves a motor or servo.
 *
 * DRAFT: compiled only, not yet run on the robot.
 */
public class StarterBotHardware {

    /*
     * Robot Configuration names. These must match the Control Hub configuration exactly
     * (case-sensitive, so "Intake" is not "intake"). A typo makes hardwareMap.get() throw an
     * error at INIT, and the OpMode will not run.
     * (The Control Hub is the robot's main control box. Its configuration is the saved list of
     * device names, usually set up from the Driver Station. Below, "the hub" means the control
     * box a motor is plugged into.)
     *
     * "public static final" means: a fixed value (it can never change while the code runs) that
     * any file can read, e.g. StarterBotHardware.LAUNCHER_NAME.
     */
    public static final String LEFT_DRIVE_NAME = "left_drive";
    public static final String RIGHT_DRIVE_NAME = "right_drive";
    public static final String INTAKE_NAME = "intake";
    public static final String LAUNCHER_NAME = "launcher";
    public static final String WINDMILL_NAME = "windmill";
    public static final String LEFT_INTAKE_SERVO_NAME = "left_intake_servo";
    public static final String RIGHT_INTAKE_SERVO_NAME = "right_intake_servo";

    /*
     * Directions. FORWARD or REVERSE picks which way a device spins when we give it positive
     * power. The left and right drive motors face opposite ways on the robot, so one of them is
     * set to REVERSE to make both sides roll forward together.
     * Pushing forward MUST drive the robot forward; flip the drive directions here if
     * the first test drive goes backwards.
     */
    public static final DcMotorSimple.Direction LEFT_DRIVE_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction RIGHT_DRIVE_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction INTAKE_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction WINDMILL_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction LEFT_INTAKE_SERVO_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction RIGHT_INTAKE_SERVO_DIRECTION = DcMotorSimple.Direction.REVERSE;

    /*
     * The goBILDA code never sets the launcher direction, so it runs at the SDK default (FORWARD).
     * We set it explicitly so behavior is the same but the choice is visible.
     * TODO(test): spin the launcher and check it throws the right way AND getVelocity() reads
     * POSITIVE. If the velocity reads negative, the PIDF loop fights itself: flip this to REVERSE.
     * (getVelocity() reads the launcher's measured speed. The PIDF loop is the speed controller
     * explained just below; if the measured speed has the wrong sign it pushes the wrong way.)
     */
    public static final DcMotorSimple.Direction LAUNCHER_DIRECTION = DcMotorSimple.Direction.FORWARD;

    /*
     * Launcher velocity control (RUN_USING_ENCODER). Velocities are encoder ticks per second.
     *
     * An encoder is a sensor on the motor that counts small steps ("ticks") as the shaft turns.
     * Ticks per second is how fast those counts go up, so it is our unit for speed: a bigger
     * number means the flywheel is spinning faster.
     *
     * RUN_USING_ENCODER tells the hub to watch the encoder and keep the motor at the speed we
     * ask for. The hub does this with a PIDF controller, a small formula that adjusts the power
     * many times a second:
     *   - P (proportional): adds extra power based on how far the speed is from the target.
     *   - F (feedforward): gives a base amount of power that grows with the target speed, so
     *     the motor gets close to the target even before P has to correct anything.
     *   - I and D are two more correction terms. They are 0 here, so they do nothing.
     *
     * The launcher only feeds once it is above LAUNCHER_MIN_VELOCITY (96% of target).
     * (1200 is 96% of 1250. OpModes like BlockheadsAutoLaunchPark check this number before
     * feeding, because a flywheel that is still too slow throws the ball short.
     * LauncherTuner uses its own +/-3% band instead of this number.)
     */
    public static final double LAUNCHER_P = 40;
    public static final double LAUNCHER_I = 0;
    public static final double LAUNCHER_D = 0;
    public static final double LAUNCHER_F = 12.5;
    public static final double LAUNCHER_TARGET_VELOCITY = 1250;
    public static final double LAUNCHER_MIN_VELOCITY = 1200;

    /*
     * Feeding powers, copied from the goBILDA launch() function: the windmill runs at full power
     * and the intake gets +0.5 to help push elements up.
     * (Power goes from -1.0 = full speed backward, through 0 = stopped, to 1.0 = full forward.)
     */
    public static final double WINDMILL_FEED_POWER = 1.0;
    public static final double INTAKE_FEED_POWER = 0.5;

    /*
     * Drivetrain geometry, copied from StarterBotAuto. 537.7 ticks/rev is goBILDA's published
     * output-shaft resolution for the 5203 19.2:1 (312 RPM) motor; the wheels are 96 mm.
     * (In plain words: the drive encoder counts 537.7 ticks for one full turn of the motor's
     * output shaft, and the math below treats one shaft turn as one wheel turn.)
     */
    public static final double WHEEL_DIAMETER_MM = 96;
    public static final double DRIVE_TICKS_PER_REV = 537.7;
    // Ticks per mm: how many encoder ticks equal 1 mm of driving. One wheel turn rolls the
    // robot one wheel circumference (diameter x pi, about 301.6 mm), and that turn is 537.7
    // ticks, so this works out to about 1.78 ticks per mm. Autonomous code (like
    // BlockheadsAutoLaunchPark) uses it to turn a distance in mm into an encoder target.
    public static final double DRIVE_TICKS_PER_MM = DRIVE_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI);
    // Track width: the side-to-side distance between the left and right wheels. Use it in turn
    // math: to spin in place, each side drives about (turn angle in radians) x (TRACK_WIDTH_MM / 2)
    // mm, one side forward and one side backward.
    public static final double TRACK_WIDTH_MM = 402;

    // Devices. Public so OpModes can use them directly, e.g. robot.launcher.getVelocity().
    // DcMotorEx = a DC motor with extra features, like setting and reading speed in ticks/s.
    // CRServo = a "continuous rotation" servo: it spins round and round like a small motor,
    // and setPower() picks its speed and direction (not a position).
    // They start as null (empty) and get filled in by init() below.
    public DcMotorEx leftDrive = null;
    public DcMotorEx rightDrive = null;
    public DcMotorEx intake = null;
    public DcMotorEx launcher = null;
    public CRServo windmill = null;
    public CRServo leftIntakeServo = null;
    public CRServo rightIntakeServo = null;

    // We keep hold of the hardware list here, so getBatteryVoltage() can use it later.
    private HardwareMap hardwareMap = null;

    /*
     * Call this ONCE from your OpMode's init(). It finds every device, sets directions and modes,
     * and writes zero power to everything. Nothing moves.
     */
    public void init(HardwareMap hardwareMap) {
        // Save the hardware list for later (getBatteryVoltage() needs it).
        this.hardwareMap = hardwareMap;

        // Find each device in the robot's configuration by its name and say what kind it is.
        leftDrive = hardwareMap.get(DcMotorEx.class, LEFT_DRIVE_NAME);
        rightDrive = hardwareMap.get(DcMotorEx.class, RIGHT_DRIVE_NAME);
        intake = hardwareMap.get(DcMotorEx.class, INTAKE_NAME);
        launcher = hardwareMap.get(DcMotorEx.class, LAUNCHER_NAME);
        windmill = hardwareMap.get(CRServo.class, WINDMILL_NAME);
        leftIntakeServo = hardwareMap.get(CRServo.class, LEFT_INTAKE_SERVO_NAME);
        rightIntakeServo = hardwareMap.get(CRServo.class, RIGHT_INTAKE_SERVO_NAME);

        // Tell each device which way counts as "forward" (the directions set at the top).
        leftDrive.setDirection(LEFT_DRIVE_DIRECTION);
        rightDrive.setDirection(RIGHT_DRIVE_DIRECTION);
        intake.setDirection(INTAKE_DIRECTION);
        launcher.setDirection(LAUNCHER_DIRECTION);
        windmill.setDirection(WINDMILL_DIRECTION);
        leftIntakeServo.setDirection(LEFT_INTAKE_SERVO_DIRECTION);
        rightIntakeServo.setDirection(RIGHT_INTAKE_SERVO_DIRECTION);

        // "Zero power behavior" is what a motor does when its power is set to 0.
        // BRAKE makes the robot stop quickly instead of coasting. The launcher's zero-power
        // behavior is left at the SDK default, the same as the goBILDA code.
        leftDrive.setZeroPowerBehavior(BRAKE);
        rightDrive.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        // Start every OpMode with the drive encoders at zero and the drive in plain power mode.
        // Autos switch to RUN_TO_POSITION themselves when they need it.
        // STOP_AND_RESET_ENCODER = stop the motor and set its tick count back to 0.
        // RUN_WITHOUT_ENCODER = setPower() sets the motor power directly (the encoder still
        // counts, but it is not used to control the motor).
        // RUN_TO_POSITION = the motor drives itself to a target tick count.
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Put the launcher in speed-control mode (RUN_USING_ENCODER, explained above) and
        // load our starting PIDF numbers.
        // The launcher needs its encoder cable plugged in for velocity control. If the wheel jumps
        // straight past the target, check the encoder cable first.
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setLauncherPIDF(LAUNCHER_P, LAUNCHER_I, LAUNCHER_D, LAUNCHER_F);

        // Set every motor and servo to zero power so nothing moves (rule G304.H).
        // Writing zero to the CR servos also initializes the servo controller.
        stopAll();
    }

    /*
     * Send new velocity PIDF coefficients to the hub. The launcher tuner calls this whenever P or
     * F changes.
     * (Coefficients are just the four P, I, D, F numbers. We set them for RUN_USING_ENCODER
     * because that is the speed-control mode the launcher runs in.)
     */
    public void setLauncherPIDF(double p, double i, double d, double f) {
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f));
    }

    /*
     * Zero power to every motor and servo. Safe to call at any time after init() has run
     * (before that the devices are still null), including from stop().
     * (stop() is the OpMode method that runs once when the driver presses STOP.)
     */
    public void stopAll() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        intake.setPower(0);
        launcher.setPower(0);
        windmill.setPower(0);
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
    }

    /*
     * Battery voltage in volts: the lowest non-zero reading from any hub, or 0 if none.
     * Launcher speed depends on the battery, so log this next to every tuning result.
     */
    public double getBatteryVoltage() {
        // Start "lowest" at infinity, so the first real reading is always smaller and replaces it.
        double lowest = Double.POSITIVE_INFINITY;
        // Check every voltage sensor the robot has and keep the smallest reading above 0.
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double volts = sensor.getVoltage();
            if (volts > 0) {
                lowest = Math.min(lowest, volts);
            }
        }
        // If no sensor gave a reading above 0, "lowest" is still infinity, so return 0 instead.
        return (lowest == Double.POSITIVE_INFINITY) ? 0 : lowest;
    }
}
