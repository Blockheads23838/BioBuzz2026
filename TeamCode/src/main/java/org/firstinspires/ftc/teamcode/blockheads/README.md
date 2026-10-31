# blockheads/: new StarterBot programs (drafts)

New programs for the goBILDA 6WD StarterBot, kept in their **own folder** so they don't change anything that's already here.
- `BioBuzzStarterBotTeleop` and `StarterBotAuto` are untouched.
- No SDK or Gradle files are changed.

**Status: drafts. They compile against FTC SDK 12.0 but have NOT been run on the real robot yet.**

| File | Shows on the Driver Station as | What it does |
|---|---|---|
| `StarterBotHardware.java` | (not an OpMode) | All 7 device names, directions and launcher settings in one place, so programs don't copy-paste them |
| `BHBenchCheck.java` | **BH Bench Check** (TeleOp, group "BH Dev") | Robot-day test. It checks the config names, that nothing moves after INIT, the encoders, the launcher speed sign and spin-up time, the gyro, and which end shoots |
| `LauncherTuner.java` | **Dev: Launcher Tuner** (TeleOp, group "Dev") | Change the launcher speed and PIDF live with the dpad and bumpers; shows speed, error and spin-up time |
| `BlockheadsAutoLaunchPark.java` | BH Auto: Launch + Park (**hidden** with `@Disabled`) | Simple auto: shoot the 4 pre-loads, back off the wall, park. Parking distances must be measured first |
| `BlockheadsAutoStarterBot.java` | **BH Auto: StarterBot** (**hidden** with `@Disabled`) | **The rebuilt full auto (use this one).** Press **A** in INIT to confirm the launcher points at the HIVE. Then it backs off the wall, aims (a small ~6° turn), shoots for 4 s, and drives a square path along our side into the **middle** of the LOADING ZONE. Without A it only backs off the wall. Passed red, blue, unconfirmed and backwards-placement tests in the simulator. |
| `BlockheadsAutoFull.java` | BH Auto: Full (**hidden** with `@Disabled`) | Shoots from the **audience wall** (in front of the HIVE's upward cell), then leaves the wall and parks. Has an alliance/start menu and gyro turns. Tested in the virtual_robot simulator (driving, timing and parking only) |

## Before using on the robot

1. The Robot Configuration needs: `left_drive`, `right_drive`, `intake`, `launcher` (encoder plugged in), `windmill`, `left_intake_servo`, `right_intake_servo`, and `imu`.
2. Switch all 3 goBILDA Dual Mode servos to **continuous** mode, or they jump at INIT.
3. Run **BH Bench Check** first, with the robot on blocks.
4. Remove `@Disabled` from an auto only after it passes its tests. The first runs go on blocks, then on the floor at low power.

Every file has plain-language comments every few blocks, written for new team coders.
