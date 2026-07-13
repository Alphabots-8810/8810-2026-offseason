# CLAUDE.md

FRC Team 8810 offseason robot code for the **2026 game "REBUILT"**. Java 17, WPILib command-based, built on the AdvantageKit TalonFX swerve template.

## Game context (2026 REBUILT)

- **FUEL** — the game piece: small foam balls, handled in bulk (intake → hopper → indexer → shooter).
- **HUB** — the central scoring goal on each alliance side. Blue hub at (4.623, 4.030) m, red at (11.917, 4.030) m (`Constants.FieldConstants`).
- **TRENCH** — a low tunnel structure robots can drive under to cross between field zones. There are four trench openings: two X positions (blue x≈4.625 m, red x≈11.915 m, structure centers) × two Y positions (y≈0.666 m "down", y≈7.403 m "up"). The structure is ~47 in (1.194 m) deep along X; sim collision box is 53 in. See `AutoTrenchCommandConstants` — those numbers come from maple-sim's `Arena2026Rebuilt` collision geometry.
- **Ferrying** — when on the far (neutral) side of the HUB, lob fuel toward the alliance side instead of shooting (`Ferry` command; `FieldLayout.isPoseInAllianceArea` decides shoot vs ferry).
- Field: 54 ft × 26 ft. All poses are stored blue-origin; `FieldLayout.handleAllianceFlip` rotates 180° about field center for red.

## Build & workflow

```
./gradlew build          # compile + spotless check
./gradlew spotlessApply  # auto-format (Google Java Format) — run before committing
./gradlew deploy         # deploy to roboRIO
./gradlew simulateJava   # desktop sim (maple-sim physics; sim GUI disabled by default for replay support)
./gradlew replayWatch    # AdvantageKit log replay
```

CI (`.github/workflows/build.yml`) runs `./gradlew build`, which fails on unformatted code.

## Architecture

### Three runtime modes (AdvantageKit)

`Constants.currentMode` = `REAL` / `SIM` / `REPLAY`. Every subsystem picks its IO implementation with a switch on this mode; `REPLAY` uses empty IO interfaces so logs can be replayed deterministically.

### Hardware abstraction layer — `frc.lib`

Team-generic, game-agnostic code. **No `frc.robot` imports should leak in here.**

- `frc.lib.io.MotorIO` — universal motor interface (voltage / velocity / position / torque-current control, Motion Magic constraints). Implementations: `MotorIOPhoenix6` (TalonFX, config-driven), `MotorIOSim` (WPILib physics sim). Inputs are logged in **radians**; the subsystem-facing API is in **rotations**.
- `frc.lib.io.SensorIO` — boolean proximity sensor with debounce. Implementations: `SensorIOCANRange` (CTRE CANrange), `SensorIOSim`.
- `frc.lib.io.vision.CameraIO` — vision interface with Limelight (`LimelightHelpers`-based) and PhotonVision implementations.
- `frc.lib.bases.MotorSubsystem` — base class wrapping one `MotorIO`: periodic input logging, disconnected alert, rot-unit getters/setters, `isAtSetpoint()`. Most mechanism subsystems are thin subclasses of this.
- `frc.lib.bases.CameraSubsystem` — same idea for cameras.
- `frc.lib.simulation` — maple-sim glue (swerve drivetrain, fuel/game-piece simulation, arena).

### Subsystems — `frc.robot.subsystems`

**Singleton pattern**: every subsystem exposes `public static final X mInstance` with a private constructor. `RobotContainer` holds a field reference to each one purely to force class-init at boot — otherwise the motor config (gear ratio, PID, current limits) is only flashed on first button press. **If you add a subsystem, add its `mInstance` reference to `RobotContainer`.**

Each subsystem folder pairs `X.java` with `XConstants.java` (CAN IDs, gains, sim config).

| Subsystem | Role |
|---|---|
| `drive/Drive` | AdvantageKit TalonFX swerve (Pigeon2, 250 Hz odometry thread, PathPlanner integration, SysId routines). `TunerConstants` is CTRE Tuner X generated. |
| `Drum` | Shooter drum/flywheel |
| `Hood` | Shot-angle hood (zeroed against hard stop via `HoodZeroCommand`) |
| `Feeder`, `Indexer` | Move fuel from hopper to shooter |
| `IntakeDeploy`, `IntakeRoller` | Deployable intake (position-controlled arm + roller) |
| `FeedPath` | Sensor-only subsystem: two CANrange sensors (hopper + ball tunnel) reporting `HopperFilled()` / `IndexerFilled()` |
| `vision/Cameras` | Vision pose estimates fed into drive odometry. Must initialize **after** `Drive.mInstance` is set — its periodic reads the drive pose. |

### Commands — `frc.robot.commands`

One folder per command, each with its own `*Constants.java`:

- `ShootingCommand/Shooting` — aim + spin up + feed at the HUB (moving-while-shooting takes joystick suppliers).
- `FerryCommand/Ferry` — lob from neutral zone. `RobotContainer.shootOrFerryCommand()` defers between the two based on pose.
- `AutoTrenchCommand` — driver-assist trench pass: PID aligns Y + heading to the closest trench (ALIGN state), gates X speed near the entry face, then lets the driver drive through (PASS state). Retargets to the closest trench every cycle.
- `AutopilotTrenchCommand` — fully automatic trench pass using the [Autopilot library](https://github.com/therekrab/autopilot). The handoff adapts to the next Choreo path's ideal starting state: path starts at rest → settle at its start pose; starts moving past the trench exit → PASS carries that velocity and finishes crossing the handoff plane; starts moving *before the entrance* (the path drives the trench itself) → APPROACH targets the path start directly at its start velocity and hands off there. Used in autonomous.
- `AutoCommands/BlockAutoBuilder` — dashboard-configurable "building block" auto: scans `deploy/choreo` for `<Side><Section><Variant>.traj` (e.g. `Left1InnerSwap.traj`), publishes side + per-section choosers, assembles path→shoot→trench-pass sequences at auto start.
- `IntakeCommand`, `AutoalignIntakeCommand` — intake with optional auto-align to fuel.
- `HoodZeroCommand`, `IntakeDeployOutwardZeroCommand` — current-spike homing against hard stops.
- `DriveCommands` — field-relative joystick drive, wheel-radius / feedforward characterization.

### Logging & tuning conventions

- All IO inputs go through `Logger.processInputs(...)`; command internals via `Logger.recordOutput("CommandName/...")`.
- `LoggedTunableNumber` for values tuned live from AdvantageScope/Elastic.
- **Sim does not exercise real-robot control gains** — `MotorIOSim` has its own sim config, so PID tuned in sim says nothing about the real mechanism. Real tuning is done from AdvantageScope log review.

## Key files

- `RobotContainer.java` — subsystem wiring, button bindings, auto chooser.
- `Constants.java` — mode switch + hub locations. `FieldLayout.java` — field geometry + alliance flipping helpers.
- `generated/TunerConstants.java` — swerve hardware constants (regenerate with Tuner X, don't hand-edit casually).
- `vendordeps/` — AdvantageKit, Phoenix6, PathplannerLib, photonlib, maple-sim, Autopilot, Studica (all frcYear 2026).

## Reference links

- **AdvantageKit docs** (template this project is based on): https://docs.advantagekit.org — see the TalonFX Swerve Template pages for `Drive`/`Module` structure.
- **Team 6328 Mechanical Advantage** (origin of AdvantageKit + the IO-layer pattern): https://github.com/Mechanical-Advantage — their yearly `RobotCode20XX` repos are the canonical examples.
- **Team 1678 Citrus Circuits** (reference for superstructure/state-machine and per-mechanism-constants style): https://github.com/frc1678 — e.g. their public season code repos (`C2024`, etc.).
- **maple-sim** (physics sim + `Arena2026Rebuilt` field/trench collision geometry): https://github.com/Shenzhen-Robotics-Alliance/maple-sim
- **Autopilot** (trench auto-pass motion library): https://github.com/therekrab/autopilot
- **PathPlanner**: https://pathplanner.dev — **Choreo** (trajectories in `deploy/choreo`): https://choreo.autos
- **CTRE Phoenix 6 docs** (TalonFX, CANrange, Motion Magic): https://v6.docs.ctr-electronics.com
- **2026 game manual (REBUILT)** — official TRENCH/HUB/FUEL definitions and dimensions: https://www.firstinspires.org/robotics/frc/game-and-season
