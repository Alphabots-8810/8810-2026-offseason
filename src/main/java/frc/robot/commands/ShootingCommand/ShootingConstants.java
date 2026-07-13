package frc.robot.commands.ShootingCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;

public final class ShootingConstants {
  // Feed speeds applied once we transition into the SHOOT state.
  public static final LoggedTunableNumber IndexerRotpsTunable =
      new LoggedTunableNumber("Shooting/IndexerRotps", 80);
  public static final LoggedTunableNumber IntakeRollerRotpsTunable =
      new LoggedTunableNumber("Shooting/IntakeRollerRotps", 40);
  public static final LoggedTunableNumber FeederRotpsTunable =
      new LoggedTunableNumber("Shooting/FeederRotps", 80);

  // Readiness tolerances used to decide when AIM is satisfied and we may shoot.
  public static final double SHOOTER_VELOCITY_TOLERANCE_ROTPS = 1.;
  public static final double HOOD_ANGLE_TOLERANCE_ROT = 0.02;
  public static final double AIM_ANGLE_TOLERANCE_RAD = Units.degreesToRadians(15.0);

  // AIM -> SHOOT fires only after everything has been continuously ready for this long,
  // so a single lucky velocity sample inside the tolerance band cannot start the volley.
  public static final double READY_DEBOUNCE_SEC = 0.15;

  // Left-stick deadband for repositioning during AIM. Matches teleop drive deadband.
  public static final double DRIVE_DEADBAND = 0.03;

  // Chassis must be below this measured linear speed (m/s) to count as stopped for shooting.
  public static final double CHASSIS_STOP_VELOCITY_MPS = 0.08;

  // Inside this heading error the omega command is zeroed. Errors this small produce omega
  // commands below what the swerve can execute; static friction turns them into a ~5 Hz
  // limit-cycle wiggle. 1 deg of heading is ~9 cm lateral at 5 m, well inside the HUB.
  public static final double AIM_ERROR_DEADBAND_RAD = Units.degreesToRadians(1.0);

  // Aim heading controller (profiled). Gains carry over from the pre-profile aim controller
  // (verified: no overshoot, well damped); the trapezoid constraints are inherited from the
  // field-proven teleop snap-to-angle controller in DriveCommandsConstants as a first guess.
  public static final double AIM_KP = 5.0;
  // 0.4 damps the pass-through at the goal (same value as the teleop snap controller).
  public static final double AIM_KD = 0.4;
  public static final double AIM_MAX_VELOCITY_RAD_PER_SEC = 8.0;
  // aimtest2: planned 20 rad/s^2 decel but the chassis only achieved ~17.5 (0.35 rad
  // overshoot from 3.5 rad/s). 10 leaves headroom so the profile brakes early enough.
  public static final double AIM_MAX_ACCELERATION_RAD_PER_SEC2 = 10.0;

  // Indexer jam detection during SHOOT/RETRACT: stator current continuously above this threshold
  // for the debounce time means a ball is stalled against the indexer (free-running feed draws far
  // less; the config's stator limit is 100 A so a true stall does reach 90). NOTE: only the normal
  // velocity-closed-loop feed can trip this — energy-save mode drives the indexer at 50 A torque
  // current, which caps stator current below the threshold.
  public static final double INDEXER_JAM_CURRENT_AMPS = 90.0;
  public static final double INDEXER_JAM_DEBOUNCE_SEC = 0.2;
  // Unjam response: run the indexer backwards at this speed for this long, then resume the volley.
  public static final LoggedTunableNumber UNJAM_INDEXER_ROTPS =
      new LoggedTunableNumber("Shooting/UnjamIndexerRotps", 40);
  public static final LoggedTunableNumber UNJAM_DURATION_SEC =
      new LoggedTunableNumber("Shooting/UnjamDurationSec", 0.5);

  // Once SHOOT starts and the CANrange reports a long-distance reading, wait this long (seconds)
  // before retracting the intake.
  public static final LoggedTunableNumber RETRACT_DELAY_SEC =
      new LoggedTunableNumber("Shooting/retract_delay", 0.2);
  // Intake deploy position to retract to, in centimeters.
  public static final LoggedTunableNumber INTAKE_RETRACT_POSITION_CM =
      new LoggedTunableNumber("Shooting/retract", 35);

  // Distance (meters, robot to HUB) -> shooter flywheel velocity (rotations / sec).
  public static final InterpolatingDoubleTreeMap distanceToShooterRotps =
      new InterpolatingDoubleTreeMap();

  // Distance (meters, robot to HUB) -> hood angle (degrees).
  public static final InterpolatingDoubleTreeMap distanceToHoodDeg =
      new InterpolatingDoubleTreeMap();

  // indexer perimeter  131.88mm
  // feeder perimeter 120mm
  public static final InterpolatingDoubleTreeMap distanceToFeedVelo =
      new InterpolatingDoubleTreeMap();

  // Alpha Sim V3.1 projectile fit (projectile sim/AlphaSim-main/handoff/lookup_polynomial.json,
  // regenerated 2026-07-08 at the measured 0.4935 m exit height):
  // launch elevation above the horizon (deg) = A*d^2 + B*d + C, d in meters, fit RMSE 0.11 deg.
  // Only valid on [1.5, 5.0] m — the fit is U-shaped and turns back up outside, so clamp.
  private static final double SIM_HOOD_A = 0.886320;
  private static final double SIM_HOOD_B = -10.544026;
  private static final double SIM_HOOD_C = 86.479696;
  private static final double SIM_DISTANCE_MIN_M = 1.5;
  private static final double SIM_DISTANCE_MAX_M = 5.0;

  // The hood mechanism zero is not horizontal: hood at 0 deg launches 78.2 deg above the
  // horizon, and increasing the hood angle flattens the shot.
  public static final double HOOD_ZERO_ELEVATION_DEG = 78.2;

  // The polynomial's distance is measured from the ball's EXIT POINT, which sits this far
  // in front of robot center (measured 2026-07-08). The robot aims straight at the HUB
  // while shooting, so the exit point is simply this much closer along the aim line.
  public static final double EXIT_FORWARD_OFFSET_M = 0.17641;

  // Measured exit height used by Alpha Sim V3.1 and maple-sim projectile launch.
  public static final double EXIT_HEIGHT_M = 0.4935;

  // Feeder wheel diameter (m); perimeter comment in static block uses 120 mm.
  public static final double FEEDER_WHEEL_DIAMETER_M = 0.120 / Math.PI;

  /** Feeder surface travel per motor rotation (m). */
  public static final double FEEDER_SURFACE_M_PER_MOTOR_ROT = Math.PI * FEEDER_WHEEL_DIAMETER_M;

  // Approximate spacing between balls in the feed path (m); one feeder revolution is a
  // reasonable first guess for sim launch cadence.
  public static final double FEED_BALL_SPACING_M = 0.120;

  /** Sim-predicted hood mechanism angle (deg) for a robot-center distance to the HUB (m). */
  public static double simHoodDeg(double distanceMeters) {
    double d =
        MathUtil.clamp(
            distanceMeters - EXIT_FORWARD_OFFSET_M, SIM_DISTANCE_MIN_M, SIM_DISTANCE_MAX_M);
    double elevationDeg = SIM_HOOD_A * d * d + SIM_HOOD_B * d + SIM_HOOD_C;
    return HOOD_ZERO_ELEVATION_DEG - elevationDeg;
  }

  // Same sim fit for the flywheel: SURFACE (rim) linear speed in m/s, with the 0.18
  // wheel-ball slip already included, on the same clamped exit-point distance.
  private static final double SIM_FLYWHEEL_A = 0.016406;
  private static final double SIM_FLYWHEEL_B = 0.714772;
  private static final double SIM_FLYWHEEL_C = 6.420549;

  // Drum geometry: 80 mm wheels, 20T motor pinion driving a 30T drum pulley. The drum's
  // rotor-to-mechanism ratio is configured as 1.0, so setVelocityRotPerSec commands MOTOR
  // rot/s: one motor rotation moves the wheel surface pi * 0.080 * (20/30) = 0.16755 m.
  public static final double DRUM_WHEEL_DIAMETER_M = 0.080;
  public static final double DRUM_MOTOR_TO_WHEEL_RATIO = 20.0 / 30.0;
  public static final double DRUM_SURFACE_M_PER_MOTOR_ROT =
      Math.PI * DRUM_WHEEL_DIAMETER_M * DRUM_MOTOR_TO_WHEEL_RATIO;

  // THE field-calibration knob from the sim handoff, applied to the drum speed at use time
  // (not baked into the table): balls landing SHORT -> raise, LONG -> lower. 1.0 = raw sim.
  // Field testing (2026-07-12) found the required correction GROWS with distance — the sim's
  // fixed 0.18 slip and drag model under-predict the far shots — so the single knob became a
  // two-point PURE linear line through (K_NEAR_DISTANCE_M, kSpeedNear) and
  // (K_FAR_DISTANCE_M, kSpeedFar), extrapolated without clamping on both sides, so the
  // defaults give 1.07 @ 2 m, 1.08 @ 3 m, 1.09 @ 4 m, 1.10 @ 5 m. Same tuning rule per
  // knob: balls SHORT at that distance -> raise, LONG -> lower.
  public static final double K_NEAR_DISTANCE_M = 2.0;
  public static final double K_FAR_DISTANCE_M = 3.0;
  public static final LoggedTunableNumber kSpeed =
      new LoggedTunableNumber("Shooting/kSpeedNear", 1.065);

  // Maple-sim projectile: launch speed (m/s) = drum target rot/s × this constant.
  public static final LoggedTunableNumber SimLaunchSpeedPerDrumRotps =
      new LoggedTunableNumber("Shooting/SimLaunchSpeedPerDrumRotps", 0.12);

  // Maple-sim projectile: launch elevation (deg) = 90° − hood target (deg) − this offset.
  // Default offset (90 − 78.2 = 11.8) reproduces hood=0 → 78.2° elevation from measurement.
  public static final LoggedTunableNumber SimLaunchAngleOffsetDeg =
      new LoggedTunableNumber("Shooting/SimLaunchAngleOffsetDeg", 90.0 - HOOD_ZERO_ELEVATION_DEG);

  /** Sim-predicted drum command (motor rot/s, before kSpeed) for a robot-center distance (m). */
  public static double simDrumRotps(double distanceMeters) {
    double d =
        MathUtil.clamp(
            distanceMeters - EXIT_FORWARD_OFFSET_M, SIM_DISTANCE_MIN_M, SIM_DISTANCE_MAX_M);
    double surfaceMps = SIM_FLYWHEEL_A * d * d + SIM_FLYWHEEL_B * d + SIM_FLYWHEEL_C;
    return surfaceMps / DRUM_SURFACE_M_PER_MOTOR_ROT;
  }

  static {
    // Both maps hold raw sim values (kSpeed = 1) at the same distance keys; Shooting
    // multiplies the drum speed by the distance-dependent kSpeed() ramp at use time. Keys
    // are robot-center distances; simDrumRotps/simHoodDeg subtract the exit offset
    // internally.
    distanceToShooterRotps.put(2.118, simDrumRotps(2.118)); // 46.97
    distanceToShooterRotps.put(2.54, simDrumRotps(2.54)); // 48.95
    distanceToShooterRotps.put(3.05, simDrumRotps(3.05)); // 51.39
    distanceToShooterRotps.put(3.53, simDrumRotps(3.53)); // 53.73
    distanceToShooterRotps.put(4.15, simDrumRotps(4.15)); // 56.82
    distanceToShooterRotps.put(4.55, simDrumRotps(4.55)); // 58.85
    distanceToShooterRotps.put(5.06, simDrumRotps(5.06)); // 61.49
    // Hood angles come from simHoodDeg() above (Alpha Sim polynomial converted to the
    // mechanism frame). Kept as a table so individual points can still be nudged on the
    // field; compare against the logged "Shooting/simHoodDeg" to see any hand edits.
    distanceToHoodDeg.put(2.118, simHoodDeg(2.118)); // 8.85
    distanceToHoodDeg.put(2.54, simHoodDeg(2.54)); // 11.69
    distanceToHoodDeg.put(3.05, simHoodDeg(3.05)); // 14.70
    distanceToHoodDeg.put(3.53, simHoodDeg(3.53)); // 17.11
    distanceToHoodDeg.put(4.15, simHoodDeg(4.15)); // 19.62
    distanceToHoodDeg.put(4.55, simHoodDeg(4.55)); // 20.88
    distanceToHoodDeg.put(5.06, simHoodDeg(5.06)); // 22.07
  }

  private ShootingConstants() {}
}
