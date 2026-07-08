package frc.robot.commands.ShootingCommand;

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

  // Once SHOOT starts and the CANrange reports a long-distance reading, wait this long (seconds)
  // before retracting the intake.
  public static final LoggedTunableNumber RETRACT_DELAY_SEC =
      new LoggedTunableNumber("Shooting/retract_delay", 0.1);
  // Intake deploy position to retract to, in centimeters.
  public static final LoggedTunableNumber INTAKE_RETRACT_POSITION_CM =
      new LoggedTunableNumber("Shooting/retract", 45);

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

  static {
    // PLACEHOLDER TABLE - tune on the real field. Only 2.118 m and 2.540 m are measured;
    // the rest are interpolated/extrapolated guesses so the maps cover the full range.
    // Both maps must stay monotonic in distance.
    /*
     * 2.118 0 47.5
     * 2.54  5  49.08
     * 3.05  12 51.08
     * 3.53 16 53.26
     * 4.15 20.5 56.
     * 4.55 23 58
     * 5.06 25 61
     *
     */
    distanceToShooterRotps.put(2.118, 49.5);
    distanceToShooterRotps.put(2.54, 51.08);
    distanceToShooterRotps.put(3.05, 53.6);
    distanceToShooterRotps.put(3.53, 55.26);
    distanceToShooterRotps.put(4.15, 58.);
    distanceToShooterRotps.put(4.55, 61.2);
    distanceToShooterRotps.put(5.06, 64.2);
    distanceToHoodDeg.put(2.118, 0.);
    distanceToHoodDeg.put(2.54, 5.);
    distanceToHoodDeg.put(3.05, 12.);
    distanceToHoodDeg.put(3.53, 16.);
    distanceToHoodDeg.put(4.15, 20.5);
    distanceToHoodDeg.put(4.55, 23.);
    distanceToHoodDeg.put(5.06, 25.);
    // distance(m) -> shooter (rot/s)

    // distance(m) -> hood (deg)

    distanceToFeedVelo.put(2., 100.);
    distanceToFeedVelo.put(4., 100.);
    distanceToFeedVelo.put(5., 80.);
  }

  private ShootingConstants() {}
}
