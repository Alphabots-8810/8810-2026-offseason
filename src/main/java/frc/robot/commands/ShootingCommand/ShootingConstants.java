package frc.robot.commands.ShootingCommand;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;

public final class ShootingConstants {
  // Feed speeds applied once we transition into the SHOOT state.
  public static final LoggedTunableNumber IndexerRotpsTunable =
      new LoggedTunableNumber("Shooting/IndexerRotps", 95);
  public static final LoggedTunableNumber IntakeRollerRotpsTunable =
      new LoggedTunableNumber("Shooting/IntakeRollerRotps", 40);
  public static final LoggedTunableNumber FeederRotpsTunable =
      new LoggedTunableNumber("Shooting/FeederRotps", 95);

  // Motion compensation tuning. Ball speed is estimated from flywheel speed so the command can
  // calculate a distance-based flight time, then clamp it to a realistic range.
//   public static final LoggedTunableNumber BallSpeedMpsPerShooterRotpsTunable =
//       new LoggedTunableNumber("Shooting/MotionComp/BallSpeedMpsPerShooterRotps", 0.09);
//   public static final LoggedTunableNumber MinFlightTimeSecTunable =
//       new LoggedTunableNumber("Shooting/MotionComp/MinFlightTimeSec", 0.15);
//   public static final LoggedTunableNumber MaxFlightTimeSecTunable =
//       new LoggedTunableNumber("Shooting/MotionComp/MaxFlightTimeSec", 0.75);
//   public static final LoggedTunableNumber CompensationScaleTunable =
//       new LoggedTunableNumber("Shooting/MotionComp/CompensationScale", 1.0);

  // Readiness tolerances used to decide when AIM is satisfied and we may shoot.
  public static final double SHOOTER_VELOCITY_TOLERANCE_ROTPS = 1.0;
  public static final double HOOD_ANGLE_TOLERANCE_ROT = 0.02;
  public static final double AIM_ANGLE_TOLERANCE_RAD = Units.degreesToRadians(2.0);

  // Distance (meters, robot to HUB) -> shooter flywheel velocity (rotations / sec).
  public static final InterpolatingDoubleTreeMap distanceToShooterRotps =
      new InterpolatingDoubleTreeMap();

  // Distance (meters, robot to HUB) -> hood angle (degrees).
  public static final InterpolatingDoubleTreeMap distanceToHoodDeg =
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
    distanceToShooterRotps.put(2.118, 47.5);
    distanceToShooterRotps.put(2.54, 49.08);
    distanceToShooterRotps.put(3.05, 51.08);
    distanceToShooterRotps.put(3.53, 53.26);
    distanceToShooterRotps.put(4.15, 56.);
    distanceToShooterRotps.put(4.55, 58.);
    distanceToShooterRotps.put(5.06, 61.);
    distanceToHoodDeg.put(2.118, 0.);
    distanceToHoodDeg.put(2.54, 5.);
    distanceToHoodDeg.put(3.05, 12.);
    distanceToHoodDeg.put(3.53, 16.);
    distanceToHoodDeg.put(4.15, 20.5);
    distanceToHoodDeg.put(4.55, 23.);
    distanceToHoodDeg.put(5.06, 25.);
    // distance(m) -> shooter (rot/s)

    // distance(m) -> hood (deg)

  }

  private ShootingConstants() {}
}
