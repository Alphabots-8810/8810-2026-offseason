package frc.robot.commands.ShootingCommand;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;

public final class ShootingConstants {
  // Feed speeds applied once we transition into the SHOOT state.
  public static final LoggedTunableNumber IndexerRotpsTunable =
      new LoggedTunableNumber("Shooting/IndexerRotps", 40);
  public static final LoggedTunableNumber IntakeRollerRotpsTunable =
      new LoggedTunableNumber("Shooting/IntakeRollerRotps", 40);
  public static final LoggedTunableNumber FeederRotpsTunable =
      new LoggedTunableNumber("Shooting/FeederRotps", 40);

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

    // distance(m) -> shooter (rot/s)
 
    // distance(m) -> hood (deg)

  }

  private ShootingConstants() {}
}
