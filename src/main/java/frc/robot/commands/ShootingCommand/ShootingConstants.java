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

  }

  private ShootingConstants() {}
}
