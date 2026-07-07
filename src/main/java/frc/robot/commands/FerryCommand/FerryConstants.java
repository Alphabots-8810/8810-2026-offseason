package frc.robot.commands.FerryCommand;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.Units;

public class FerryConstants {
  // Depot-Side Alliance Corner
  public static final Translation2d kBlueLeftFerry =
      new Translation2d(Units.Meters.of(1.5), Units.Feet.of(18.5));
  // Outpost-Side Alliance Corner
  public static final Translation2d kBlueRightFerry =
      new Translation2d(Units.Meters.of(1.5), Units.Feet.of(6.0));
  // Depot-Side Alliance Trench
  public static final Translation2d kBlueLeftNeutralFerry =
      new Translation2d(Units.Meters.of(1.5), Units.Feet.of(18.0));
  // Outpost-Side Alliance Trench
  public static final Translation2d kBlueRightNeutralFerry =
      new Translation2d(Units.Meters.of(1.5), Units.Feet.of(9.0));
  // Depot-Side Alliance Trench
  public static final Translation2d kBlueLeftFarAllianceFerry =
      new Translation2d(Units.Meters.of(1.5), Units.Feet.of(20.0));
  // Outpost-Side Alliance Trench
  public static final Translation2d kBlueRightFarAllianceFerry =
      new Translation2d(Units.Meters.of(1.5), Units.Feet.of(5.0));

  public static final InterpolatingDoubleTreeMap ferryDistanceToShooterRotps =
      new InterpolatingDoubleTreeMap();
  public static final InterpolatingDoubleTreeMap ferryDistanceToHoodDeg =
      new InterpolatingDoubleTreeMap();

  static {
    ferryDistanceToShooterRotps.put(2.118 * 1.4, 49.5);
    ferryDistanceToShooterRotps.put(2.54 * 1.4, 51.08);
    ferryDistanceToShooterRotps.put(3.05 * 1.4, 53.6);
    ferryDistanceToShooterRotps.put(3.53 * 1.4, 55.26);
    ferryDistanceToShooterRotps.put(4.15 * 1.4, 58.);
    ferryDistanceToShooterRotps.put(4.55 * 1.4, 61.2);
    ferryDistanceToShooterRotps.put(5.06 * 1.4, 64.2);

    ferryDistanceToHoodDeg.put(2.118 * 1.2, 0.);
    ferryDistanceToHoodDeg.put(2.54 * 1.2, 5.);
    ferryDistanceToHoodDeg.put(3.05 * 1.2, 12.);
    ferryDistanceToHoodDeg.put(3.53 * 1.2, 16.);
    ferryDistanceToHoodDeg.put(4.15 * 1.2, 20.5);
    ferryDistanceToHoodDeg.put(4.55 * 1.2, 23.);
    ferryDistanceToHoodDeg.put(5.06 * 1.2, 25.);
  }
}
