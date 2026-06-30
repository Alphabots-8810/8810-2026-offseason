package frc.robot.commands.FerryCommand;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.Units;

public class FerryConstants {
  // Depot-Side Alliance Corner
  public static final Translation2d kBlueLeftFerry =
      new Translation2d(Units.Meters.of(0.5), Units.Feet.of(18.5));
  // Outpost-Side Alliance Corner
  public static final Translation2d kBlueRightFerry =
      new Translation2d(Units.Meters.of(0.5), Units.Feet.of(6.0));
  // Depot-Side Alliance Trench
  public static final Translation2d kBlueLeftNeutralFerry =
      new Translation2d(Units.Meters.of(5.5), Units.Feet.of(18.0));
  // Outpost-Side Alliance Trench
  public static final Translation2d kBlueRightNeutralFerry =
      new Translation2d(Units.Meters.of(5.5), Units.Feet.of(9.0));
  // Depot-Side Alliance Trench
  public static final Translation2d kBlueLeftFarAllianceFerry =
      new Translation2d(Units.Meters.of(0.5), Units.Feet.of(20.0));
  // Outpost-Side Alliance Trench
  public static final Translation2d kBlueRightFarAllianceFerry =
      new Translation2d(Units.Meters.of(0.5), Units.Feet.of(5.0));

  public static final InterpolatingDoubleTreeMap FerryDistanceToDrumRotps =
      new InterpolatingDoubleTreeMap();
  public static final InterpolatingDoubleTreeMap FerryDistanceToHoodAngle =
      new InterpolatingDoubleTreeMap();

  static {
    FerryDistanceToDrumRotps.put(Double.MIN_VALUE, 50.0);
    FerryDistanceToDrumRotps.put(5.22, 50.0);
    FerryDistanceToDrumRotps.put(8.03, 60.0);
    FerryDistanceToDrumRotps.put(10.03, 75.0);
    FerryDistanceToDrumRotps.put(Double.MAX_VALUE, 75.0);

    FerryDistanceToDrumRotps.put(Double.MIN_VALUE, 40.0);
    FerryDistanceToHoodAngle.put(5.22, 40.0);
    FerryDistanceToHoodAngle.put(8.03, 45.0);
    FerryDistanceToHoodAngle.put(10.03, 49.5);
    FerryDistanceToHoodAngle.put(Double.MAX_VALUE, 49.5);
  }
}
