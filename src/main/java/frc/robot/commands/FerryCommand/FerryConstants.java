package frc.robot.commands.FerryCommand;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.Units;

public class FerryConstants {
  // Depot-Side Alliance Corner
  public static final Translation2d kBlueLeftFerry =
      new Translation2d(Units.Meters.of(0.5), Units.Feet.of(15.5));
  // Outpost-Side Alliance Corner
  public static final Translation2d kBlueRightFerry =
      new Translation2d(Units.Meters.of(0.5), Units.Feet.of(8.0));
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
    FerryDistanceToDrumRotps.put(Double.MIN_VALUE, 30.0);
    FerryDistanceToDrumRotps.put(4.977, 30.0);
    FerryDistanceToDrumRotps.put(6.902, 31.7);
    FerryDistanceToDrumRotps.put(8.477, 35.0);
    FerryDistanceToDrumRotps.put(9.029, 36.7);
    FerryDistanceToDrumRotps.put(9.590, 40.0);
    FerryDistanceToDrumRotps.put(10.061, 42.5);
    FerryDistanceToDrumRotps.put(12.460, 52.5);
    FerryDistanceToDrumRotps.put(13.133, 58.3);
    FerryDistanceToDrumRotps.put(Double.MAX_VALUE, 60.0);

    FerryDistanceToDrumRotps.put(Double.MIN_VALUE, 52.5);
    FerryDistanceToHoodAngle.put(4.977, 52.5);
    FerryDistanceToHoodAngle.put(6.902, 50.5);
    FerryDistanceToHoodAngle.put(8.477, 48.5);
    FerryDistanceToHoodAngle.put(9.029, 46.5);
    FerryDistanceToHoodAngle.put(9.590, 45.5);
    FerryDistanceToHoodAngle.put(10.061, 43.5);
    FerryDistanceToHoodAngle.put(12.460, 45.0);
    FerryDistanceToHoodAngle.put(13.133, 45.0);
    FerryDistanceToHoodAngle.put(Double.MAX_VALUE, 45.0);
  }
}
