package frc.robot.commands.ManualCommand;

import frc.robot.util.LoggedTunableNumber;

public final class ManualConstants {
  public static final LoggedTunableNumber ShooterRotpsTunable =
      new LoggedTunableNumber("Manual/ShooterRotps", 46);
  public static final LoggedTunableNumber HoodDegTunable =
      new LoggedTunableNumber("Manual/HoodDegree", 0);
  public static final LoggedTunableNumber IndexerRotpsTunable =
      new LoggedTunableNumber("Manual/IndexerRotps", 40);
  public static final LoggedTunableNumber IntakeRollerRotpsTunable =
      new LoggedTunableNumber("Manual/IntakeRollerRotps", 40);
  public static final LoggedTunableNumber FeederRotpsTunable =
      new LoggedTunableNumber("Manual/FeederRotps", 40);
  /*
   * 2.118 0 47.5
   * 2.54  5  49.08
   * 3.05  12 51.08
   * 3.53 16 53.26
   * 4.15 20.5 56.
   * 4.55 23 58
   * 5.06 26 60
   */
  public static final double SHOOTER_VELOCITY_TOLERANCE_ROTPS = 1.0;
  public static final double HOOD_ANGLE_TOLERANCE_ROT = 0.1;

  private ManualConstants() {}
}
