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
   * 2.54  5
   */
  public static final double SHOOTER_VELOCITY_TOLERANCE_ROTPS = 1.0;
  public static final double HOOD_ANGLE_TOLERANCE_ROT = 0.1;

  private ManualConstants() {}
}
