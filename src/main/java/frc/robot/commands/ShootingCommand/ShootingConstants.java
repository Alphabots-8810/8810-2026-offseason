package frc.robot.commands.ShootingCommand;

import frc.robot.util.LoggedTunableNumber;

public final class ShootingConstants {
  public static final LoggedTunableNumber IndexerRotpsTunable =
      new LoggedTunableNumber("Manual/IndexerRotps", 40);
  public static final LoggedTunableNumber IntakeRollerRotpsTunable =
      new LoggedTunableNumber("Manual/IntakeRollerRotps", 40);
  public static final LoggedTunableNumber FeederRotpsTunable =
      new LoggedTunableNumber("Manual/FeederRotps", 40);
}
