package frc.robot.commands.ManualCommand;

import frc.robot.util.LoggedTunableNumber;

public final class ManualConstants {
    public static final LoggedTunableNumber ShooterRadpsTunable = new LoggedTunableNumber("Manual/ShooterRadps", 60);
    public static final LoggedTunableNumber HoodAngleTunable = new LoggedTunableNumber("Manual/Angle", 10);
    public static final LoggedTunableNumber IndexerRadpsTunable = new LoggedTunableNumber("Manual/IndexerRadps", 40);
    public static final LoggedTunableNumber RollerRadpsTunable = new LoggedTunableNumber("Manual/Angle", 40);
    public static final LoggedTunableNumber FeederRadpsTunable = new LoggedTunableNumber("Manual/Angle", 40);
}
