package frc.robot.commands.IntakeDeployZeroCommand;

public final class IntakeDeployZeroCommandConstants {
  public static final double OUTWARD_ZEROING_VOLTAGE = 3.0;
  public static final double INWARD_ZEROING_VOLTAGE = -3.0;
  public static final double CURRENT_THRESHOLD = 40.0;
  public static final double OUTWARD_DEBOUNCE_TIME_SECONDS = 0.3;
  // Inward zeroing runs at auto start with the intake already seated on the stowed hard stop, so
  // the spike is immediate and a shorter debounce keeps the pre-path delay small.
  public static final double INWARD_DEBOUNCE_TIME_SECONDS = 0.15;
  public static final double INWARD_ZERO_POSITION_CM = 0.0;

  private IntakeDeployZeroCommandConstants() {}
}
