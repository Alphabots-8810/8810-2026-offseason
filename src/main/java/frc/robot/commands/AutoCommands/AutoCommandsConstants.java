package frc.robot.commands.AutoCommands;

public final class AutoCommandsConstants {
  private AutoCommandsConstants() {}

  // How long after Left1_path1 starts before the intake deploys and runs.
  public static final double INTAKE_START_DELAY_SEC = 1.0;

  // How long each timed Shooting burst runs in the auto sequence.
  public static final double SHOOTING_DURATION_SEC = 2.0;

  // Choreo trajectory names (loaded via PathPlannerPath.fromChoreoTrajectory).
  public static final String LEFT_PATH_1 = "Left1_path1";
  public static final String LEFT_PATH_2 = "Left1_path2";
  public static final String LEFT_PATH_3 = "Left1_path3";
}
