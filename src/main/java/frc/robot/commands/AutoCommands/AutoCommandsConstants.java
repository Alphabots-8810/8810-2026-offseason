package frc.robot.commands.AutoCommands;

public final class AutoCommandsConstants {
  private AutoCommandsConstants() {}

  // Safety cap on the pre-path inward intake zero: if the current spike is never seen, give up
  // and run the auto without the intake rather than stalling on the first step.
  public static final double INWARD_ZERO_TIMEOUT_SEC = 1.0;

  // How long each timed Shooting burst runs in the auto sequence.
  public static final double SHOOTING_DURATION_SEC = 3.0;

  // Commented out 2026-07-11: these Choreo trajectories were removed from deploy/choreo when the
  // Swap-trajectory BlockAutoBuilder scheme replaced the hand-built three-piece auto (commits
  // b9b9530 and 69b2537). Restore together with leftThreePieceAuto() in AutoCommands if the
  // .traj files are recreated.
  //
  // // Choreo trajectory names (loaded via PathPlannerPath.fromChoreoTrajectory).
  // public static final String LEFT_PATH_1 = "Left1_path1";
  // public static final String LEFT_PATH_2 = "Left1_path2";
  // public static final String LEFT_PATH_3 = "Left1_path3";
}
