package frc.robot.commands.AutopilotTrenchCommand;

public final class AutopilotTrenchCommandConstants {
  /** Cruise speed through the trench. */
  public static final double PASS_SPEED_METERS_PER_SEC = 4;

  public static final double MAX_ACCELERATION_METERS_PER_SEC_SQ = 9.0;
  public static final double MAX_JERK_METERS_PER_SEC_CUBED = 16.0;

  /**
   * Margin before the trench's near entry face (along X) for the stage-1 entrance target. The
   * trench is a 1.2 m long structure, so the face sits at center +/- half length. Autopilot's
   * entry-angle spiral only finishes converging onto the centerline at its target, so the target
   * must sit before the entry face: alignment completes before the robot is inside the structure.
   */
  public static final double ENTRANCE_MARGIN_METERS = 0.4;

  /** Margin past the trench's far exit face (along X) for the stage-2 exit target. */
  public static final double EXIT_MARGIN_METERS = 1.2;

  /**
   * Minimum runway before the entrance plane for the flow-through approach (entry-angle spiral at
   * pass speed). With less room than this, the spiral can't converge in time, so the approach
   * instead beelines to the entrance point and stops there before passing.
   */
  public static final double MIN_RUNWAY_METERS = 0.5;

  /** Lateral and heading tolerance required at the entrance plane before PASS may begin. */
  public static final double ALIGN_Y_TOLERANCE_METERS = 0.08;

  public static final double ALIGN_HEADING_TOLERANCE_DEGREES = 5.0;

  public static final double ERROR_XY_METERS = 0.05;
  public static final double ERROR_THETA_DEGREES = 3.0;
  public static final double BEELINE_RADIUS_METERS = 0.10;

  public static final double ANGLE_KP = 5.0;
  public static final double ANGLE_KD = 0.4;
  public static final double ANGLE_MAX_VELOCITY_RAD_PER_SEC = 10.0;
  public static final double ANGLE_MAX_ACCELERATION_RAD_PER_SEC_SQ = 30.0;
  public static final double MAX_ANGULAR_SPEED_RAD_PER_SEC = 8.0;

  private AutopilotTrenchCommandConstants() {}
}
