package frc.robot.commands.AutopilotTrenchCommand;

public final class AutopilotTrenchCommandConstants {
  /** Cruise speed through the trench. */
  public static final double PASS_SPEED_METERS_PER_SEC = 5;

  /**
   * Floor for the APPROACH arc-speed cap (the cap itself is computed each cycle from the remaining
   * arc's turn radius — see {@code maxTrackableArcSpeed}). Keeps the robot from crawling when the
   * geometry is momentarily degenerate, e.g. the target sits nearly behind the robot mid-wrap.
   */
  public static final double APPROACH_MIN_SPEED_METERS_PER_SEC = 1.5;

  public static final double MAX_ACCELERATION_METERS_PER_SEC_SQ = 9.0;

  /**
   * Lateral deceleration assumed by the APPROACH centerline brake clamp. Deliberately far below the
   * real acceleration limit: the drivetrain follows the commanded velocity with ~0.1-0.2 s of lag,
   * so the command must start braking early for the actual robot to merge onto the centerline
   * without swinging past it. Sim sweep 2026-07-12: no clamp = handoff velocity 22 deg off-axis and
   * 10 cm off-center at the trench face; 4.0 = 7 deg / 4 cm; 2.5 = 0.6 deg / 4 mm (+0.1 s total).
   */
  public static final double CENTERLINE_BRAKE_ACCEL_METERS_PER_SEC_SQ = 2.5;

  public static final double MAX_JERK_METERS_PER_SEC_CUBED = 16.0;

  /**
   * Robot-center margin before the trench's near entry face for the entrance target.
   *
   * <p>The simulated bumper is 0.85 m long, so its leading edge is 0.425 m ahead of the pose.
   * Keeping the pose 0.60 m outside the face leaves 0.175 m for control/odometry error and one
   * 20-ms cycle of travel at high speed.
   */
  public static final double ENTRANCE_MARGIN_METERS = 0.60;

  /** Margin past the trench's far exit face for the no-next-path goal. */
  public static final double EXIT_MARGIN_METERS = 1.2;

  /**
   * Distance before the entrance target at which control may switch to the goal target. This keeps
   * Autopilot from decelerating onto (or producing zero output exactly at) the intermediate
   * entrance pose. The robot must already be centerline- and heading-aligned at the switch, so
   * looking through the entrance is both smooth and collision-safe.
   */
  public static final double PASS_TARGET_LOOKAHEAD_METERS = 0.35;

  /** Lateral and heading tolerance required before PASS may begin. */
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
