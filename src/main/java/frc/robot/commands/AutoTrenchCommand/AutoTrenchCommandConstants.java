package frc.robot.commands.AutoTrenchCommand;

public final class AutoTrenchCommandConstants {
  public static final double FIELD_MID_Y_METERS = 4.0345;
  public static final double DOWN_TRENCH_Y_METERS = 0.66599;
  public static final double UP_TRENCH_Y_METERS = 7.40334;

  // Trench structure CENTER along X, from maple-sim Arena2026Rebuilt collision geometry:
  // field mid 8.27 -/+ (120in + 47in/2). Entry faces sit at center -/+ half length.
  public static final double BLUE_TRENCH_X_METERS = 4.625;
  public static final double RED_TRENCH_X_METERS = 11.915;

  /**
   * Trench structure length along X. Matches the maple-sim collision box (53 in = 1.346 m); the
   * official structure depth is 47 in (1.194 m), so this carries ~7.6 cm extra margin per face on
   * the real field.
   */
  public static final double TRENCH_LENGTH_METERS = 1.346;

  public static final double TRENCH_HALF_LENGTH_METERS = TRENCH_LENGTH_METERS / 2.0;

  public static final double Y_KP = 2.2;
  public static final double Y_KI = 0.0;
  public static final double Y_KD = 0.08;
  public static final double MAX_Y_SPEED_METERS_PER_SEC = 2.0;
  public static final double Y_TOLERANCE_METERS = 0.06;

  public static final double ANGLE_KP = 5.0;
  public static final double ANGLE_KD = 0.4;
  public static final double ANGLE_MAX_VELOCITY_RAD_PER_SEC = 8.0;
  public static final double ANGLE_MAX_ACCELERATION_RAD_PER_SEC_SQ = 20.0;
  public static final double MAX_ANGULAR_SPEED_RAD_PER_SEC = 6.0;

  public static final double JOYSTICK_DEADBAND = 0.03;
  public static final double X_STOP_DISTANCE_METERS = 0.5;
  public static final double X_SLOWDOWN_RANGE_METERS = 2.0;

  private AutoTrenchCommandConstants() {}
}
