package frc.robot.commands.AutoTrenchCommand;

public final class AutoTrenchCommandConstants {
  public static final double FIELD_MID_Y_METERS = 4.0345;
  public static final double DOWN_TRENCH_Y_METERS = 0.66599;
  public static final double UP_TRENCH_Y_METERS = 7.40334;

  public static final double BLUE_TRENCH_X_METERS = 4.0;
  public static final double RED_TRENCH_X_METERS = 12.5;

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
