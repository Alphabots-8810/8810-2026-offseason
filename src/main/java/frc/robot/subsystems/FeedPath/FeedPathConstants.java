package frc.robot.subsystems.FeedPath;

public final class FeedPathConstants {

  public static final String CAN_BUS = "rio";

  public static final int HOPPER_CAN_ID = 0;
  public static final int BALL_TUNNEL_CAN_ID = 1;

  public static final double MIN_SIGNAL_STRENGTH = 2500;
  public static final double PROXIMITY_THRESHOLD_M = 0.4;
  public static final double PROXIMITY_HYSTERESIS_M = 0.01;

  public static final double HOPPER_MAX_DISTANCE_M = 0.3;
  public static final double HOPPER_DEBOUNCE_SECS = 0.75;
  public static final double HOPPER_FOV_RANGE_X_DEG = 27.0;
  public static final double HOPPER_FOV_RANGE_Y_DEG = 6.75;

  public static final double BALL_TUNNEL_MAX_DISTANCE_M = 0.6;
  public static final double BALL_TUNNEL_DEBOUNCE_SECS = 0.06;
  public static final double BALL_TUNNEL_FOV_RANGE_X_DEG = 10.0;
  public static final double BALL_TUNNEL_FOV_RANGE_Y_DEG = 6.75;

  private FeedPathConstants() {}
}
