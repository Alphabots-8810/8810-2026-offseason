package frc.robot.subsystems.FeedPath;

public final class FeedPathConstants {

  // TODO: 改成你们实际的 CAN 总线名（RoboRIO 总线用 "rio"，CANivore 用总线名称）
  public static final String CAN_BUS = "rio";

  // TODO: 改成实际 CAN ID
  public static final int HOPPER_CAN_ID = 0;
  public static final int INDEXER_CAN_ID = 1;

  // 共用的近距离检测参数
  public static final double MIN_SIGNAL_STRENGTH = 2500;
  public static final double HOPPER_PROXIMITY_THRESHOLD_M = 0.55;
  public static final double HOPPER_PROXIMITY_HYSTERESIS_M = 0.01;
  public static final double INDEXER_PROXIMITY_THRESHOLD_M = 0.55;
  public static final double INDEXER_PROXIMITY_HYSTERESIS_M = 0.01;

  // Hopper（入口）：宽视角，慢防抖，避免误触发
  public static final double HOPPER_MAX_DISTANCE_M = 0.3;
  public static final double HOPPER_DEBOUNCE_SECS = 0.75;
  public static final double HOPPER_FOV_RANGE_X_DEG = 6.75;
  public static final double HOPPER_FOV_RANGE_Y_DEG = 6.75;

  // BallTunnel（通道）：窄视角，快防抖，实时判断球是否到位
  public static final double INDEXER_MAX_DISTANCE_M = 0.6;
  public static final double INDEXER_DEBOUNCE_SECS = 0.01;
  public static final double INDEXER_FOV_RANGE_X_DEG = 10.0;
  public static final double INDEXER_FOV_RANGE_Y_DEG = 20;

  private FeedPathConstants() {}
}
