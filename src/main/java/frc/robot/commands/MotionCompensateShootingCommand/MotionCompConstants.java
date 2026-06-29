package frc.robot.commands.MotionCompensateShootingCommand;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.util.LoggedTunableNumber;

public final class MotionCompConstants {
  public static final LoggedTunableNumber MaxShootWhileMovingSpeedMpsTunable =
      new LoggedTunableNumber("Shooting/MotionComp/MaxShootWhileMovingSpeedMps", 1.5);

  public static final LoggedTunableNumber CompensationScaleTunable =
      new LoggedTunableNumber("Shooting/MotionComp/CompensationScale", 1.0);

  public static final double LAUNCH_DELAY_SEC = 0.0;

  public static final double YAW_UNSTABLE_THRESHOLD_RADPS = 0.75;

  public static final double STATIC_THRESHOLD_MPS = 0.09;

  public static final InterpolatingDoubleTreeMap distanceToFlyTime =
      new InterpolatingDoubleTreeMap();

  static {
    distanceToFlyTime.put(1.967387, 1.046);
    distanceToFlyTime.put(2.207408, 1.080);
    distanceToFlyTime.put(2.570000, 1.100);
    distanceToFlyTime.put(2.663911, 1.107);
    distanceToFlyTime.put(2.750000, 1.200);
    distanceToFlyTime.put(2.806310, 1.233);
    distanceToFlyTime.put(3.000000, 1.237);
    distanceToFlyTime.put(3.569663, 1.267);
    // 以下两点尚未确认，暂时注释；确认后取消注释并补充更多远距数据
    // distanceToFlyTime.put(4.180000, 1.300);
    // distanceToFlyTime.put(4.500000, 1.466);
  }

  public static double getFlightTimeSec(double distanceMeters) {
    return distanceToFlyTime.get(distanceMeters);
  }

  private MotionCompConstants() {}
}
