package frc.robot.commands.MotionCompensateShootingCommand;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.util.LoggedTunableNumber;

public final class MotionCompConstants {

  // ── 可调参数（可通过 SmartDashboard / Tunable 实时修改） ─────────────────

  /** 射击移动时允许的最大线速度（m/s）。超出则限幅，防止高速移动造成补偿误差。 */
  public static final LoggedTunableNumber MaxShootWhileMovingSpeedMpsTunable =
      new LoggedTunableNumber("Shooting/MotionComp/MaxShootWhileMovingSpeedMps", 1.5);

  /**
   * 飞行时间补偿缩放比例（无量纲）。
   * 1.0 = 按实测飞行时间完整补偿；调低可在补偿过冲时减弱效果。
   */
  public static final LoggedTunableNumber CompensationScaleTunable =
      new LoggedTunableNumber("Shooting/MotionComp/CompensationScale", 1.0);

  // ── 固定常量 ─────────────────────────────────────────────────────────────

  /**
   * 扳机按下到球实际离开炮口的延迟时间（秒）。
   *
   * <p>这段时间里机器人仍在移动，需要单独积分（含旋转分量）。
   * 目前置 0.0：若实测延迟显著可填入实测值。
   */
  public static final double LAUNCH_DELAY_SEC = 0.0;

  /**
   * 偏航角速度不稳定阈值（rad/s）。
   *
   * <p>角速度超过此值时认为车头旋转过猛，look-ahead 补偿可能引入额外误差。
   * 结合瞄准误差超限时，放弃 look-ahead，退化为直接用当前位置瞄准。
   * 参考范围：0.5 ~ 1.0 rad/s；可按实际调整。
   */
  public static final double YAW_UNSTABLE_THRESHOLD_RADPS = 0.75;

  /**
   * 线速度静止判定阈值（m/s）。
   *
   * <p>线速度低于此值时视为静止射击，运动补偿量几乎为 0，
   * 跳过 look-ahead 直接瞄准，避免噪声干扰。
   */
  public static final double STATIC_THRESHOLD_MPS = 0.09;

  // ── 距离 → 飞行时间插值表 ───────────────────────────────────────────────

  /**
   * 距离（米）→ 球飞行时间（秒）插值表。
   *
   * <p>数据来源：慢动作视频逐帧测量。表格只覆盖约 2 ~ 3.6 m；
   * 超出范围时 {@link InterpolatingDoubleTreeMap} 会外推端点，精度下降，请注意。
   */
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

  /** 根据当前距离（米）查询球飞行时间（秒）。 */
  public static double getFlightTimeSec(double distanceMeters) {
    return distanceToFlyTime.get(distanceMeters);
  }

  private MotionCompConstants() {}
}
