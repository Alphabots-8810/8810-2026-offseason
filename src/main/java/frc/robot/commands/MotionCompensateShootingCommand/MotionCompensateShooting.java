package frc.robot.commands.MotionCompensateShootingCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.ShootingCommand.ShootingConstants;
import frc.robot.subsystems.Drum.Drum;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.IntakeDeploy.IntakeDeploy;
import frc.robot.subsystems.IntakeRoller.IntakeRoller;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * 运动补偿射击指令（Look-Ahead 预判法）。
 *
 * <h3>核心原理</h3>
 *
 * <p>机器人在移动中射击时，球离开炮口到命中目标之间有一段"飞行时间"，
 * 这段时间里机器人位置已经发生了变化。
 * 若仍以"当前位置→目标"的角度瞄准，球落点会产生系统性偏差。
 *
 * <p><b>Look-Ahead 解决方案：</b>不从"现在的我"瞄准目标，
 * 而是从"球出口时的我"瞄准目标。具体分两段积分：
 *
 * <pre>
 *  当前姿态 ──(发射延迟积分)──▶ 出口姿态 ──(飞行时间积分)──▶ Look-Ahead 位置
 *                                                                      │
 *                                                              从这里向目标做角度计算
 * </pre>
 *
 * <h3>拒绝预判（Reject Look-Ahead）</h3>
 *
 * <p>以下情况放弃 look-ahead，退化为直接用当前位置瞄准：
 * <ul>
 *   <li>偏航角速度过大 <b>且</b> 瞄准误差超限：旋转剧烈时 look-ahead 会放大误差；
 *   <li>线速度极低（近似静止）：补偿量趋近 0，直接瞄准反而更稳定。
 * </ul>
 *
 * <h3>状态机</h3>
 *
 * <pre>
 *  AIM  ──(三项就绪)──▶  SHOOT
 *  (预热飞轮+炮管,       (维持飞轮+炮管,
 *   调整车头方向,         继续跟踪目标,
 *   供弹停止)             驱动全段供弹链条)
 * </pre>
 */
public class MotionCompensateShooting extends Command {

  // =====================================================================
  //  状态机枚举
  // =====================================================================

  private enum States {
    AIM,
    SHOOT
  }

  private States state;

  // =====================================================================
  //  常量
  // =====================================================================

  /** 手柄输入死区。幅值小于此值的输入视为零，防止摇杆漂移。 */
  private static final double DEADBAND = 0.1;

  // =====================================================================
  //  输入
  // =====================================================================

  /** 手柄 X 轴（场地前后方向）输入，范围 [-1, 1]。 */
  private final DoubleSupplier xSupplier;

  /** 手柄 Y 轴（场地左右方向）输入，范围 [-1, 1]。 */
  private final DoubleSupplier ySupplier;

  // =====================================================================
  //  旋转 PID 控制器
  // =====================================================================

  /**
   * 带梯形速度曲线（Trapezoidal Profile）的旋转 PID 控制器。
   *
   * <p>约束：最大角速度 8 rad/s，最大角加速度 30 rad/s²。
   * 增益 P=5, D=0.4（与 DriveCommands.joystickDriveAtAngle 保持一致）。
   * 启用连续输入（-π ~ π），防止穿越 ±180° 时出现反向转动。
   */
  private final ProfiledPIDController angleController =
      new ProfiledPIDController(5.0, 0.0, 0.4, new TrapezoidProfile.Constraints(8.0, 30.0));

  // =====================================================================
  //  每帧缓存（在 execute() 开头统一刷新，同帧内复用）
  // =====================================================================

  /**
   * 本帧机器人到 Hub 的距离（米）。
   * 用于查询飞行时间插值表、飞轮转速表和炮管角度表。
   */
  private double cachedDistanceM;

  /**
   * 本帧 Look-Ahead 预判姿态。
   *
   * <p>含义：若以当前速度匀速运动，经过「发射延迟 + 飞行时间」后机器人将在的位置。
   * 从此位置向 Hub 方向计算瞄准角，即为当前帧的目标方向。
   */
  private Pose2d cachedLookAheadPose;

  /**
   * 本帧是否拒绝 look-ahead 补偿。
   * {@code true} = 退化为直接用当前位置瞄准。
   */
  private boolean cachedRejectLookAhead;

  // =====================================================================
  //  构造函数
  // =====================================================================

  /** 无手柄输入版本：机器人原地旋转对准目标后射击。 */
  public MotionCompensateShooting() {
    this(() -> 0.0, () -> 0.0);
  }

  /**
   * 完整版：支持边移动边射击。
   *
   * @param xSupplier 手柄 X 轴（场地前后）输入函数
   * @param ySupplier 手柄 Y 轴（场地左右）输入函数
   */
  public MotionCompensateShooting(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    addRequirements(
        Drive.mInstance,
        Drum.mInstance,
        Feeder.mInstance,
        Hood.mInstance,
        Indexer.mInstance,
        IntakeDeploy.mInstance,
        IntakeRoller.mInstance);
    angleController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // =====================================================================
  //  目标位置
  // =====================================================================

  /**
   * 返回本联盟 Hub 的场地坐标（Translation2d）。
   * 红队 / 蓝队使用不同坐标，每帧从 DriverStation 读取联盟颜色。
   */
  private Translation2d hubLocation() {
    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    return isRed
        ? Constants.FieldConstants.RED_HUB_LOCATION
        : Constants.FieldConstants.BLUE_HUB_LOCATION;
  }

  // =====================================================================
  //  Look-Ahead 预判位置计算
  // =====================================================================

  /**
   * 计算 Look-Ahead 预判姿态：球命中目标时刻，机器人将在的场地位置。
   *
   * <h4>两段积分</h4>
   *
   * <p><b>第一段（发射延迟）</b>：从当前姿态用机器人相对速度积分 {@code LAUNCH_DELAY_SEC}。
   * 此阶段包含旋转分量（omegaRadiansPerSecond），因为在延迟期间机器人真实在旋转。
   * 使用 {@link Pose2d#exp(Twist2d)} 而非线性叠加，可正确处理弧线运动。
   *
   * <p><b>第二段（飞行时间）</b>：从出口姿态继续积分 {@code flightTimeSec × scale}。
   * 旋转分量置 0（球在空中，机器人旋转不影响球的飞行轨迹）。
   * 乘以 {@code CompensationScaleTunable} 可在现场微调补偿强度。
   *
   * <h4>等价关系</h4>
   *
   * <p>对纯平移运动，此方法与原"虚拟目标偏移法"数学等价：
   * <pre>
   *   从 (currentPos + v·t) 瞄准 hub
   *   ≡ 从 currentPos 瞄准 (hub - v·t)
   * </pre>
   * 但 Look-Ahead 能正确处理旋转导致的曲线运动，且具备拒绝逻辑。
   *
   * @param flightTimeSec 球在空中的飞行时间（秒），由距离插值表查询得到
   * @return 预判后的机器人场地姿态
   */
  private Pose2d computeLookAheadPose(double flightTimeSec) {
    Pose2d currentPose = Drive.mInstance.getPose();
    // getChassisSpeeds() 返回机器人相对速度（robot-relative），
    // 与 Pose2d.exp() 接受的 Twist2d 坐标系一致，无需额外旋转变换。
    ChassisSpeeds speeds = Drive.mInstance.getChassisSpeeds();

    // ── 第一段：发射延迟 ──────────────────────────────────────────────────
    // 从按下扳机到球离开炮口，机器人已经移动并旋转了 LAUNCH_DELAY_SEC。
    // 如果 LAUNCH_DELAY_SEC = 0（当前值），此步骤不改变姿态。
    Pose2d poseAtRelease = currentPose.exp(new Twist2d(
        speeds.vxMetersPerSecond * MotionCompConstants.LAUNCH_DELAY_SEC,
        speeds.vyMetersPerSecond * MotionCompConstants.LAUNCH_DELAY_SEC,
        speeds.omegaRadiansPerSecond * MotionCompConstants.LAUNCH_DELAY_SEC));

    // ── 第二段：飞行时间 ──────────────────────────────────────────────────
    // 球在空中期间，机器人继续平移（旋转分量对瞄准角影响极小，置 0）。
    // 乘以 CompensationScaleTunable 便于现场微调：
    //   > 1.0 → 补偿过冲时减小（调低比例）
    //   < 1.0 → 补偿不足时增大（调高比例）
    double scaledFlightTime =
        flightTimeSec * MotionCompConstants.CompensationScaleTunable.getAsDouble();
    return poseAtRelease.exp(new Twist2d(
        speeds.vxMetersPerSecond * scaledFlightTime,
        speeds.vyMetersPerSecond * scaledFlightTime,
        0.0));
  }

  // =====================================================================
  //  拒绝 Look-Ahead 判断
  // =====================================================================

  /**
   * 判断偏航角速度是否不稳定（旋转过快）。
   *
   * <p>角速度超过 {@code YAW_UNSTABLE_THRESHOLD_RADPS} 时，
   * look-ahead 位置误差会被旋转运动放大，瞄准质量下降。
   */
  private boolean isYawUnstable() {
    return Math.abs(Drive.mInstance.getChassisSpeeds().omegaRadiansPerSecond)
        >= MotionCompConstants.YAW_UNSTABLE_THRESHOLD_RADPS;
  }

  /**
   * 判断当前 PID 瞄准误差是否在可接受范围内。
   *
   * <p>误差 = 当前车头角度与目标方向之差（由 PID 控制器持续跟踪）。
   * 误差过大说明车头还未转到位，此时 look-ahead 的预判意义有限。
   */
  private boolean isAimErrorTolerable() {
    return Math.abs(angleController.getPositionError()) < ShootingConstants.AIM_ANGLE_TOLERANCE_RAD;
  }

  /**
   * 计算本帧是否应拒绝 look-ahead 补偿。
   *
   * <p>拒绝条件（满足其一即拒绝）：
   * <ol>
   *   <li><b>旋转不稳定 {@code &&} 误差超限</b>：两者同时满足说明旋转剧烈且还没对准，
   *       look-ahead 会把不稳定的旋转误差进一步放大，不如直接瞄准当前位置；
   *   <li><b>线速度 < STATIC_THRESHOLD_MPS</b>：机器人几乎静止，
   *       look-ahead 偏移量接近 0，跳过补偿可避免速度估计噪声的干扰。
   * </ol>
   *
   * @return {@code true} 表示本帧放弃 look-ahead，使用当前位置瞄准
   */
  private boolean computeRejectLookAhead() {
    ChassisSpeeds speeds = Drive.mInstance.getChassisSpeeds();
    double linearSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    boolean isStatic = linearSpeed < MotionCompConstants.STATIC_THRESHOLD_MPS;
    return (isYawUnstable() && !isAimErrorTolerable()) || isStatic;
  }

  // =====================================================================
  //  瞄准角度计算
  // =====================================================================

  /**
   * 选择本帧用于计算瞄准角的"基准平移量"。
   *
   * <ul>
   *   <li>接受 look-ahead → 使用预判位置（{@code cachedLookAheadPose} 的平移分量）
   *   <li>拒绝 look-ahead → 使用机器人当前位置（回退到非补偿模式）
   * </ul>
   */
  private Translation2d getBaseTranslation() {
    return cachedRejectLookAhead
        ? Drive.mInstance.getPose().getTranslation()
        : cachedLookAheadPose.getTranslation();
  }

  /**
   * 计算从基准位置到 Hub 的目标方向角（场地坐标系）。
   *
   * <p>这是运动补偿的核心输出：
   * 若 look-ahead 被接受，此角度已经考虑了机器人在球飞行期间的位移，
   * 使飞出的球能精确命中目标而非飞过目标后方。
   */
  private Rotation2d targetAngleToHub() {
    return hubLocation().minus(getBaseTranslation()).getAngle();
  }

  // =====================================================================
  //  底盘控制
  // =====================================================================

  /**
   * 将手柄原始输入转换为归一化线速度向量（方向 + 幅值，幅值 ∈ [0, 1]）。
   *
   * <p>处理步骤：
   * <ol>
   *   <li>对幅值施加死区（{@code DEADBAND}），消除摇杆漂移；
   *   <li>对幅值做平方映射（small → 更精细控制，large → 更快响应）；
   *   <li>用 {@link Pose2d#transformBy} 将幅值与方向重新合并为 Translation2d。
   * </ol>
   */
  private Translation2d getLinearVelocityFromJoysticks() {
    double x = xSupplier.getAsDouble();
    double y = ySupplier.getAsDouble();
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // 平方映射：低速更精细，高速响应更快
    linearMagnitude = linearMagnitude * linearMagnitude;

    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  /**
   * 将手柄线速度限制在射击移动最大速度以内。
   *
   * <p>高速移动时运动补偿精度下降，限速可降低射击时的系统误差。
   * 方向保持不变，仅在超限时对幅值做等比缩放。
   *
   * @param linearVelocityMetersPerSec 原始线速度向量（单位 m/s）
   * @return 限速后的线速度向量，方向不变，幅值 ≤ MaxShootWhileMovingSpeedMpsTunable
   */
  private Translation2d limitLinearVelocity(Translation2d linearVelocityMetersPerSec) {
    double speed = linearVelocityMetersPerSec.getNorm();
    double maxSpeed =
        Math.max(0.0, MotionCompConstants.MaxShootWhileMovingSpeedMpsTunable.getAsDouble());
    if (speed <= maxSpeed || speed < 1e-6) {
      return linearVelocityMetersPerSec;
    }
    return linearVelocityMetersPerSec.times(maxSpeed / speed);
  }

  private void aimDrive() {
    double omega =
        angleController.calculate(
            Drive.mInstance.getRotation().getRadians(), targetAngleToHub().getRadians());

    // 手柄线速度：归一化 → 限速 → 换算为场地 m/s
    Translation2d linearVelocity = getLinearVelocityFromJoysticks();
    Translation2d limitedLinearVelocity =
        limitLinearVelocity(linearVelocity.times(Drive.mInstance.getMaxLinearSpeedMetersPerSec()));

    ChassisSpeeds fieldRelativeSpeeds =
        new ChassisSpeeds(limitedLinearVelocity.getX(), limitedLinearVelocity.getY(), omega);

    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    Drive.mInstance.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldRelativeSpeeds,
            isFlipped
                ? Drive.mInstance.getRotation().plus(Rotation2d.kPi)
                : Drive.mInstance.getRotation()));
  }


  private void prepareShooter() {
    Drum.mInstance.setVelocityRotPerSec(targetShooterRotps());
    Hood.mInstance.setPositionRot(targetHoodRot());
  }

  private double targetShooterRotps() {
    return ShootingConstants.distanceToShooterRotps.get(cachedDistanceM);
  }

  private double targetHoodRot() {
    return ShootingConstants.distanceToHoodDeg.get(cachedDistanceM) / 360.0;
  }

  private boolean isReadyToShoot() {
    boolean drumReady =
        Math.abs(Drum.mInstance.getVelocityRotPerSec() - targetShooterRotps())
            < ShootingConstants.SHOOTER_VELOCITY_TOLERANCE_ROTPS;
    boolean hoodReady =
        Math.abs(Hood.mInstance.getPositionRot() - targetHoodRot())
            < ShootingConstants.HOOD_ANGLE_TOLERANCE_ROT;
    boolean aimReady =
        Math.abs(angleController.getPositionError()) < ShootingConstants.AIM_ANGLE_TOLERANCE_RAD;
    return drumReady && hoodReady && aimReady;
  }

  private void aim() {
    prepareShooter();
    aimDrive();
    // 供弹链条停止，防止在未就绪时提前送球导致射击失败
    IntakeRoller.mInstance.setV(0);
    Indexer.mInstance.setV(0);
    Feeder.mInstance.setV(0);

    if (isReadyToShoot()) {
      state = States.SHOOT;
    }
  }
  private void shoot() {
    prepareShooter();
    aimDrive();
    IntakeRoller.mInstance.setVelocityRotPerSec(
        ShootingConstants.IntakeRollerRotpsTunable.getAsDouble());
    Indexer.mInstance.setVelocityRotPerSec(ShootingConstants.IndexerRotpsTunable.getAsDouble());
    Feeder.mInstance.setVelocityRotPerSec(ShootingConstants.FeederRotpsTunable.getAsDouble());
  }

  private void log() {
    double flightTime = MotionCompConstants.getFlightTimeSec(cachedDistanceM);
    Logger.recordOutput("Shooting/MotionComp/DistanceM", cachedDistanceM);
    Logger.recordOutput("Shooting/MotionComp/FlightTimeSec", flightTime);
    Logger.recordOutput("Shooting/MotionComp/LookAheadPose", cachedLookAheadPose);
    Logger.recordOutput("Shooting/MotionComp/RejectLookAhead", cachedRejectLookAhead);
    Logger.recordOutput("Shooting/MotionComp/BaseTranslation", getBaseTranslation());
    Logger.recordOutput("Shooting/MotionComp/TargetAngle", targetAngleToHub());
    Logger.recordOutput(
        "Shooting/MotionComp/MaxShootWhileMovingSpeedMps",
        MotionCompConstants.MaxShootWhileMovingSpeedMpsTunable.getAsDouble());

    SmartDashboard.putBoolean("MotionCompAligning", state == States.AIM);
    SmartDashboard.putBoolean("MotionCompShooting", state == States.SHOOT);
    SmartDashboard.putBoolean("MotionCompRejectLookAhead", cachedRejectLookAhead);
    SmartDashboard.putNumber("MotionCompFlightTimeSec", flightTime);
    SmartDashboard.putNumber("MotionCompDistanceM", cachedDistanceM);
  }


  @Override
  public void initialize() {
    state = States.AIM;
    angleController.reset(Drive.mInstance.getRotation().getRadians());
  }

  @Override
  public void execute() {
    cachedDistanceM =
        Drive.mInstance.getPose().getTranslation().getDistance(hubLocation());
    double flightTimeSec = MotionCompConstants.getFlightTimeSec(cachedDistanceM);
    cachedLookAheadPose = computeLookAheadPose(flightTimeSec);
    cachedRejectLookAhead = computeRejectLookAhead();

    switch (state) {
      case AIM -> aim();
      case SHOOT -> shoot();
    }

    log();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Drum.mInstance.stop();
    Hood.mInstance.stop();
    IntakeRoller.mInstance.stop();
    Indexer.mInstance.stop();
    Feeder.mInstance.stop();
    Drive.mInstance.stop();
  }
}
