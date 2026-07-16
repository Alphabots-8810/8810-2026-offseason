package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import org.littletonrobotics.junction.Logger;

/**
 * Vision localization from a single Limelight running MegaTag2 AprilTag pose estimation. Publishes
 * the robot orientation each loop (required by MegaTag2) and feeds accepted pose estimates into the
 * drive odometry. Heading stays locked to the gyro, so rotation is given an enormous std dev.
 */
public class Vision extends SubsystemBase {

  public static final Vision mInstance = new Vision("limelight");
  public static final Vision mInstance2 = new Vision("limelight-l");

  // Reject vision while moving faster than these thresholds; MegaTag2 degrades under speed.
  private static final double MAX_LINEAR_SPEED_MPS = 2.0;
  private static final double MAX_ANGULAR_SPEED_RAD_PER_SEC = 3.0;

  // Heading is always locked to the gyro, so give rotation a huge std dev.
  private static final double THETA_STD_DEV_RAD = 99999999.0;

  // TA (target area, 0-100) maps to XY std dev in meters. Larger TA = closer tag = more trust.
  private static final InterpolatingDoubleTreeMap TA_TO_XY_STD_DEV_METERS = taToXYStdDevMeters();

  private final String name;
  private final Alert disconnectedAlert;

  private boolean enabled = true;

  public Vision(String name) {
    super(name);
    this.name = name;
    this.disconnectedAlert =
        new Alert("Disconnected vision camera \"" + name + "\".", AlertType.kWarning);

    // No camera hardware or vision simulation outside the real robot.
    if (Constants.currentMode != Constants.Mode.REAL) {
      enabled = false;
    }
  }

  @Override
  public void periodic() {
    if (!enabled) return;

    boolean connected = isConnected();
    disconnectedAlert.set(!connected);
    Logger.recordOutput(name + "/Connected", connected);

    updateGyro();
    updateLocalization();
  }

  /** Publishes the robot orientation to the Limelight, required by MegaTag2. */
  private void updateGyro() {
    ChassisSpeeds speeds = Drive.mInstance.getChassisSpeeds();
    LimelightHelpers.SetRobotOrientation(
        name,
        Drive.mInstance.getRotation().getDegrees(),
        Math.toDegrees(speeds.omegaRadiansPerSecond),
        0.0,
        0.0,
        0.0,
        0.0);
  }

  private void updateLocalization() {
    // Reject vision while moving fast; MegaTag2 estimates degrade under speed.
    ChassisSpeeds speeds = Drive.mInstance.getChassisSpeeds();
    if (Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) > MAX_LINEAR_SPEED_MPS
        || Math.abs(speeds.omegaRadiansPerSecond) > MAX_ANGULAR_SPEED_RAD_PER_SEC) {
      return;
    }

    PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    if (estimate == null
        || (estimate.pose.getX() == 0.0 && estimate.pose.getY() == 0.0)
        || estimate.tagCount < 1) {
      return;
    }

    double xyStdDev = TA_TO_XY_STD_DEV_METERS.get(Math.max(0.0, LimelightHelpers.getTA(name)));
    Drive.mInstance.addVisionMeasurement(
        estimate.pose,
        estimate.timestampSeconds,
        VecBuilder.fill(xyStdDev, xyStdDev, THETA_STD_DEV_RAD));

    Logger.recordOutput(name + "/Estimate/Pose", estimate.pose);
    Logger.recordOutput(name + "/Estimate/TimestampSeconds", estimate.timestampSeconds);
    Logger.recordOutput(name + "/Estimate/TagCount", estimate.tagCount);
    Logger.recordOutput(name + "/Estimate/AverageTagDistance", estimate.avgTagDist);
  }

  /** Connected if the "tv" entry has been updated within the last 250ms. */
  private boolean isConnected() {
    return (RobotController.getFPGATime()
            - LimelightHelpers.getLimelightNTTableEntry(name, "tv").getLastChange())
        < 250_000;
  }

  public void disable() {
    enabled = false;
  }

  public void enable() {
    enabled = true;
  }

  public boolean getEnabled() {
    return enabled;
  }

  private static InterpolatingDoubleTreeMap taToXYStdDevMeters() {
    InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();
    map.put(0.0, 4.00); // Barely visible, almost no trust
    map.put(0.5, 0.50);
    map.put(1.0, 0.30);
    map.put(3.0, 0.10); // Close tag, high trust
    return map;
  }
}
