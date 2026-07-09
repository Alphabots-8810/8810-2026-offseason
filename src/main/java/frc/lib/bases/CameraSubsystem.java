package frc.lib.bases;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.vision.CameraIO;
import frc.lib.util.vision.VisionEstimate;
import frc.robot.subsystems.drive.Drive;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class CameraSubsystem extends SubsystemBase {

  private final VisionConfig config;
  private final Alert[] disconnectedAlerts;

  private boolean enabled = true;
  private Pose2d lastPose = new Pose2d();
  private double lastUpdatePoseTimeSeconds = 0.0;

  public CameraSubsystem(VisionConfig config) {
    super(config.name);
    this.config = config;

    disconnectedAlerts = new Alert[config.cameras.length];
    for (int i = 0; i < config.cameras.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Disconnected vision camera \"" + config.cameras[i].getName() + "\".",
              AlertType.kWarning);
    }
  }

  @Override
  public void periodic() {
    if (!enabled) return;

    for (int i = 0; i < config.cameras.length; i++) {
      CameraIO camera = config.cameras[i];
      camera.update();
      disconnectedAlerts[i].set(!camera.isConnected());
      Logger.recordOutput(
          config.name + "/" + camera.getName() + "/Connected", camera.isConnected());
    }

    updateLocalization();

    Logger.recordOutput(config.name + "/LastPose", lastPose);
    Logger.recordOutput(config.name + "/LastUpdatePoseTime", lastUpdatePoseTimeSeconds);
  }

  private void updateLocalization() {
    // Reject vision while moving fast; MegaTag2 estimates degrade under speed
    ChassisSpeeds speeds = Drive.mInstance.getChassisSpeeds();
    if (Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
            > config.maxLinearSpeedMetersPerSec
        || Math.abs(speeds.omegaRadiansPerSecond) > config.maxAngularSpeedRadPerSec) {
      return;
    }

    for (CameraIO camera : config.cameras) {
      Optional<List<VisionEstimate>> estimatesOptional = camera.getLastEstimates();
      estimatesOptional.ifPresent(
          estimates -> {
            for (VisionEstimate estimate : estimates) {
              applyVisionEstimate(camera, estimate);
            }
          });
    }
  }

  public void applyVisionEstimate(CameraIO camera, VisionEstimate estimate) {
    Drive.mInstance.addVisionMeasurement(
        estimate.getPose(), estimate.getTimestampSeconds(), camera.getEstimateStdDevs(estimate));

    estimate.log(config.name + "/" + camera.getName() + "/Estimate");
    lastPose = estimate.getPose();
    lastUpdatePoseTimeSeconds = Timer.getFPGATimestamp();
  }

  public Pose2d getLatestUpdate() {
    return lastPose;
  }

  public double getLastUpdatedPoseTimeSeconds() {
    return lastUpdatePoseTimeSeconds;
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

  public static class CameraIOConfig {
    public String name = null;

    // Null = use the transform configured on the camera itself (e.g. the Limelight web UI)
    public Pose3d robotToCameraOffset = null;

    // TA (target area, 0-100) maps to XY std dev in meters. Larger TA = closer tag.
    public InterpolatingDoubleTreeMap taToXYStdDevMeters = defaultTaToXYStdDevMeters();

    // Heading is always locked to the gyro, so give rotation a huge std dev.
    public double thetaStdDevRad = 99999999.0;

    private static InterpolatingDoubleTreeMap defaultTaToXYStdDevMeters() {
      InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();
      map.put(0.0, 4.00); // Barely visible, almost no trust
      map.put(0.5, 0.50);
      map.put(1.0, 0.30);
      map.put(3.0, 0.10); // Close tag, high trust
      return map;
    }
  }

  public static class VisionConfig {
    public String name = "Vision";
    public CameraIO[] cameras = new CameraIO[0];

    // Reject vision while moving faster than these thresholds
    public double maxLinearSpeedMetersPerSec = 2.0;
    public double maxAngularSpeedRadPerSec = 3.0;
  }
}
