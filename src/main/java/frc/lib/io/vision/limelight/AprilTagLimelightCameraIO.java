package frc.lib.io.vision.limelight;

import frc.lib.bases.CameraSubsystem.CameraIOConfig;
import frc.lib.util.vision.VisionEstimate;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import java.util.List;
import java.util.Optional;

/** Camera IO for a Limelight running MegaTag2 AprilTag pose estimation. */
public class AprilTagLimelightCameraIO extends LimelightCameraIO {

  public AprilTagLimelightCameraIO(CameraIOConfig config) {
    super(config);
  }

  /** Publishes the robot orientation to the Limelight, required by MegaTag2. */
  private void updateGyro() {
    var speeds = Drive.mInstance.getChassisSpeeds();
    LimelightHelpers.SetRobotOrientation(
        config.name,
        Drive.mInstance.getRotation().getDegrees(),
        Math.toDegrees(speeds.omegaRadiansPerSecond),
        0.0,
        0.0,
        0.0,
        0.0);
  }

  @Override
  public Optional<List<VisionEstimate>> getLastEstimates() {
    PoseEstimate limelightEstimate =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(config.name);
    if (limelightEstimate == null
        || (limelightEstimate.pose.getX() == 0.0 && limelightEstimate.pose.getY() == 0.0)
        || limelightEstimate.tagCount < 1) {
      return Optional.empty();
    }

    int[] tagIds = new int[limelightEstimate.rawFiducials.length];
    for (int i = 0; i < tagIds.length; i++) {
      tagIds[i] = limelightEstimate.rawFiducials[i].id;
    }

    VisionEstimate estimate =
        new VisionEstimate(limelightEstimate.pose, limelightEstimate.timestampSeconds, tagIds)
            .withAverageDistance(limelightEstimate.avgTagDist)
            .withTargetArea(LimelightHelpers.getTA(config.name));
    return Optional.of(List.of(estimate));
  }

  @Override
  public void update() {
    updateGyro();
  }
}
