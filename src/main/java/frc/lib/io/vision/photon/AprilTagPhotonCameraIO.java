package frc.lib.io.vision.photon;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.bases.CameraSubsystem.CameraIOConfig;
import frc.lib.util.vision.VisionEstimate;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

/** Camera IO for a PhotonVision camera running AprilTag pose estimation. */
public class AprilTagPhotonCameraIO extends PhotonCameraIO {

  private final PhotonPoseEstimator estimator;
  private final PoseStrategy strategy;

  public AprilTagPhotonCameraIO(
      CameraIOConfig config, PoseStrategy strategy, AprilTagFieldLayout layout) {
    super(config);
    this.strategy = strategy;
    estimator =
        new PhotonPoseEstimator(
            layout,
            new Transform3d(
                config.robotToCameraOffset.getTranslation(),
                config.robotToCameraOffset.getRotation()));
  }

  private Optional<EstimatedRobotPose> getPoseEstimateWithStrategy(PhotonPipelineResult result) {
    switch (strategy) {
      case MULTI_TAG_PNP_ON_COPROCESSOR:
        return estimator.estimateAverageBestTargetsPose(result);
      case PNP_DISTANCE_TRIG_SOLVE:
        return estimator.estimatePnpDistanceTrigSolvePose(result);
      case AVERAGE_BEST_TARGETS:
        return estimator.estimateAverageBestTargetsPose(result);
      case CLOSEST_TO_CAMERA_HEIGHT:
        return estimator.estimateClosestToCameraHeightPose(result);
      case LOWEST_AMBIGUITY:
        return estimator.estimateLowestAmbiguityPose(result);
      default:
        return estimator.estimateLowestAmbiguityPose(result);
    }
  }

  private Optional<VisionEstimate> convertRawEstimate(PhotonPipelineResult result) {
    Optional<EstimatedRobotPose> updated = getPoseEstimateWithStrategy(result);
    if (updated.isEmpty()) {
      return Optional.empty();
    }
    EstimatedRobotPose estimate = updated.get();

    int[] tagIds = new int[estimate.targetsUsed.size()];
    double totalDistance = 0.0;
    double totalArea = 0.0;
    for (int i = 0; i < tagIds.length; i++) {
      var target = estimate.targetsUsed.get(i);
      tagIds[i] = target.fiducialId;
      totalDistance += target.bestCameraToTarget.getTranslation().getNorm();
      totalArea += target.getArea();
    }
    int count = Math.max(1, tagIds.length);

    return Optional.of(
        new VisionEstimate(estimate.estimatedPose.toPose2d(), estimate.timestampSeconds, tagIds)
            .withAverageDistance(totalDistance / count)
            .withTargetArea(totalArea / count));
  }

  @Override
  public Optional<List<VisionEstimate>> getLastEstimates() {
    if (getLastInputBuffer().isEmpty()) {
      return Optional.empty();
    }

    ArrayList<VisionEstimate> buffer = new ArrayList<>();
    for (PhotonPipelineResult result : getLastInputBuffer()) {
      convertRawEstimate(result).ifPresent(buffer::add);
    }
    return buffer.isEmpty() ? Optional.empty() : Optional.of(buffer);
  }

  @Override
  public void update() {
    super.update();
    estimator.addHeadingData(Timer.getFPGATimestamp(), Drive.mInstance.getRotation());
  }
}
