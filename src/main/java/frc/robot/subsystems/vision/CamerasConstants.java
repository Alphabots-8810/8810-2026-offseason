package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.lib.bases.CameraSubsystem.CameraIOConfig;
import frc.lib.bases.CameraSubsystem.VisionConfig;
import frc.lib.io.vision.CameraIO;
import frc.lib.io.vision.limelight.AprilTagLimelightCameraIO;
import frc.lib.io.vision.photon.AprilTagPhotonCameraIO;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class CamerasConstants {

  public static final AprilTagFieldLayout LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  public static final PoseStrategy DEFAULT_APRIL_TAG_STRATEGY =
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

  public static final class LimelightConstants {

    // Null keeps the transform configured in the Limelight web UI; set a Pose3d to
    // override it from code.
    public static final Pose3d OFFSET_FROM_CENTER = null;

    public static CameraIOConfig getConfig() {
      CameraIOConfig config = new CameraIOConfig();
      config.name = "limelight";
      config.robotToCameraOffset = OFFSET_FROM_CENTER;
      return config;
    }

    public static CameraIO getIO() {
      return new AprilTagLimelightCameraIO(getConfig());
    }
  }

  public static final class LeftPhotonConstants {

    public static final PoseStrategy APRIL_TAG_STRATEGY = DEFAULT_APRIL_TAG_STRATEGY;

    // Example mount: left of center, angled up 15 deg and outward 30 deg.
    // Measure the real position before trusting it on the field.
    public static final Pose3d OFFSET_FROM_CENTER =
        new Pose3d(
            Units.inchesToMeters(8.0),
            Units.inchesToMeters(11.0),
            Units.inchesToMeters(9.5),
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(-15.0),
                Units.degreesToRadians(30.0)));

    public static CameraIOConfig getConfig() {
      CameraIOConfig config = new CameraIOConfig();
      config.name = "left";
      config.robotToCameraOffset = OFFSET_FROM_CENTER;
      return config;
    }

    public static CameraIO getIO() {
      return new AprilTagPhotonCameraIO(getConfig(), APRIL_TAG_STRATEGY, LAYOUT);
    }
  }

  public static final class RightPhotonConstants {

    public static final PoseStrategy APRIL_TAG_STRATEGY = DEFAULT_APRIL_TAG_STRATEGY;

    // Mirror of the left camera across the robot centerline
    public static final Pose3d OFFSET_FROM_CENTER =
        new Pose3d(
            Units.inchesToMeters(8.0),
            Units.inchesToMeters(-11.0),
            Units.inchesToMeters(9.5),
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(-15.0),
                Units.degreesToRadians(-30.0)));

    public static CameraIOConfig getConfig() {
      CameraIOConfig config = new CameraIOConfig();
      config.name = "right";
      config.robotToCameraOffset = OFFSET_FROM_CENTER;
      return config;
    }

    public static CameraIO getIO() {
      return new AprilTagPhotonCameraIO(getConfig(), APRIL_TAG_STRATEGY, LAYOUT);
    }
  }

  public static VisionConfig getConfig() {
    VisionConfig config = new VisionConfig();
    config.name = "Vision";
    // Only the Limelight is installed for now. When the PhotonVision cameras are
    // added, list every installed camera here, e.g. for the duo PhotonVision setup:
    // config.cameras =
    //     new CameraIO[] {
    //       LimelightConstants.getIO(), LeftPhotonConstants.getIO(), RightPhotonConstants.getIO()
    //     };
    config.cameras = new CameraIO[] {LimelightConstants.getIO()};
    return config;
  }
}
