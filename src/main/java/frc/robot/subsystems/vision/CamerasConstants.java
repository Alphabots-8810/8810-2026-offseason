package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import frc.lib.bases.CameraSubsystem.CameraIOConfig;
import frc.lib.bases.CameraSubsystem.VisionConfig;
import frc.lib.io.vision.CameraIO;
import frc.lib.io.vision.limelight.AprilTagLimelightCameraIO;

public class CamerasConstants {

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

  public static VisionConfig getConfig() {
    VisionConfig config = new VisionConfig();
    config.name = "Vision";
    config.cameras = new CameraIO[] {LimelightConstants.getIO()};
    return config;
  }
}
