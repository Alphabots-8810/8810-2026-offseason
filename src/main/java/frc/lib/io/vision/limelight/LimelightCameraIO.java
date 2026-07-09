package frc.lib.io.vision.limelight;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.bases.CameraSubsystem.CameraIOConfig;
import frc.lib.io.vision.CameraIO;
import frc.robot.util.LimelightHelpers;

public abstract class LimelightCameraIO extends CameraIO {

  public LimelightCameraIO(CameraIOConfig config) {
    super(config);
    // A null offset keeps the transform configured in the Limelight web UI
    if (RobotBase.isReal() && config.robotToCameraOffset != null) {
      LimelightHelpers.setCameraPose_RobotSpace(
          config.name,
          config.robotToCameraOffset.getX(),
          config.robotToCameraOffset.getY(),
          config.robotToCameraOffset.getZ(),
          Units.radiansToDegrees(config.robotToCameraOffset.getRotation().getX()),
          Units.radiansToDegrees(config.robotToCameraOffset.getRotation().getY()),
          Units.radiansToDegrees(config.robotToCameraOffset.getRotation().getZ()));
    }
  }

  @Override
  public boolean isConnected() {
    // Connected if the "tv" entry has been updated within the last 250ms
    return (RobotController.getFPGATime()
            - LimelightHelpers.getLimelightNTTableEntry(config.name, "tv").getLastChange())
        < 250_000;
  }
}
