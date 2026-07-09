package frc.robot.subsystems.vision;

import frc.lib.bases.CameraSubsystem;
import frc.robot.Constants;

public class Cameras extends CameraSubsystem {

  public static final Cameras mInstance = new Cameras();

  private Cameras() {
    super(CamerasConstants.getConfig());

    // No camera hardware or vision simulation outside the real robot
    if (Constants.currentMode != Constants.Mode.REAL) {
      disable();
    }
  }
}
