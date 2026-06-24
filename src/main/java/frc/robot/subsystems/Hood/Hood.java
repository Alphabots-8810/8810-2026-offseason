package frc.robot.subsystems.Hood;

import frc.lib.bases.MotorSubsystem;
import frc.lib.io.MotorIO;
import frc.lib.io.MotorIOPhoenix6;
import frc.lib.io.MotorIOSim;
import frc.robot.Constants;

public class Hood extends MotorSubsystem {
  public static final Hood mInstance = new Hood();

  private Hood() {
    super(
        switch (Constants.currentMode) {
          case REAL -> new MotorIOPhoenix6(HoodConstants.Hood_CONFIG);
          case SIM -> new MotorIOSim(HoodConstants.Hood_SIM_CONFIG);
          case REPLAY -> new MotorIO() {};
        },
        "Hood",
        "Disconnected Hood motor.");
  }
}
