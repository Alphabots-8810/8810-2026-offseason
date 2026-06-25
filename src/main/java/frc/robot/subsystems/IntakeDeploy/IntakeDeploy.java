package frc.robot.subsystems.IntakeDeploy;

import frc.lib.bases.MotorSubsystem;
import frc.lib.io.MotorIO;
import frc.lib.io.MotorIOPhoenix6;
import frc.lib.io.MotorIOSim;
import frc.robot.Constants;

public class IntakeDeploy extends MotorSubsystem {
  public static final IntakeDeploy mInstance = new IntakeDeploy();

  private IntakeDeploy() {
    super(
        switch (Constants.currentMode) {
          case REAL -> new MotorIOPhoenix6(IntakeDeployConstants.IntakeDeploy_CONFIG);
          case SIM -> new MotorIOSim(IntakeDeployConstants.IntakeDeploy_SIM_CONFIG);
          case REPLAY -> new MotorIO() {};
        },
        "IntakeDeploy",
        "Disconnected IntakeDeploy motor.");
  }
}
