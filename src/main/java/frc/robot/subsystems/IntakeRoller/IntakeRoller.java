package frc.robot.subsystems.IntakeRoller;

import frc.lib.bases.MotorSubsystem;
import frc.lib.io.MotorIO;
import frc.lib.io.MotorIOPhoenix6;
import frc.lib.io.MotorIOSim;
import frc.robot.Constants;

public class IntakeRoller extends MotorSubsystem {
  public static final IntakeRoller mInstance = new IntakeRoller();

  private IntakeRoller() {
    super(
        switch (Constants.currentMode) {
          case REAL -> new MotorIOPhoenix6(IntakeRollerConstants.IntakeRoller_CONFIG);
          case SIM -> new MotorIOSim(IntakeRollerConstants.IntakeRoller_SIM_CONFIG);
          case REPLAY -> new MotorIO() {};
        },
        "IntakeRoller",
        "Disconnected IntakeRoller motor.");
  }
}
