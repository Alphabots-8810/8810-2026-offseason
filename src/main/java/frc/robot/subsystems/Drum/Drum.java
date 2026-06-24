package frc.robot.subsystems.Drum;

import frc.lib.bases.MotorSubsystem;
import frc.lib.io.MotorIO;
import frc.lib.io.MotorIOPhoenix6;
import frc.lib.io.MotorIOSim;
import frc.robot.Constants;

public class Drum extends MotorSubsystem {
  public static final Drum mInstance = new Drum();

  private Drum() {
    super(
        switch (Constants.currentMode) {
          case REAL -> new MotorIOPhoenix6(DrumConstants.Drum_CONFIG);
          case SIM -> new MotorIOSim(DrumConstants.Drum_SIM_CONFIG);
          case REPLAY -> new MotorIO() {};
        },
        "Drum",
        "Disconnected Drum motor.");
  }
}
