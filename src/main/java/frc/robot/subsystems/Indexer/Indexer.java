package frc.robot.subsystems.Indexer;

import frc.lib.bases.MotorSubsystem;
import frc.lib.io.MotorIO;
import frc.lib.io.MotorIOPhoenix6;
import frc.lib.io.MotorIOSim;
import frc.robot.Constants;

public class Indexer extends MotorSubsystem {
  public static final Indexer mInstance = new Indexer();

  private Indexer() {
    super(
        switch (Constants.currentMode) {
          case REAL -> new MotorIOPhoenix6(IndexerConstants.Indexer_CONFIG);
          case SIM -> new MotorIOSim(IndexerConstants.Indexer_SIM_CONFIG);
          case REPLAY -> new MotorIO() {};
        },
        "Indexer",
        "Disconnected Indexer motor.");
  }
}
