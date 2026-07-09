package frc.robot.commands.IntakeCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.simulation.FuelSimulation;
import frc.robot.subsystems.FeedPath.FeedPath;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.IntakeDeploy.IntakeDeploy;
import frc.robot.subsystems.IntakeRoller.IntakeRoller;

public class IntakeCommand extends Command {
  public IntakeCommand() {
    addRequirements(
        IntakeRoller.mInstance, IntakeDeploy.mInstance, Indexer.mInstance, Feeder.mInstance);
  }

  @Override
  public void initialize() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      FuelSimulation.setIntakeRunning(true);
    }
    runIntake();
    updateIndexer();
  }

  @Override
  public void execute() {
    runIntake();
    updateIndexer();
  }

  private void runIntake() {
    IntakeRoller.mInstance.setVelocityRotPerSec(IntakeCommandConstants.INTAKE_ROLLER_ROTPS);
    IntakeDeploy.mInstance.setPositionCentimeter(
        IntakeCommandConstants.INTAKE_DEPLOY_POSITION_CM,
        IntakeCommandConstants.INTAKE_DEPLOY_CRUISE_VELOCITY,
        IntakeCommandConstants.INTAKE_DEPLOY_ACCELERATION,
        IntakeCommandConstants.INTAKE_DEPLOY_JERK);
  }

  private void updateIndexer() {
    if (FeedPath.mInstance.IndexerFilled()) {
      stopIndexing();
    } else if (FeedPath.mInstance.HopperFilled()) {
      runIndexing();
    } else {
      stopIndexing();
    }
  }

  private void runIndexing() {
    Indexer.mInstance.setV(IntakeCommandConstants.INDEXING_VOLTAGE);
  }

  private void stopIndexing() {
    Indexer.mInstance.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    if (Constants.currentMode == Constants.Mode.SIM) {
      FuelSimulation.setIntakeRunning(false);
    }
    IntakeRoller.mInstance.stop();
    stopIndexing();
  }
}
