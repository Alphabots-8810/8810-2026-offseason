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
  private final double intakeDeployCruiseVelocity;
  private final double intakeDeployAcceleration;
  private final boolean preloadEnabled;

  public IntakeCommand() {
    this(
        IntakeCommandConstants.INTAKE_DEPLOY_CRUISE_VELOCITY,
        IntakeCommandConstants.INTAKE_DEPLOY_ACCELERATION,
        true);
  }

  /** Creates an intake command with custom deploy Motion Magic constraints and preload behavior. */
  public IntakeCommand(
      double intakeDeployCruiseVelocity, double intakeDeployAcceleration, boolean preloadEnabled) {
    this.intakeDeployCruiseVelocity = intakeDeployCruiseVelocity;
    this.intakeDeployAcceleration = intakeDeployAcceleration;
    this.preloadEnabled = preloadEnabled;
    addRequirements(
        IntakeRoller.mInstance, IntakeDeploy.mInstance, Indexer.mInstance, Feeder.mInstance);
  }

  @Override
  public void initialize() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      FuelSimulation.setIntakeRunning(true);
    }
    runIntake();
    updatePreload();
  }

  @Override
  public void execute() {
    runIntake();
    updatePreload();
  }

  private void runIntake() {
    IntakeRoller.mInstance.setVelocityRotPerSec(IntakeCommandConstants.INTAKE_ROLLER_ROTPS);
    IntakeDeploy.mInstance.setPositionCentimeter(
        IntakeCommandConstants.INTAKE_DEPLOY_POSITION_CM,
        intakeDeployCruiseVelocity,
        intakeDeployAcceleration,
        IntakeCommandConstants.INTAKE_DEPLOY_JERK);
  }

  private void updateIndexer() {
    // Preload only after the hopper CANrange sees a game piece, and stop as soon as the
    // indexer CANrange confirms it has reached the staged position.
    if (FeedPath.mInstance.HopperFilled() && !FeedPath.mInstance.IndexerFilled()) {
      runIndexing();
    } else {
      stopIndexing();
    }
  }

  private void updatePreload() {
    if (preloadEnabled) {
      updateIndexer();
    } else {
      stopIndexing();
    }
  }

  private void runIndexing() {
    Indexer.mInstance.setV(IntakeCommandConstants.INDEXING_VOLTAGE);
    Feeder.mInstance.setV(IntakeCommandConstants.INDEXING_VOLTAGE);
  }

  private void stopIndexing() {
    Indexer.mInstance.stop();
    Feeder.mInstance.stop();
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
