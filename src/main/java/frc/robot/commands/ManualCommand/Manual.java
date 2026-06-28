package frc.robot.commands.ManualCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drum.Drum;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.IntakeDeploy.IntakeDeploy;
import frc.robot.subsystems.IntakeRoller.IntakeRoller;
import frc.robot.subsystems.drive.Drive;

public class Manual extends Command {
  private enum States {
    SHOOT
  } 
  private States state;
  public Manual() {
    addRequirements(Drum.mInstance, Feeder.mInstance, Hood.mInstance, Indexer.mInstance, IntakeDeploy.mInstance, IntakeRoller.mInstance);
  }

  @Override
  public void initialize() {
    state = States.SHOOT;
  }

  private void shoot()
  {
    Drum.mInstance.setVelocityRadPerSec(ManualConstants.ShooterRadpsTunable.getAsDouble());
    Hood.mInstance.setPositionRad(ManualConstants.HoodAngleTunable.getAsDouble());
    IntakeRoller.mInstance.setVelocityRadPerSec(ManualConstants.IntakeRollerRadpsTunable.getAsDouble());
    Indexer.mInstance.setVelocityRadPerSec(ManualConstants.IndexerRadpsTunable.getAsDouble());
    Feeder.mInstance.setVelocityRadPerSec(ManualConstants.FeederRadpsTunable.getAsDouble());
  }
  private void retract()
  {

  }

  @Override
  public void execute() {
    switch(state)
    {
        case SHOOT -> shoot();
    };
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
