package frc.robot.commands.ShootingCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drum.Drum;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.IntakeDeploy.IntakeDeploy;
import frc.robot.subsystems.IntakeRoller.IntakeRoller;
import frc.robot.subsystems.drive.Drive;

public class Shooting extends Command {
  private enum States {
    AIM,
    SHOOT
  }

  private States state;

  public Shooting() {
    addRequirements(
        Drive.mInstance,
        Drum.mInstance,
        Feeder.mInstance,
        Hood.mInstance,
        Indexer.mInstance,
        IntakeDeploy.mInstance,
        IntakeRoller.mInstance);
  }

  @Override
  public void initialize() {
    state = States.SHOOT;
  }

  // private double AngleToHub()
  // {

  // }

  private void aim() {}

  private void shoot() {
    IntakeRoller.mInstance.setVelocityRotPerSec(
        ShootingConstants.IntakeRollerRotpsTunable.getAsDouble());
    Indexer.mInstance.setVelocityRotPerSec(ShootingConstants.IndexerRotpsTunable.getAsDouble());
    Feeder.mInstance.setVelocityRotPerSec(ShootingConstants.FeederRotpsTunable.getAsDouble());
  }

  private void retract() {}

  @Override
  public void execute() {
    switch (state) {
      case AIM -> aim();
      case SHOOT -> shoot();
    }
    ;
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
