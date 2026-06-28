package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drum.Drum;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.IntakeDeploy.IntakeDeploy;
import frc.robot.subsystems.IntakeRoller.IntakeRoller;
import frc.robot.subsystems.drive.Drive;

public class AimCommand extends Command {
  private enum States {
    Aim,
    Shoot,
    Retract
  }

  private final Drive drive;
  private final Drum drum;
  private final Feeder feeder;
  private final Hood hood;
  private final Indexer indexer;
  private final IntakeDeploy intakeDeploy;
  private final IntakeRoller intakeRoller;

  public AimCommand() {
    this.drive = Drive.mInstance;
    this.drum = Drum.mInstance;
    this.feeder = Feeder.mInstance;
    this.hood = Hood.mInstance;
    this.indexer = Indexer.mInstance;
    this.intakeDeploy = IntakeDeploy.mInstance;
    this.intakeRoller = IntakeRoller.mInstance;
    addRequirements(drive, drum, feeder, hood, indexer, intakeDeploy, intakeRoller);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
