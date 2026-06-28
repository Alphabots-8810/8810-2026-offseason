package frc.robot.commands.ManualCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drum.Drum;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.IntakeDeploy.IntakeDeploy;
import frc.robot.subsystems.IntakeRoller.IntakeRoller;

public class Manual extends Command {
  private enum States {
    PREPARE,
    SHOOT
  }

  private States state;

  public Manual() {
    addRequirements(
        Drum.mInstance,
        Feeder.mInstance,
        Hood.mInstance,
        Indexer.mInstance,
        IntakeDeploy.mInstance,
        IntakeRoller.mInstance);
  }

  @Override
  public void initialize() {
    state = States.PREPARE;
  }

  private boolean isDrumAtSpeed() {
    double target = ManualConstants.ShooterRotpsTunable.getAsDouble();
    return Math.abs(Drum.mInstance.getVelocityRotPerSec() - target)
        < ManualConstants.SHOOTER_VELOCITY_TOLERANCE_ROTPS;
  }

  private boolean isHoodAtAngle() {
    double target = ManualConstants.HoodDegTunable.getAsDouble() / 360.;
    return Math.abs(Hood.mInstance.getPositionRot() - target)
        < ManualConstants.HOOD_ANGLE_TOLERANCE_ROT;
  }

  private void prepare() {
    Drum.mInstance.setVelocityRotPerSec(ManualConstants.ShooterRotpsTunable.getAsDouble());
    Hood.mInstance.setPositionRot(ManualConstants.HoodDegTunable.getAsDouble() / 360.);
    IntakeRoller.mInstance.setV(0);
    Indexer.mInstance.setV(0);
    Feeder.mInstance.setV(0);
  }

  private void shoot() {
    Drum.mInstance.setVelocityRotPerSec(ManualConstants.ShooterRotpsTunable.getAsDouble());
    Hood.mInstance.setPositionRot(ManualConstants.HoodDegTunable.getAsDouble() / 360.);
    IntakeRoller.mInstance.setVelocityRotPerSec(
        ManualConstants.IntakeRollerRotpsTunable.getAsDouble());
    Indexer.mInstance.setVelocityRotPerSec(ManualConstants.IndexerRotpsTunable.getAsDouble());
    Feeder.mInstance.setVelocityRotPerSec(ManualConstants.FeederRotpsTunable.getAsDouble());
  }

  @Override
  public void execute() {
    switch (state) {
      case PREPARE:
        prepare();
        if (isDrumAtSpeed() && isHoodAtAngle()) {
          state = States.SHOOT;
        }
        break;
      case SHOOT:
        shoot();
        break;
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Drum.mInstance.stop();
    Hood.mInstance.stop();
    IntakeRoller.mInstance.stop();
    Indexer.mInstance.stop();
    Feeder.mInstance.stop();
  }
}
