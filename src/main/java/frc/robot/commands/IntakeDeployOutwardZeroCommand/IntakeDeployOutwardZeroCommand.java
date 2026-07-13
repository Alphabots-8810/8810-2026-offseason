package frc.robot.commands.IntakeDeployOutwardZeroCommand;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeDeploy.IntakeDeploy;
import frc.robot.subsystems.IntakeDeploy.IntakeDeployConstants;

public class IntakeDeployOutwardZeroCommand extends Command {

  private final Debouncer currentDebouncer =
      new Debouncer(
          IntakeDeployOutwardZeroCommandConstants.DEBOUNCE_TIME_SECONDS, DebounceType.kRising);

  public IntakeDeployOutwardZeroCommand() {
    addRequirements(IntakeDeploy.mInstance);
  }

  @Override
  public void initialize() {
    IntakeDeploy.mInstance.setV(IntakeDeployOutwardZeroCommandConstants.ZEROING_VOLTAGE);
  }

  @Override
  public boolean isFinished() {
    return currentDebouncer.calculate(
        Math.abs(IntakeDeploy.mInstance.getStatorCurrentAmps())
            > IntakeDeployOutwardZeroCommandConstants.CURRENT_THRESHOLD);
  }

  @Override
  public void end(boolean interrupted) {
    IntakeDeploy.mInstance.stop();
    if (!interrupted) {
      IntakeDeploy.mInstance.resetEncoderPositionCentimeter(
          IntakeDeployConstants.IntakeOutPosition);
    }
  }
}
