package frc.robot.commands.HoodZeroCommand;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Hood.HoodConstants;

public class HoodZeroCommand extends Command {

  private final Debouncer currentDebouncer =
      new Debouncer(HoodZeroCommandConstants.DEBOUNCE_TIME_SECONDS, DebounceType.kRising);

  public HoodZeroCommand() {
    addRequirements(Hood.mInstance);
  }

  @Override
  public void initialize() {
    Hood.mInstance.setV(HoodZeroCommandConstants.ZEROING_VOLTAGE);
  }

  @Override
  public boolean isFinished() {
    return currentDebouncer.calculate(
        Math.abs(Hood.mInstance.getStatorCurrentAmps())
            > HoodZeroCommandConstants.CURRENT_THRESHOLD);
  }

  @Override
  public void end(boolean interrupted) {
    Hood.mInstance.stop();
    if (!interrupted) {
      Hood.mInstance.setEncoderPositionRot(HoodConstants.HoodZeroPosition);
    }
  }
}
