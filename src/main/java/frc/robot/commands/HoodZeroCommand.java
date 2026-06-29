package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Hood.HoodConstants;

public class HoodZeroCommand extends Command {
  private static final double ZEROING_VOLTAGE = -3.0;
  private static final double CURRENT_THRESHOLD = 40.0;
  private static final double DEBOUNCE_TIME_SECONDS = 0.3;

  private final Debouncer currentDebouncer =
      new Debouncer(DEBOUNCE_TIME_SECONDS, DebounceType.kRising);

  public HoodZeroCommand() {
    addRequirements(Hood.mInstance);
  }

  @Override
  public void initialize() {
    Hood.mInstance.setV(ZEROING_VOLTAGE);
  }

  @Override
  public boolean isFinished() {
    return currentDebouncer.calculate(
        Math.abs(Hood.mInstance.getStatorCurrentAmps()) > CURRENT_THRESHOLD);
  }

  @Override
  public void end(boolean interrupted) {
    Hood.mInstance.stop();
    if (!interrupted) {
      Hood.mInstance.setEncoderPositionRot(HoodConstants.HoodZeroPosition);
    }
  }
}
