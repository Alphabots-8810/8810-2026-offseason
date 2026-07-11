package frc.robot.commands.IntakeDeployZeroCommand;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeDeploy.IntakeDeploy;
import frc.robot.subsystems.IntakeDeploy.IntakeDeployConstants;

/**
 * Current-spike homing of the intake deploy against a hard stop.
 *
 * <p>{@code OUTWARD} drives to the deployed hard stop and resets the encoder to {@code
 * IntakeOutPosition} (teleop re-zero). {@code INWARD} drives into the stowed hard stop and resets
 * to 0 — used at auto start, where the intake already sits on the stowed stop, so the spike is
 * nearly immediate and the normal IntakeCommand can deploy right after.
 */
public class IntakeDeployZeroCommand extends Command {

  public enum Direction {
    OUTWARD,
    INWARD
  }

  private final Direction direction;
  private Debouncer currentDebouncer;

  public IntakeDeployZeroCommand(Direction direction) {
    this.direction = direction;
    addRequirements(IntakeDeploy.mInstance);
  }

  @Override
  public void initialize() {
    // Fresh debouncer each run so state never leaks between schedulings.
    currentDebouncer =
        new Debouncer(
            direction == Direction.OUTWARD
                ? IntakeDeployZeroCommandConstants.OUTWARD_DEBOUNCE_TIME_SECONDS
                : IntakeDeployZeroCommandConstants.INWARD_DEBOUNCE_TIME_SECONDS,
            DebounceType.kRising);
    IntakeDeploy.mInstance.setV(
        direction == Direction.OUTWARD
            ? IntakeDeployZeroCommandConstants.OUTWARD_ZEROING_VOLTAGE
            : IntakeDeployZeroCommandConstants.INWARD_ZEROING_VOLTAGE);
  }

  @Override
  public boolean isFinished() {
    return currentDebouncer.calculate(
        Math.abs(IntakeDeploy.mInstance.getStatorCurrentAmps())
            > IntakeDeployZeroCommandConstants.CURRENT_THRESHOLD);
  }

  @Override
  public void end(boolean interrupted) {
    IntakeDeploy.mInstance.stop();
    if (!interrupted) {
      IntakeDeploy.mInstance.resetEncoderPositionCentimeter(
          direction == Direction.OUTWARD
              ? IntakeDeployConstants.IntakeOutPosition
              : IntakeDeployZeroCommandConstants.INWARD_ZERO_POSITION_CM);
    }
  }
}
