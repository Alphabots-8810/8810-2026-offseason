package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

/**
 * Locks one swerve module in place and rotates the robot around it as a pivot point.
 *
 * <p>The pivot module receives zero drive velocity while the remaining three modules are driven
 * tangentially around the pivot. Full joystick deflection maps to the maximum angular speed
 * achievable given the pivot geometry.
 */
public class CornerPivotCommand extends Command {

  /**
   * Which corner module to use as the fixed pivot. Indices match Drive's module array (FL=0, FR=1,
   * BL=2, BR=3).
   */
  public enum PivotCorner {
    FRONT_LEFT(0),
    FRONT_RIGHT(1),
    BACK_LEFT(2),
    BACK_RIGHT(3);

    public final int index;

    PivotCorner(int index) {
      this.index = index;
    }
  }

  private final Drive drive;
  private final Translation2d pivotTranslation;
  private final DoubleSupplier omegaSupplier;
  private final double maxAngularSpeedRadPerSec;

  /**
   * @param drive The drive subsystem
   * @param corner Which module to pivot around
   * @param omegaSupplier Joystick rotation input [-1, 1]; positive = counter-clockwise
   */
  public CornerPivotCommand(Drive drive, PivotCorner corner, DoubleSupplier omegaSupplier) {
    this.drive = drive;
    this.omegaSupplier = omegaSupplier;

    Translation2d[] translations = Drive.getModuleTranslations();
    pivotTranslation = translations[corner.index];

    // Compute the distance from the pivot to the farthest module so full stick
    // maps to the maximum achievable angular speed (wheel speed = max linear speed).
    double maxRadius = 0.0;
    for (Translation2d t : translations) {
      maxRadius = Math.max(maxRadius, t.getDistance(pivotTranslation));
    }
    maxAngularSpeedRadPerSec = drive.getMaxLinearSpeedMetersPerSec() / maxRadius;

    addRequirements(drive);
  }

  @Override
  public void execute() {
    double omega =
        MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DriveCommandsConstants.DEADBAND);
    omega = Math.copySign(omega * omega, omega);

    drive.runVelocityWithCenterOfRotation(
        new ChassisSpeeds(0.0, 0.0, omega * maxAngularSpeedRadPerSec), pivotTranslation);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
