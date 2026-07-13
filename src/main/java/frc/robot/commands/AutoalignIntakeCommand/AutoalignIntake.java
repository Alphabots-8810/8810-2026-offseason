package frc.robot.commands.AutoalignIntakeCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.IntakeCommand.IntakeCommandConstants;
import frc.robot.simulation.FuelSimulation;
import frc.robot.subsystems.FeedPath.FeedPath;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.IntakeRoller.IntakeRoller;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Drives field-relative from the joysticks while running the intake and using a profiled PID
 * controller to keep the <b>intake</b> pointed along the robot's direction of motion (so the intake
 * leads into whatever the driver is translating toward). The intake is on the robot's tail, so the
 * target heading is 180 deg from the direction of motion. When the sticks are inside the deadband
 * the last commanded heading is held.
 */
public class AutoalignIntake extends Command {
  private final Drive drive;
  private final IntakeRoller roller;
  private final Indexer indexer;
  private final Feeder feeder;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier manualFixSupplier;

  private final ProfiledPIDController angleController;
  private Rotation2d targetHeading = Rotation2d.kZero;

  public AutoalignIntake(
      DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier manualFixSupplier) {
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.drive = Drive.mInstance;
    this.roller = IntakeRoller.mInstance;
    this.indexer = Indexer.mInstance;
    this.feeder = Feeder.mInstance;
    this.manualFixSupplier = manualFixSupplier;
    angleController =
        new ProfiledPIDController(
            AutoalignIntakeConstants.ANGLE_KP,
            0.0,
            AutoalignIntakeConstants.ANGLE_KD,
            new TrapezoidProfile.Constraints(
                AutoalignIntakeConstants.ANGLE_MAX_VELOCITY,
                AutoalignIntakeConstants.ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drive, roller, feeder, indexer);
  }

  @Override
  public void initialize() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      FuelSimulation.setIntakeRunning(true);
    }
    // Hold the current heading until the driver gives a motion direction.
    targetHeading = drive.getRotation();
    angleController.reset(drive.getRotation().getRadians());
  }

  @Override
  public void execute() {
    Logger.recordOutput("AutoalignIntake/Setpoint", angleController.getSetpoint());
    Logger.recordOutput("AutoalignIntake/Position", drive.getRotation().getRadians());
    roller.setVelocityRotPerSec(AutoalignIntakeConstants.ROLLER_VELOCITY_ROT_PER_SEC);

    // Linear velocity (unitless, field-relative) from the joysticks.
    Translation2d linearVelocity =
        getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    // Only update the heading goal when the driver is actually translating, otherwise the
    // direction is undefined (atan2(0, 0)) and we would snap to an arbitrary angle.
    if (linearVelocity.getNorm() > 1e-6) {
      // Field-relative direction the robot is moving in.
      Rotation2d motionDirection = linearVelocity.getAngle();
      if (isFlipped) {
        motionDirection = motionDirection.plus(Rotation2d.kPi);
      }
      // The intake is on the robot's tail, so point the heading 180 deg away from the direction of
      // motion -> the tail (intake) leads into the travel direction.
      targetHeading = motionDirection.plus(Rotation2d.kPi);
    }
    targetHeading =
        targetHeading.plus(
            new Rotation2d(
                manualFixSupplier.getAsDouble() * AutoalignIntakeConstants.MANUAL_FIX_SCALOR));
    // Profiled PID on heading produces the angular velocity command, clamped so it can't dominate
    // the module speed budget and starve translation.
    double omega =
        MathUtil.clamp(
            angleController.calculate(drive.getRotation().getRadians(), targetHeading.getRadians()),
            -AutoalignIntakeConstants.MAX_ANGULAR_VELOCITY_RAD_PER_SEC,
            AutoalignIntakeConstants.MAX_ANGULAR_VELOCITY_RAD_PER_SEC);

    linearVelocity = linearVelocity.times(Math.cos(angleController.getPositionError()));
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
            omega);

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds, isFlipped ? drive.getRotation().plus(Rotation2d.kPi) : drive.getRotation()));
    if (FeedPath.mInstance.IndexerFilled()) {
      stopIndexing();
    } else if (FeedPath.mInstance.HopperFilled()) {
      runIndexing();
    } else {
      stopIndexing();
    }
  }

  private void runIndexing() {
    Indexer.mInstance.setV(IntakeCommandConstants.INDEXING_VOLTAGE);
  }

  private void stopIndexing() {
    Indexer.mInstance.stop();
  }

  @Override
  public void end(boolean interrupted) {
    if (Constants.currentMode == Constants.Mode.SIM) {
      FuelSimulation.setIntakeRunning(false);
    }
    roller.setV(0);
    drive.runVelocity(new ChassisSpeeds());
  }

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband on the magnitude so diagonal motion isn't clipped per-axis.
    double linearMagnitude =
        MathUtil.applyDeadband(Math.hypot(x, y), AutoalignIntakeConstants.DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control at low speeds.
    linearMagnitude = linearMagnitude * linearMagnitude;

    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }
}
