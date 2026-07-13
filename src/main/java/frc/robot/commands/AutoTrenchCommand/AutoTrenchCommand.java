package frc.robot.commands.AutoTrenchCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class AutoTrenchCommand extends Command {
  private enum State {
    ALIGN,
    PASS
  }

  private final Drive drive;
  private final DoubleSupplier xSupplier;
  private final PIDController yController =
      new PIDController(
          AutoTrenchCommandConstants.Y_KP,
          AutoTrenchCommandConstants.Y_KI,
          AutoTrenchCommandConstants.Y_KD);
  private final ProfiledPIDController angleController =
      new ProfiledPIDController(
          AutoTrenchCommandConstants.ANGLE_KP,
          0.0,
          AutoTrenchCommandConstants.ANGLE_KD,
          new TrapezoidProfile.Constraints(
              AutoTrenchCommandConstants.ANGLE_MAX_VELOCITY_RAD_PER_SEC,
              AutoTrenchCommandConstants.ANGLE_MAX_ACCELERATION_RAD_PER_SEC_SQ));

  private State state = State.ALIGN;
  private Rotation2d targetHeading = Rotation2d.kZero;
  private double targetX = AutoTrenchCommandConstants.BLUE_TRENCH_X_METERS;
  private double targetY = AutoTrenchCommandConstants.DOWN_TRENCH_Y_METERS;

  public AutoTrenchCommand(DoubleSupplier xSupplier) {
    this.drive = Drive.mInstance;
    this.xSupplier = xSupplier;
    yController.setTolerance(AutoTrenchCommandConstants.Y_TOLERANCE_METERS);
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    state = State.ALIGN;
    yController.reset();
    angleController.reset(drive.getRotation().getRadians());
    targetX = getClosestTrenchX(drive.getPose().getX());
    updateTargets();
  }

  @Override
  public void execute() {
    updateTargets();
    switch (state) {
      case ALIGN -> align();
      case PASS -> pass();
    }
  }

  /**
   * Retargets to the closest trench every cycle, so holding the button while driving across the
   * field always aims at the trench actually being approached. If the closest trench changes while
   * in PASS (crossed the field midline toward the other trench), drop back to ALIGN so the approach
   * slowdown and alignment gate run again for the new trench.
   */
  private void updateTargets() {
    Pose2d pose = drive.getPose();
    double closestX = getClosestTrenchX(pose.getX());
    if (state == State.PASS && closestX != targetX) {
      state = State.ALIGN;
      yController.reset();
    }
    targetX = closestX;
    if (state == State.ALIGN) {
      targetY = getClosestTrenchY(pose.getY());
      targetHeading = getClosestZeroOrPiHeading(drive.getRotation());
      yController.setSetpoint(targetY);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  private void align() {
    Pose2d pose = drive.getPose();
    double joystickXSpeed = calculateJoystickXSpeed();
    double xSpeed = calculateAlignXSpeed(pose.getX(), joystickXSpeed);
    double ySpeed = calculateYSpeed(pose.getY());
    double omega = calculateHeadingSpeed();

    logOutputs(joystickXSpeed, xSpeed, ySpeed, omega);
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(xSpeed, ySpeed, omega), drive.getRotation()));

    if (yController.atSetpoint() && angleController.atSetpoint()) {
      state = State.PASS;
    }
  }

  private void pass() {
    Pose2d pose = drive.getPose();
    double joystickXSpeed = calculateJoystickXSpeed();
    double ySpeed = calculateYSpeed(pose.getY());
    double omega = calculateHeadingSpeed();

    logOutputs(joystickXSpeed, joystickXSpeed, ySpeed, omega);
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(joystickXSpeed, ySpeed, omega), drive.getRotation()));
  }

  private double calculateYSpeed(double robotY) {
    return MathUtil.clamp(
        yController.calculate(robotY),
        -AutoTrenchCommandConstants.MAX_Y_SPEED_METERS_PER_SEC,
        AutoTrenchCommandConstants.MAX_Y_SPEED_METERS_PER_SEC);
  }

  private double calculateJoystickXSpeed() {
    double xInput =
        MathUtil.applyDeadband(
            xSupplier.getAsDouble(), AutoTrenchCommandConstants.JOYSTICK_DEADBAND);
    return xInput * drive.getMaxLinearSpeedMetersPerSec();
  }

  private double calculateHeadingSpeed() {
    return MathUtil.clamp(
        angleController.calculate(drive.getRotation().getRadians(), targetHeading.getRadians()),
        -AutoTrenchCommandConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC,
        AutoTrenchCommandConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC);
  }

  private double calculateAlignXSpeed(double robotX, double joystickXSpeed) {
    // Measure to the near entry face of the 1.2 m structure, not its centerline; negative when
    // already inside the trench footprint.
    double distanceToEntryFace =
        Math.abs(targetX - robotX) - AutoTrenchCommandConstants.TRENCH_HALF_LENGTH_METERS;
    if (distanceToEntryFace <= AutoTrenchCommandConstants.X_STOP_DISTANCE_METERS) {
      return 0.0;
    }

    double slowdownScale =
        MathUtil.clamp(
            (distanceToEntryFace - AutoTrenchCommandConstants.X_STOP_DISTANCE_METERS)
                / AutoTrenchCommandConstants.X_SLOWDOWN_RANGE_METERS,
            0.0,
            1.0);

    return joystickXSpeed * slowdownScale;
  }

  private void logOutputs(double joystickXSpeed, double xSpeed, double ySpeed, double omega) {
    Logger.recordOutput("AutoTrench/State", state.toString());
    Logger.recordOutput("AutoTrench/TargetX", targetX);
    Logger.recordOutput("AutoTrench/TargetY", targetY);
    Logger.recordOutput("AutoTrench/TargetHeadingDeg", targetHeading.getDegrees());
    Logger.recordOutput("AutoTrench/YError", yController.getError());
    Logger.recordOutput("AutoTrench/HeadingErrorRad", angleController.getPositionError());
    Logger.recordOutput("AutoTrench/JoystickXSpeed", joystickXSpeed);
    Logger.recordOutput("AutoTrench/XSpeed", xSpeed);
    Logger.recordOutput("AutoTrench/YSpeed", ySpeed);
    Logger.recordOutput("AutoTrench/Omega", omega);
  }

  private static Rotation2d getClosestZeroOrPiHeading(Rotation2d heading) {
    if (heading.getCos() >= 0.0) {
      return Rotation2d.kZero;
    }
    return Rotation2d.kPi;
  }

  private static double getClosestTrenchX(double robotX) {
    double blueX = AutoTrenchCommandConstants.BLUE_TRENCH_X_METERS;
    double redX = AutoTrenchCommandConstants.RED_TRENCH_X_METERS;
    return Math.abs(robotX - blueX) <= Math.abs(robotX - redX) ? blueX : redX;
  }

  private static double getClosestTrenchY(double robotY) {
    if (robotY >= AutoTrenchCommandConstants.FIELD_MID_Y_METERS) {
      return AutoTrenchCommandConstants.UP_TRENCH_Y_METERS;
    }
    return AutoTrenchCommandConstants.DOWN_TRENCH_Y_METERS;
  }
}
