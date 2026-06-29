package frc.robot.commands.MotionCompensateShootingCommand;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.ShootingCommand.ShootingConstants;
import frc.robot.subsystems.Drum.Drum;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.IntakeDeploy.IntakeDeploy;
import frc.robot.subsystems.IntakeRoller.IntakeRoller;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class MotionCompensateShooting extends Command {
  private enum States {
    AIM,
    SHOOT
  }

  private States state;

  private static final double DEADBAND = 0.1;

  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;

  private final ProfiledPIDController angleController =
      new ProfiledPIDController(5.0, 0.0, 0.4, new TrapezoidProfile.Constraints(8.0, 30.0));

  public MotionCompensateShooting() {
    this(() -> 0.0, () -> 0.0);
  }

  public MotionCompensateShooting(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    addRequirements(
        Drive.mInstance,
        Drum.mInstance,
        Feeder.mInstance,
        Hood.mInstance,
        Indexer.mInstance,
        IntakeDeploy.mInstance,
        IntakeRoller.mInstance);
    angleController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    state = States.AIM;
    angleController.reset(Drive.mInstance.getRotation().getRadians());
  }

  private Translation2d hubLocation() {
    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    return isRed
        ? Constants.FieldConstants.RED_HUB_LOCATION
        : Constants.FieldConstants.BLUE_HUB_LOCATION;
  }

  private Translation2d robotTranslation() {
    return Drive.mInstance.getPose().getTranslation();
  }

  private double distanceToHub() {
    return robotTranslation().getDistance(hubLocation());
  }

  private double targetShooterRotps() {
    return ShootingConstants.distanceToShooterRotps.get(distanceToHub());
  }

  private double targetHoodRot() {
    return ShootingConstants.distanceToHoodDeg.get(distanceToHub()) / 360.0;
  }

  private Translation2d fieldRelativeRobotVelocity() {
    ChassisSpeeds speeds = Drive.mInstance.getChassisSpeeds();
    Rotation2d rotation = Drive.mInstance.getRotation();
    double cos = rotation.getCos();
    double sin = rotation.getSin();

    return new Translation2d(
        speeds.vxMetersPerSecond * cos - speeds.vyMetersPerSecond * sin,
        speeds.vxMetersPerSecond * sin + speeds.vyMetersPerSecond * cos);
  }

  private double estimatedFlightTimeSec() {
    return MotionCompConstants.getFlightTimeSec(distanceToHub());
  }

  private Translation2d compensatedHubLocation() {
    Translation2d velocityCompensation =
        fieldRelativeRobotVelocity()
            .times(
                estimatedFlightTimeSec()
                    * MotionCompConstants.CompensationScaleTunable.getAsDouble());
    return hubLocation().minus(velocityCompensation);
  }

  private Rotation2d compensatedAngleToHub() {
    return compensatedHubLocation().minus(robotTranslation()).getAngle();
  }

  private Translation2d getLinearVelocityFromJoysticks() {
    double x = xSupplier.getAsDouble();
    double y = ySupplier.getAsDouble();
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    linearMagnitude = linearMagnitude * linearMagnitude;

    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  private Translation2d limitLinearVelocity(Translation2d linearVelocityMetersPerSec) {
    double speed = linearVelocityMetersPerSec.getNorm();
    double maxSpeed = Math.max(0.0, MotionCompConstants.MaxShootWhileMovingSpeedMpsTunable);
    if (speed <= maxSpeed || speed < 1e-6) {
      return linearVelocityMetersPerSec;
    }

    return linearVelocityMetersPerSec.times(maxSpeed / speed);
  }

  private void aimDrive() {
    double omega =
        angleController.calculate(
            Drive.mInstance.getRotation().getRadians(), compensatedAngleToHub().getRadians());
    Translation2d linearVelocity = getLinearVelocityFromJoysticks();
    Translation2d limitedLinearVelocity =
        limitLinearVelocity(
            linearVelocity.times(Drive.mInstance.getMaxLinearSpeedMetersPerSec()));
    ChassisSpeeds fieldRelativeSpeeds =
        new ChassisSpeeds(limitedLinearVelocity.getX(), limitedLinearVelocity.getY(), omega);
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    Drive.mInstance.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldRelativeSpeeds,
            isFlipped
                ? Drive.mInstance.getRotation().plus(Rotation2d.kPi)
                : Drive.mInstance.getRotation()));
  }

  private void prepareShooter() {
    Drum.mInstance.setVelocityRotPerSec(targetShooterRotps());
    Hood.mInstance.setPositionRot(targetHoodRot());
  }

  private boolean isReadyToShoot() {
    boolean drumReady =
        Math.abs(Drum.mInstance.getVelocityRotPerSec() - targetShooterRotps())
            < ShootingConstants.SHOOTER_VELOCITY_TOLERANCE_ROTPS;
    boolean hoodReady =
        Math.abs(Hood.mInstance.getPositionRot() - targetHoodRot())
            < ShootingConstants.HOOD_ANGLE_TOLERANCE_ROT;
    boolean aimReady =
        Math.abs(angleController.getPositionError()) < ShootingConstants.AIM_ANGLE_TOLERANCE_RAD;

    return drumReady && hoodReady && aimReady;
  }

  private void aim() {
    prepareShooter();
    aimDrive();
    IntakeRoller.mInstance.setV(0);
    Indexer.mInstance.setV(0);
    Feeder.mInstance.setV(0);

    if (isReadyToShoot()) {
      state = States.SHOOT;
    }
  }

  private void shoot() {
    prepareShooter();
    aimDrive();
    IntakeRoller.mInstance.setVelocityRotPerSec(
        ShootingConstants.IntakeRollerRotpsTunable.getAsDouble());
    Indexer.mInstance.setVelocityRotPerSec(ShootingConstants.IndexerRotpsTunable.getAsDouble());
    Feeder.mInstance.setVelocityRotPerSec(ShootingConstants.FeederRotpsTunable.getAsDouble());
  }

  private void log() {
    Translation2d compensatedTarget = compensatedHubLocation();
    Logger.recordOutput("Shooting/MotionComp/FlightTimeSec", estimatedFlightTimeSec());
    Logger.recordOutput("Shooting/MotionComp/Target", compensatedTarget);
    Logger.recordOutput("Shooting/MotionComp/TargetAngle", compensatedAngleToHub());
    Logger.recordOutput(
        "Shooting/MotionComp/MaxShootWhileMovingSpeedMps",
        MotionCompConstants.MaxShootWhileMovingSpeedMpsTunable);

    SmartDashboard.putBoolean("MotionCompAligning", state == States.AIM);
    SmartDashboard.putBoolean("MotionCompShooting", state == States.SHOOT);
    SmartDashboard.putNumber("MotionCompFlightTimeSec", estimatedFlightTimeSec());
  }

  @Override
  public void execute() {
    switch (state) {
      case AIM -> aim();
      case SHOOT -> shoot();
    }

    log();
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
    Drive.mInstance.stop();
  }
}
