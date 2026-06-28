package frc.robot.commands.ShootingCommand;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drum.Drum;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.IntakeDeploy.IntakeDeploy;
import frc.robot.subsystems.IntakeRoller.IntakeRoller;
import frc.robot.subsystems.drive.Drive;

public class Shooting extends Command {
  private enum States {
    AIM,
    SHOOT
  }

  private States state;

  // Same gains as DriveCommands.joystickDriveAtAngle, used to rotate the chassis toward the HUB.
  private final ProfiledPIDController angleController =
      new ProfiledPIDController(5.0, 0.0, 0.4, new TrapezoidProfile.Constraints(8.0, 30.0));

  public Shooting() {
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

  /** The HUB translation for the current alliance. */
  private Translation2d hubLocation() {
    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    return isRed
        ? Constants.FieldConstants.RED_HUB_LOCATION
        : Constants.FieldConstants.BLUE_HUB_LOCATION;
  }

  /** Horizontal distance from the robot to the HUB, in meters. */
  private double distanceToHub() {
    return Drive.mInstance.getPose().getTranslation().getDistance(hubLocation());
  }

  /** Field-relative heading that points the robot at the HUB. */
  private Rotation2d angleToHub() {
    return hubLocation().minus(Drive.mInstance.getPose().getTranslation()).getAngle();
  }

  /** Spin the chassis in place to face the HUB. */
  private void aimDrive() {
    double omega =
        angleController.calculate(
            Drive.mInstance.getRotation().getRadians(), angleToHub().getRadians());
    Drive.mInstance.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(0.0, 0.0, omega), Drive.mInstance.getRotation()));
  }

  /** Drive the flywheel and hood to the interpolated setpoints for the current distance. */
  private void prepareShooter() {
    double distance = distanceToHub();
    Drum.mInstance.setVelocityRotPerSec(ShootingConstants.distanceToShooterRotps.get(distance));
    Hood.mInstance.setPositionRot(ShootingConstants.distanceToHoodDeg.get(distance) / 360.);
  }

  /** True once the flywheel, hood, and heading are all within tolerance. */
  private boolean isReadyToShoot() {
    double distance = distanceToHub();
    double targetShooterRotps = ShootingConstants.distanceToShooterRotps.get(distance);
    double targetHoodRot = ShootingConstants.distanceToHoodDeg.get(distance) / 360.;

    boolean drumReady =
        Math.abs(Drum.mInstance.getVelocityRotPerSec() - targetShooterRotps)
            < ShootingConstants.SHOOTER_VELOCITY_TOLERANCE_ROTPS;
    boolean hoodReady =
        Math.abs(Hood.mInstance.getPositionRot() - targetHoodRot)
            < ShootingConstants.HOOD_ANGLE_TOLERANCE_ROT;
    boolean aimReady =
        Math.abs(angleController.getPositionError()) < ShootingConstants.AIM_ANGLE_TOLERANCE_RAD;

    return drumReady && hoodReady && aimReady;
  }

  private void aim() {
    prepareShooter();
    aimDrive();
    // Keep the feed path stopped until everything is on target.
    IntakeRoller.mInstance.setV(0);
    Indexer.mInstance.setV(0);
    Feeder.mInstance.setV(0);

    if (isReadyToShoot()) {
      state = States.SHOOT;
    }
  }

  private void shoot() {
    // Keep tracking the HUB and holding flywheel/hood while feeding balls.
    prepareShooter();
    aimDrive();
    IntakeRoller.mInstance.setVelocityRotPerSec(
        ShootingConstants.IntakeRollerRotpsTunable.getAsDouble());
    Indexer.mInstance.setVelocityRotPerSec(ShootingConstants.IndexerRotpsTunable.getAsDouble());
    Feeder.mInstance.setVelocityRotPerSec(ShootingConstants.FeederRotpsTunable.getAsDouble());
  }

  @Override
  public void execute() {
    switch (state) {
      case AIM -> aim();
      case SHOOT -> shoot();
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
    Drive.mInstance.stop();
  }
}
