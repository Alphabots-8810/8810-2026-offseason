package frc.robot.commands.ShootingCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drum.Drum;
import frc.robot.subsystems.Drum.DrumConstants;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.IntakeDeploy.IntakeDeploy;
import frc.robot.subsystems.IntakeDeploy.IntakeDeployConstants;
import frc.robot.subsystems.IntakeRoller.IntakeRoller;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class Shooting extends Command {
  private enum States {
    AIM,
    SHOOT,
    RETRACT
  }

  private States state;

  // Times the delay between the long-range CANrange reading and retracting the intake.
  private final Timer retractTimer = new Timer();
  private boolean retractTimerStarted;

  // Same gains as DriveCommands.joystickDriveAtAngle, used to rotate the chassis toward the HUB.
  private final PIDController angleController = new PIDController(5.0, 0.0, 0.2);

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
    // angleController.setTolerance(ShootingConstants.AIM_ANGLE_TOLERANCE_RAD);
  }

  @Override
  public void initialize() {
    state = States.AIM;
    retractTimerStarted = false;
    retractTimer.stop();
    retractTimer.reset();
    angleController.reset();
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
    omega = MathUtil.clamp(omega, -15, 15);
    Drive.mInstance.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(0.0, 0.0, omega), Drive.mInstance.getRotation()));
    Logger.recordOutput("Shooging/aimError", angleController.getPositionError());
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
        Math.abs(angleController.getError() - targetHoodRot)
            < ShootingConstants.AIM_ANGLE_TOLERANCE_RAD;
    ;
    Logger.recordOutput("Shooging/aimReady", aimReady);
    Logger.recordOutput("Shooging/hoodReady", hoodReady);
    Logger.recordOutput("Shooging/drumReady", drumReady);
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

  /** Drive the full feed path (intake roller, indexer, feeder) at the tuned shooting speeds. */
  private void runFeed() {
    if (IntakeDeploy.mInstance.getPositionCentimeter() <= 30) {
      IntakeRoller.mInstance.setVelocityRotPerSec(0);
    } else {
      IntakeRoller.mInstance.setVelocityRotPerSec(
          ShootingConstants.IntakeRollerRotpsTunable.getAsDouble());
    }
    Indexer.mInstance.setVelocityRotPerSec(ShootingConstants.IndexerRotpsTunable.getAsDouble());
    Feeder.mInstance.setVelocityRotPerSec(ShootingConstants.FeederRotpsTunable.getAsDouble());
  }

  private boolean canRange() {
    return true;
  }

  private void shoot() {
    // Keep tracking the HUB and holding flywheel/hood while feeding balls.
    prepareShooter();
    aimDrive();
    runFeed();

    // Once a long-range CANrange reading is seen, start the timer and retract after it expires.
    if (canRange() && !retractTimerStarted) {
      retractTimer.restart();
      retractTimerStarted = true;
    }
    if (retractTimerStarted
        && retractTimer.hasElapsed(ShootingConstants.RETRACT_DELAY_SEC.getAsDouble())) {
      state = States.RETRACT;
    }
  }

  private void retract() {
    // Keep shooting while pulling the intake back to the retracted position.
    prepareShooter();
    aimDrive();
    runFeed();
    IntakeDeploy.mInstance.setPositionCentimeter(
        ShootingConstants.INTAKE_RETRACT_POSITION_CM.getAsDouble(), 80, 1000, 0);
  }

  @Override
  public void execute() {
    switch (state) {
      case AIM -> aim();
      case SHOOT -> shoot();
      case RETRACT -> retract();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Drum.mInstance.setVelocityRotPerSec(DrumConstants.StowVelocity);
    ;
    Hood.mInstance.setPositionRot(0);
    ;
    IntakeRoller.mInstance.stop();
    IntakeDeploy.mInstance.setPositionCentimeter(IntakeDeployConstants.IntakeOutPosition);
    Indexer.mInstance.stop();
    Feeder.mInstance.stop();
    Drive.mInstance.stop();
  }
}
