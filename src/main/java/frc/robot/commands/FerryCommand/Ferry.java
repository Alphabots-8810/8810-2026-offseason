package frc.robot.commands.FerryCommand;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldLayout;
import frc.robot.commands.ShootingCommand.ShootingConstants;
import frc.robot.subsystems.Drum.Drum;
import frc.robot.subsystems.Drum.DrumConstants;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.IntakeDeploy.IntakeDeploy;
import frc.robot.subsystems.IntakeDeploy.IntakeDeployConstants;
import frc.robot.subsystems.IntakeRoller.IntakeRoller;
import frc.robot.subsystems.drive.Drive;

public class Ferry extends Command {

  private Pose2d robotpose = new Pose2d();

  private enum ferryStates {
    AIM,
    SHOOT,
    RETRACT
  }

  private enum ferryTargetStates {
    INACTIVE,
    LEFT_NEUTRAL,
    RIGHT_NEUTRAL,
    LEFT_ALLIANCE,
    RIGHT_ALLIANCE,
    LEFT_FAR_ALLIANCE,
    RIGHT_FAR_ALLIANCE
  }

  private final ProfiledPIDController angleController =
      new ProfiledPIDController(5.0, 0.0, 0.4, new TrapezoidProfile.Constraints(8.0, 30.0));

  private boolean is_red_alliance = false;
  private ferryStates f_state = ferryStates.AIM;
  private ferryTargetStates t_state = ferryTargetStates.INACTIVE;
  private Translation2d targetTranslation = new Translation2d();

  private final Timer retractTimer = new Timer();
  private boolean retractTimerStarted = false;

  public Ferry() {
    addRequirements(
        Drum.mInstance,
        Hood.mInstance,
        Feeder.mInstance,
        Indexer.mInstance,
        Drive.mInstance,
        IntakeDeploy.mInstance,
        IntakeRoller.mInstance);
    angleController.enableContinuousInput(-Math.PI, Math.PI);
  }

  private void calculateTarget() {
    robotpose = Drive.mInstance.getPose();
    Translation2d robotXY = robotpose.getTranslation();

    if (FieldLayout.isPoseWithinAllianceZone(is_red_alliance, robotpose)) {
      t_state = ferryTargetStates.INACTIVE;
    } else if (FieldLayout.isPoseOnLeftSide(is_red_alliance, robotpose.getMeasureY())) {
      if (FieldLayout.distanceFromAllianceWall(robotpose.getMeasureX(), is_red_alliance)
          .lte(FieldLayout.FIELD_LENGTH.minus(Units.Feet.of(13)))) {
        t_state = ferryTargetStates.LEFT_ALLIANCE;
        targetTranslation =
            FieldLayout.handleAllianceFlip(FerryConstants.kBlueLeftFerry, is_red_alliance);
      } else if (FieldLayout.distanceFromAllianceWall(robotpose.getMeasureX(), is_red_alliance)
          .lte(FieldLayout.FIELD_LENGTH.minus(Units.Feet.of(6.5)))) {
        t_state = ferryTargetStates.LEFT_FAR_ALLIANCE;
        targetTranslation =
            FieldLayout.handleAllianceFlip(
                FerryConstants.kBlueLeftFarAllianceFerry, is_red_alliance);
      } else {
        t_state = ferryTargetStates.LEFT_NEUTRAL;
        targetTranslation =
            FieldLayout.handleAllianceFlip(FerryConstants.kBlueLeftNeutralFerry, is_red_alliance);
      }
    } else {
      if (FieldLayout.distanceFromAllianceWall(robotpose.getMeasureX(), is_red_alliance)
          .lte(FieldLayout.FIELD_LENGTH.minus(Units.Feet.of(13)))) {
        t_state = ferryTargetStates.RIGHT_ALLIANCE;
        targetTranslation =
            FieldLayout.handleAllianceFlip(FerryConstants.kBlueRightFerry, is_red_alliance);
      } else if (FieldLayout.distanceFromAllianceWall(robotpose.getMeasureX(), is_red_alliance)
          .lte(FieldLayout.FIELD_LENGTH.minus(Units.Feet.of(6.5)))) {
        t_state = ferryTargetStates.RIGHT_FAR_ALLIANCE;
        targetTranslation =
            FieldLayout.handleAllianceFlip(
                FerryConstants.kBlueRightFarAllianceFerry, is_red_alliance);
      } else {
        t_state = ferryTargetStates.RIGHT_NEUTRAL;
        targetTranslation =
            FieldLayout.handleAllianceFlip(FerryConstants.kBlueRightNeutralFerry, is_red_alliance);
      }
    }
  }

  private double getTargetDistance() {
    return robotpose.getTranslation().getDistance(targetTranslation);
  }

  private Rotation2d getTargetAngle() {
    return targetTranslation.minus(robotpose.getTranslation()).getAngle();
  }

  private void aimDrive() {
    double omega =
        angleController.calculate(
            robotpose.getRotation().getRadians(), getTargetAngle().getRadians());
    Drive.mInstance.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(0.0, 0.0, omega), Drive.mInstance.getRotation()));
  }

  private void prepareShooter() {
    double distance = getTargetDistance();
    Drum.mInstance.setVelocityRotPerSec(FerryConstants.ferryDistanceToShooterRotps.get(distance));
    Hood.mInstance.setPositionRot(FerryConstants.ferryDistanceToHoodDeg.get(distance) / 360.0);
  }

  private boolean isReadyToShoot() {
    if (t_state == ferryTargetStates.INACTIVE) return false;

    double distance = getTargetDistance();
    double targetDrumRotps = FerryConstants.ferryDistanceToShooterRotps.get(distance);
    double targetHoodRot = FerryConstants.ferryDistanceToHoodDeg.get(distance) / 360.0;

    boolean drumReady =
        Math.abs(Drum.mInstance.getVelocityRotPerSec() - targetDrumRotps)
            < ShootingConstants.SHOOTER_VELOCITY_TOLERANCE_ROTPS;
    boolean hoodReady =
        Math.abs(Hood.mInstance.getPositionRot() - targetHoodRot)
            < ShootingConstants.HOOD_ANGLE_TOLERANCE_ROT;
    boolean aimReady =
        Math.abs(angleController.getPositionError()) < ShootingConstants.AIM_ANGLE_TOLERANCE_RAD;

    return drumReady && hoodReady && aimReady;
  }

  private void runFeed() {
    IntakeRoller.mInstance.setVelocityRotPerSec(
        ShootingConstants.IntakeRollerRotpsTunable.getAsDouble());
    Indexer.mInstance.setVelocityRotPerSec(ShootingConstants.IndexerRotpsTunable.getAsDouble());
    Feeder.mInstance.setVelocityRotPerSec(ShootingConstants.FeederRotpsTunable.getAsDouble());
  }

  private void aim() {
    if (t_state == ferryTargetStates.INACTIVE) {
      Drive.mInstance.stop();
      return;
    }
    prepareShooter();
    aimDrive();
    IntakeRoller.mInstance.setV(0);
    Indexer.mInstance.setV(0);
    Feeder.mInstance.setV(0);

    if (isReadyToShoot()) {
      f_state = ferryStates.SHOOT;
    }
  }

  private void shoot() {
    if (t_state == ferryTargetStates.INACTIVE) {
      f_state = ferryStates.AIM;
      return;
    }
    prepareShooter();
    aimDrive();
    runFeed();

    if (!retractTimerStarted) {
      retractTimer.restart();
      retractTimerStarted = true;
    }
    if (retractTimer.hasElapsed(ShootingConstants.RETRACT_DELAY_SEC.getAsDouble())) {
      f_state = ferryStates.RETRACT;
    }
  }

  private void retract() {
    prepareShooter();
    aimDrive();
    runFeed();
    IntakeDeploy.mInstance.setPositionCentimeter(
        ShootingConstants.INTAKE_RETRACT_POSITION_CM.getAsDouble(), 50, 1000, 0);
  }

  @Override
  public void initialize() {
    is_red_alliance =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    f_state = ferryStates.AIM;
    retractTimerStarted = false;
    retractTimer.stop();
    retractTimer.reset();
    angleController.reset(Drive.mInstance.getRotation().getRadians());
  }

  @Override
  public void execute() {
    calculateTarget();
    switch (f_state) {
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
