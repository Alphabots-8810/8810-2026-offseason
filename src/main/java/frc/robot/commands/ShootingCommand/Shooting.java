package frc.robot.commands.ShootingCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

  // Profiled heading controller: the trapezoid profile plans the turn (velocity is fed
  // forward), the PID only corrects tracking error against the moving profile setpoint.
  private final ProfiledPIDController angleController =
      new ProfiledPIDController(
          ShootingConstants.AIM_KP,
          0.0,
          ShootingConstants.AIM_KD,
          new TrapezoidProfile.Constraints(
              ShootingConstants.AIM_MAX_VELOCITY_RAD_PER_SEC,
              ShootingConstants.AIM_MAX_ACCELERATION_RAD_PER_SEC2));

  // Requires isReadyToShoot() to hold continuously before AIM -> SHOOT (recreated in initialize).
  private Debouncer readyDebouncer;

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
    // Seed the profile with the current heading and yaw rate so the plan starts from the
    // robot's actual state instead of jerking from zero.
    angleController.reset(
        Drive.mInstance.getRotation().getRadians(),
        Drive.mInstance.getChassisSpeeds().omegaRadiansPerSecond);
    readyDebouncer = new Debouncer(ShootingConstants.READY_DEBOUNCE_SEC, DebounceType.kRising);
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

  /** Wrapped heading error between the robot and the HUB goal (NOT the profile setpoint). */
  private double goalErrorRad() {
    return MathUtil.angleModulus(
        angleToHub().getRadians() - Drive.mInstance.getRotation().getRadians());
  }

  /** Spin the chassis in place to face the HUB. */
  private void aimDrive() {
    double currentAngleRad = Drive.mInstance.getRotation().getRadians();
    double targetAngleRad = angleToHub().getRadians();
    // PID correction against the moving profile setpoint, plus the profile's planned
    // velocity as feedforward so the turn actually runs at the planned speed.
    double omega =
        angleController.calculate(currentAngleRad, targetAngleRad)
            + angleController.getSetpoint().velocity;
    omega = MathUtil.clamp(omega, -15, 15);
    // Kill the stiction limit-cycle: tiny errors command omegas the drivetrain cannot
    // execute, so inside the deadband hold still instead of wiggling.
    if (Math.abs(goalErrorRad()) < ShootingConstants.AIM_ERROR_DEADBAND_RAD) {
      omega = 0.0;
    }
    Drive.mInstance.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(0.0, 0.0, omega), Drive.mInstance.getRotation()));
    Logger.recordOutput("Shooging/aimAngleRad", currentAngleRad);
    Logger.recordOutput("Shooging/aimSetpointRad", targetAngleRad);
    Logger.recordOutput("Shooging/aimProfilePositionRad", angleController.getSetpoint().position);
    Logger.recordOutput("Shooging/aimProfileVelRadPerSec", angleController.getSetpoint().velocity);
    Logger.recordOutput("Shooging/aimError", goalErrorRad());
    Logger.recordOutput("Shooging/aimOmegaRadPerSec", omega);
  }

  /** The drum setpoint for a distance: sim-derived table times the kSpeed field knob. */
  private double drumSetpointRotps(double distance) {
    return ShootingConstants.distanceToShooterRotps.get(distance)
        * ShootingConstants.kSpeedTunable.getAsDouble();
  }

  /** Drive the flywheel and hood to the interpolated setpoints for the current distance. */
  private void prepareShooter() {
    double distance = distanceToHub();
    Drum.mInstance.setVelocityRotPerSec(drumSetpointRotps(distance));
    Hood.mInstance.setPositionRot(ShootingConstants.distanceToHoodDeg.get(distance) / 360.);
    Logger.recordOutput("Shooging/drumSetpointRotps", drumSetpointRotps(distance));
    Logger.recordOutput("Shooging/simDrumRotps", ShootingConstants.simDrumRotps(distance));
    // Sim-predicted hood angle (mechanism deg) for the current distance, next to the actual
    // table setpoint so field edits to the table are visible against the model.
    Logger.recordOutput("Shooging/simHoodDeg", ShootingConstants.simHoodDeg(distance));
    Logger.recordOutput(
        "Shooging/hoodSetpointDeg", ShootingConstants.distanceToHoodDeg.get(distance));
  }

  /** True once the flywheel, hood, and heading are all within tolerance. */
  private boolean isReadyToShoot() {
    double distance = distanceToHub();
    double targetShooterRotps = drumSetpointRotps(distance);
    double targetHoodRot = ShootingConstants.distanceToHoodDeg.get(distance) / 360.;

    boolean drumReady =
        Math.abs(Drum.mInstance.getVelocityRotPerSec() - targetShooterRotps)
            < ShootingConstants.SHOOTER_VELOCITY_TOLERANCE_ROTPS;
    boolean hoodReady =
        Math.abs(Hood.mInstance.getPositionRot() - targetHoodRot)
            < ShootingConstants.HOOD_ANGLE_TOLERANCE_ROT;
    // Compare against the HUB goal, not angleController's error: the profiled controller's
    // error is measured against the moving profile setpoint and is near zero mid-turn.
    boolean aimReady = Math.abs(goalErrorRad()) < ShootingConstants.AIM_ANGLE_TOLERANCE_RAD;
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

    if (readyDebouncer.calculate(isReadyToShoot())) {
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
    Hood.mInstance.setPositionRot(0);
    IntakeRoller.mInstance.stop();
    IntakeDeploy.mInstance.setPositionCentimeter(IntakeDeployConstants.IntakeOutPosition);
    Indexer.mInstance.stop();
    Feeder.mInstance.stop();
    Drive.mInstance.stop();
  }
}
