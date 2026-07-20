package frc.robot.commands.ShootingCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
import frc.robot.subsystems.FeedPath.FeedPath;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.IntakeDeploy.IntakeDeploy;
import frc.robot.subsystems.IntakeDeploy.IntakeDeployConstants;
import frc.robot.subsystems.IntakeRoller.IntakeRoller;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Shooting extends Command {
  private enum States {
    AIM,
    SHOOT,
    RETRACT,
    UNJAM
  }

  private States state;

  // Times the delay between the long-range CANrange reading and retracting the intake.
  private final Timer retractTimer = new Timer();
  private boolean retractTimerStarted;
  private boolean isEnergySave;

  // Indexer jam detection: stator current must stay above the threshold for the full debounce
  // time before UNJAM triggers (recreated whenever detection restarts, see isIndexerJammed()).
  private Debouncer jamDebouncer;
  // Times the fixed backwards-run of the indexer in UNJAM.
  private final Timer unjamTimer = new Timer();
  // The feeding state (SHOOT or RETRACT) to resume once the unjam back-off finishes.
  private States stateAfterUnjam = States.SHOOT;

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

  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;

  public Shooting(boolean isEnergySave, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
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
    this.isEnergySave = isEnergySave;
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
    resetJamDetection();
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

  /** Field-relative linear velocity from the left stick (unitless, squared magnitude). */
  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    double linearMagnitude =
        MathUtil.applyDeadband(Math.hypot(x, y), ShootingConstants.DRIVE_DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));
    linearMagnitude = linearMagnitude * linearMagnitude;
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  /** True when the driver is commanding field-relative translation on the left stick. */
  private boolean isDriverCommandingTranslation() {
    return getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble())
            .getNorm()
        > 1e-6;
  }

  /** True when measured chassis linear speed is below the stop threshold. */
  private boolean isChassisTranslationStopped() {
    ChassisSpeeds speeds = Drive.mInstance.getChassisSpeeds();
    return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
        < ShootingConstants.CHASSIS_STOP_VELOCITY_MPS;
  }

  /** Shooting only starts once the driver releases the sticks and the chassis settles. */
  private boolean isChassisStopped() {
    return !isDriverCommandingTranslation() && isChassisTranslationStopped();
  }

  /** Drive toward the HUB heading while allowing left-stick field-relative translation. */
  private void aimDrive() {
    Translation2d linearVelocity =
        getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

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

    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * Drive.mInstance.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * Drive.mInstance.getMaxLinearSpeedMetersPerSec(),
            omega);
    Drive.mInstance.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            isFlipped
                ? Drive.mInstance.getRotation().plus(Rotation2d.kPi)
                : Drive.mInstance.getRotation()));

    Logger.recordOutput("Shooting/aimAngleRad", currentAngleRad);
    Logger.recordOutput("Shooting/aimSetpointRad", targetAngleRad);
    Logger.recordOutput("Shooting/aimProfilePositionRad", angleController.getSetpoint().position);
    Logger.recordOutput("Shooting/aimProfileVelRadPerSec", angleController.getSetpoint().velocity);
    Logger.recordOutput("Shooting/aimError", goalErrorRad());
    Logger.recordOutput("Shooting/aimOmegaRadPerSec", omega);
    Logger.recordOutput("Shooting/driverCommandingTranslation", isDriverCommandingTranslation());
    Logger.recordOutput("Shooting/chassisTranslationStopped", isChassisTranslationStopped());
    Logger.recordOutput("Shooting/chassisStopped", isChassisStopped());
  }

  /** The drum setpoint for a distance: sim-derived table times the distance-dependent kSpeed. */
  private double drumSetpointRotps(double distance) {
    return !isEnergySave
        ? (ShootingConstants.distanceToShooterRotps.get(distance)
            * ShootingConstants.kSpeed.getAsDouble())
        : (ShootingConstants.distanceToShooterRotps.get(distance)
            * ShootingConstants.kSpeed.getAsDouble()
            * 1.008);
  }

  /** Drive the flywheel and hood to the interpolated sektpoints for the current distance. */
  private void prepareShooter() {
    double distance = distanceToHub();
    Drum.mInstance.setVelocityRotPerSec(drumSetpointRotps(distance));
    Hood.mInstance.setPositionRot(ShootingConstants.distanceToHoodDeg.get(distance) / 360.);
    Logger.recordOutput("Shooting/drumSetpointRotps", drumSetpointRotps(distance));
    Logger.recordOutput("Shooting/simDrumRotps", ShootingConstants.simDrumRotps(distance));
    // Sim-predicted hood angle (mechanism deg) for the current distance, next to the actual
    // table setpoint so field edits to the table are visible against the model.
    Logger.recordOutput("Shooting/simHoodDeg", ShootingConstants.simHoodDeg(distance));
    Logger.recordOutput(
        "Shooting/hoodSetpointDeg", ShootingConstants.distanceToHoodDeg.get(distance));
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
    boolean chassisStopped = isChassisStopped();
    Logger.recordOutput("Shooting/aimReady", aimReady);
    Logger.recordOutput("Shooting/hoodReady", hoodReady);
    Logger.recordOutput("Shooting/drumReady", drumReady);
    Logger.recordOutput("Shooting/chassisReady", chassisStopped);
    return drumReady && hoodReady && aimReady && chassisStopped;
  }

  /**
   * Restart jam detection from a clean baseline. The debouncer must be recreated whenever detection
   * pauses (AIM, UNJAM): a stale one that last saw "jammed" would re-trip instantly on the first
   * sample instead of requiring the full debounce time again.
   */
  private void resetJamDetection() {
    jamDebouncer = new Debouncer(ShootingConstants.INDEXER_JAM_DEBOUNCE_SEC, DebounceType.kRising);
  }

  /** True when the indexer stator current has been above the jam threshold for the debounce. */
  private boolean isIndexerJammed() {
    boolean jammed =
        jamDebouncer.calculate(
            Indexer.mInstance.getStatorCurrentAmps() > ShootingConstants.INDEXER_JAM_CURRENT_AMPS);
    Logger.recordOutput("Shooting/indexerJammed", jammed);
    return jammed;
  }

  /**
   * SHOOT/RETRACT -> UNJAM when the indexer stalls. Returns true when the transition fired;
   * remembers the interrupted state so the volley resumes exactly where it left off.
   */
  private boolean checkJamAndStartUnjam(States resumeState) {
    if (!isIndexerJammed()) {
      return false;
    }
    stateAfterUnjam = resumeState;
    state = States.UNJAM;
    unjamTimer.restart();
    return true;
  }

  /** Back the stalled fuel out of the indexer, then resume the interrupted feeding state. */
  private void unjam() {
    if (shouldReturnToAim()) {
      returnToAim();
      return;
    }

    // Keep holding the flywheel/hood and X-lock the drivetrain so shooting resumes immediately
    // after the back-off without allowing the robot to be pushed off target.
    prepareShooter();
    aimDrive();
    IntakeRoller.mInstance.setV(0);
    Feeder.mInstance.setV(-ShootingConstants.UNJAM_Feeder_Current.getAsDouble());
    Indexer.mInstance.setVelocityRotPerSec(-ShootingConstants.UNJAM_INDEXER_Current.getAsDouble());
    IntakeDeploy.mInstance.setPositionCentimeter(55);

    if (unjamTimer.hasElapsed(ShootingConstants.UNJAM_DURATION_SEC.getAsDouble())) {
      resetJamDetection();
      state = stateAfterUnjam;
    }
  }

  /** Stop feeding and return to AIM after the driver moves or the chassis drifts during SHOOT. */
  private void returnToAim() {
    state = States.AIM;
    retractTimerStarted = false;
    retractTimer.stop();
    retractTimer.reset();
    readyDebouncer = new Debouncer(ShootingConstants.READY_DEBOUNCE_SEC, DebounceType.kRising);
    resetJamDetection();
    angleController.reset(
        Drive.mInstance.getRotation().getRadians(),
        Drive.mInstance.getChassisSpeeds().omegaRadiansPerSecond);
    IntakeRoller.mInstance.setV(0);
    Indexer.mInstance.setV(0);
    Feeder.mInstance.setV(0);
  }

  /** True when SHOOT/RETRACT should abort back to AIM because the chassis is moving. */
  private boolean shouldReturnToAim() {
    return !isChassisStopped();
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
    if (!isEnergySave) {
      if (IntakeDeploy.mInstance.getPositionCentimeter() <= 30) {
        IntakeRoller.mInstance.setVelocityRotPerSec(0);
      } else {
        IntakeRoller.mInstance.setVelocityRotPerSec(
            ShootingConstants.IntakeRollerRotpsTunable.getAsDouble());
      }
      Indexer.mInstance.setVelocityRotPerSec(ShootingConstants.IndexerRotpsTunable.getAsDouble());
      Feeder.mInstance.setVelocityRotPerSec(ShootingConstants.FeederRotpsTunable.getAsDouble());
    } else {
      if (IntakeDeploy.mInstance.getPositionCentimeter() <= 30) {
        IntakeRoller.mInstance.setVelocityRotPerSec(0);
      } else {
        IntakeRoller.mInstance.setVelocityRotPerSec(
            ShootingConstants.IntakeRollerRotpsTunable.getAsDouble());
      }
      // Indexer.mInstance。;
      Indexer.mInstance.setCurrent(50);
      ;
      Feeder.mInstance.setCurrent(50);
      ;
    }
  }

  private boolean canRange() {
    return !FeedPath.mInstance.HopperFilled();
  }

  private void shoot() {
    if (shouldReturnToAim()) {
      returnToAim();
      return;
    }
    if (checkJamAndStartUnjam(States.SHOOT)) {
      return;
    }

    // Lock the wheels once feeding begins so the robot cannot be pushed off its aimed heading.
    prepareShooter();
    aimDrive();
    runFeed();

    // Once a long-range CANrange reading is seen, start the timer and retract after it expires.
    if (!retractTimerStarted) {
      retractTimer.restart();
      retractTimerStarted = true;
    }
    if (retractTimerStarted
        && retractTimer.hasElapsed(ShootingConstants.RETRACT_DELAY_SEC.getAsDouble())) {
      state = States.RETRACT;
    }
  }

  private void retract() {
    if (shouldReturnToAim()) {
      returnToAim();
      return;
    }
    if (checkJamAndStartUnjam(States.RETRACT)) {
      return;
    }

    // Keep shooting and X-locking the drivetrain while pulling the intake back.
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
      case UNJAM -> unjam();
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
