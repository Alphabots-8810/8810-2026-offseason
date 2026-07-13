package frc.robot.commands.AutopilotTrenchCommand;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutoTrenchCommand.AutoTrenchCommandConstants;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

/**
 * Autonomous trench pass built on the Autopilot library, in two stages.
 *
 * <p>APPROACH arcs smoothly onto the trench centerline using Autopilot's entry-angle funnel, aimed
 * at a point just outside the entrance. PASS then drives straight through to the goal: the next
 * Choreo path's starting pose, reached at that path's starting speed so the path takes over without
 * braking, or a point just past the exit when there is no next path.
 *
 * <p>When the next path starts <i>before</i> the entrance (the path drives the trench itself), the
 * arc still aims at the entrance — the path start lies on the centerline, so the entrance
 * approach's tangent line passes through it — and the command finishes as soon as the robot crosses
 * the path-start plane aligned with the trench. Aiming at the path start directly would make the
 * arc whip around it instead of joining the centerline tangentially.
 *
 * <p>Heading is fixed for the whole command: the back of the robot faces the direction of travel.
 */
public class AutopilotTrenchCommand extends Command {
  private enum Stage {
    APPROACH,
    PASS
  }

  private static final Autopilot autopilot =
      new Autopilot(
          new APProfile(
                  new APConstraints()
                      .withVelocity(AutopilotTrenchCommandConstants.PASS_SPEED_METERS_PER_SEC)
                      .withAcceleration(
                          AutopilotTrenchCommandConstants.MAX_ACCELERATION_METERS_PER_SEC_SQ)
                      .withJerk(AutopilotTrenchCommandConstants.MAX_JERK_METERS_PER_SEC_CUBED))
              .withErrorXY(Meters.of(AutopilotTrenchCommandConstants.ERROR_XY_METERS))
              .withErrorTheta(Degrees.of(AutopilotTrenchCommandConstants.ERROR_THETA_DEGREES))
              .withBeelineRadius(Meters.of(AutopilotTrenchCommandConstants.BEELINE_RADIUS_METERS)));

  private final Drive drive;
  private final Pose2d nextPathStartPose;
  /**
   * Speed carried across the handoff into the next path, capped at the pass speed. Zero when there
   * is no next path or it starts at rest — the command then settles at the goal instead.
   */
  private final double handoffVelocityMetersPerSec;

  private final ProfiledPIDController angleController =
      new ProfiledPIDController(
          AutopilotTrenchCommandConstants.ANGLE_KP,
          0.0,
          AutopilotTrenchCommandConstants.ANGLE_KD,
          new TrapezoidProfile.Constraints(
              AutopilotTrenchCommandConstants.ANGLE_MAX_VELOCITY_RAD_PER_SEC,
              AutopilotTrenchCommandConstants.ANGLE_MAX_ACCELERATION_RAD_PER_SEC_SQ));

  private Stage stage = Stage.APPROACH;
  private APTarget entranceTarget = new APTarget(Pose2d.kZero);
  private APTarget goalTarget = new APTarget(Pose2d.kZero);
  // True when the next path starts before the trench entrance and drives the trench itself: the
  // command then never switches to the goal target and finishes at the path-start plane instead.
  private boolean nearSideGoal = false;
  private double passTransitionX = 0.0;
  private double trenchY = 0.0;
  private Rotation2d passHeading = Rotation2d.kZero;
  private boolean travelingPositiveX = true;
  // Autopilot ramps its output from the speeds we feed it. Feeding measured speeds would throttle
  // that ramp to the drivetrain's tracking lag, so we feed back our own previous setpoint and let
  // the drive's closed loop chase it. Field-relative.
  private Translation2d lastSetpoint = Translation2d.kZero;

  public AutopilotTrenchCommand() {
    this(null);
  }

  /**
   * Creates a trench pass that finishes at the next Choreo path's starting pose, arriving at that
   * path's ideal starting velocity. A path that begins at rest gets a stop-and-settle handoff; one
   * that begins moving gets a full-speed handoff (the pass never brakes — path following takes over
   * with matching velocity).
   */
  public AutopilotTrenchCommand(String nextPathName) {
    this.drive = Drive.mInstance;
    PathPlannerPath nextPath = nextPathName == null ? null : loadPath(nextPathName);
    this.nextPathStartPose = nextPath == null ? null : startPoseOf(nextPath, nextPathName);
    this.handoffVelocityMetersPerSec = nextPath == null ? 0.0 : handoffVelocityOf(nextPath);
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    Pose2d pose = drive.getPose();
    // The trench is anchored to the next path's start when there is one: the command's job is to
    // deliver the robot to that pose. Only the no-path variant picks the trench closest to the
    // robot.
    Pose2d nextStart = getAllianceAdjustedNextPathStartPose();
    Pose2d anchor = nextStart != null ? nextStart : pose;
    double trenchX = getClosestTrenchX(anchor.getX());
    trenchY = getClosestTrenchY(anchor.getY());

    travelingPositiveX = pose.getX() < trenchX;
    double direction = travelingPositiveX ? 1.0 : -1.0;
    Rotation2d travelDirection = travelingPositiveX ? Rotation2d.kZero : Rotation2d.kPi;
    // The back of the robot leads through the trench.
    passHeading = travelDirection.plus(Rotation2d.kPi);

    // Entrance/goal targets sit a margin outside the physical faces of the 1.2 m structure.
    double halfLength = AutoTrenchCommandConstants.TRENCH_HALF_LENGTH_METERS;
    double entranceX =
        trenchX - direction * (halfLength + AutopilotTrenchCommandConstants.ENTRANCE_MARGIN_METERS);
    double exitX =
        trenchX + direction * (halfLength + AutopilotTrenchCommandConstants.EXIT_MARGIN_METERS);
    passTransitionX =
        entranceX - direction * AutopilotTrenchCommandConstants.PASS_TARGET_LOOKAHEAD_METERS;

    entranceTarget =
        new APTarget(new Pose2d(entranceX, trenchY, passHeading))
            .withEntryAngle(travelDirection)
            .withVelocity(AutopilotTrenchCommandConstants.PASS_SPEED_METERS_PER_SEC);
    Translation2d goal =
        nextStart != null ? nextStart.getTranslation() : new Translation2d(exitX, trenchY);
    goalTarget =
        new APTarget(new Pose2d(goal, passHeading))
            .withEntryAngle(travelDirection)
            .withVelocity(handoffVelocityMetersPerSec);

    nearSideGoal = travelingPositiveX ? goal.getX() <= entranceX : goal.getX() >= entranceX;
    stage = !nearSideGoal && hasPassedX(pose.getX(), passTransitionX) ? Stage.PASS : Stage.APPROACH;

    angleController.reset(drive.getRotation().getRadians());
    // Seed the velocity setpoint from the measured speed so a moving start ramps smoothly.
    ChassisSpeeds measured = drive.getChassisSpeeds();
    lastSetpoint =
        new Translation2d(measured.vxMetersPerSecond, measured.vyMetersPerSecond)
            .rotateBy(drive.getRotation());
  }

  @Override
  public void execute() {
    // isFinished() runs after execute: once the finish condition is met, hold the previous
    // velocity request instead of computing a new one toward a target now behind the robot.
    if (isFinished()) {
      return;
    }
    Pose2d pose = drive.getPose();
    if (stage == Stage.APPROACH && !nearSideGoal && readyToPass(pose)) {
      stage = Stage.PASS;
    }
    APTarget target = stage == Stage.PASS ? goalTarget : entranceTarget;

    Translation2d setpointRobotRelative = lastSetpoint.rotateBy(drive.getRotation().unaryMinus());
    Autopilot.APResult output =
        autopilot.calculate(
            pose,
            new ChassisSpeeds(setpointRobotRelative.getX(), setpointRobotRelative.getY(), 0.0),
            target);
    double vx = output.vx().in(MetersPerSecond);
    double vy = output.vy().in(MetersPerSecond);
    if (stage == Stage.APPROACH) {
      // Cap the arc speed at what the drivetrain can actually track around the remaining curve;
      // commanding more makes the robot lag the curve and overshoot the centerline. The cap is
      // recomputed each cycle, so a shallow approach runs near full speed and the robot
      // re-accelerates along the tangent as the arc straightens out.
      double speed = Math.hypot(vx, vy);
      double cap = maxTrackableArcSpeed(pose, target);
      if (speed > cap) {
        vx *= cap / speed;
        vy *= cap / speed;
      }
      // Never approach the centerline faster than can be braked to zero on it. This makes the
      // merge critically damped: lateral speed reaches zero exactly on the centerline while the
      // X speed carries through, so the arc joins the trench straight tangentially instead of
      // swinging past it toward the wall. The brake accel is derated well below the real limit
      // because the drivetrain tracks the command with lag — the command must lead the plant.
      double toCenter = trenchY - pose.getY();
      double brakeableVy =
          Math.sqrt(
              2.0
                  * AutopilotTrenchCommandConstants.CENTERLINE_BRAKE_ACCEL_METERS_PER_SEC_SQ
                  * Math.abs(toCenter));
      if (toCenter > 0.0) {
        vy = Math.min(vy, brakeableVy);
      } else {
        vy = Math.max(vy, -brakeableVy);
      }
    }
    lastSetpoint = new Translation2d(vx, vy);

    double omega =
        MathUtil.clamp(
            angleController.calculate(drive.getRotation().getRadians(), passHeading.getRadians()),
            -AutopilotTrenchCommandConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC,
            AutopilotTrenchCommandConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC);

    logOutputs(vx, vy, omega);
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(vx, vy, omega), drive.getRotation()));
  }

  @Override
  public boolean isFinished() {
    Pose2d pose = drive.getPose();
    if (nearSideGoal) {
      // The next path drives the trench itself; hand off as soon as the robot crosses its start
      // plane running along the centerline. The arc's tangent line passes through the path start,
      // so this happens naturally at the end of APPROACH.
      return hasPassedX(pose.getX(), goalTarget.getReference().getX()) && isAlignedWithTrench(pose);
    }
    if (stage != Stage.PASS) {
      return false;
    }
    if (handoffVelocityMetersPerSec > 0.0) {
      // Moving handoff: at pass speed the robot covers ~10 cm per cycle, so Autopilot's positional
      // tolerance can be skipped over entirely. Finish on crossing the goal plane and let the next
      // path's feedback controller absorb the residual error.
      return hasPassedX(pose.getX(), goalTarget.getReference().getX());
    }
    return autopilot.atTarget(pose, goalTarget);
  }

  @Override
  public void end(boolean interrupted) {
    Logger.recordOutput("AutopilotTrench/EndReason", interrupted ? "interrupted" : "finished");
    Logger.recordOutput("AutopilotTrench/EndPose", drive.getPose());
    // On a moving handoff the next path command initializes on the following cycle; commanding a
    // stop here would insert one cycle of hard braking between the two. The drive holds the last
    // velocity request until the path takes over.
    if (interrupted || handoffVelocityMetersPerSec == 0.0) {
      drive.stop();
    }
  }

  /**
   * The switch from the entrance target to the goal happens just before the entrance, once the
   * robot is on the centerline — waiting for the exact entrance pose would make Autopilot settle
   * there. Passing the entrance plane itself forces the switch so the entrance target is never
   * chased from behind.
   */
  private boolean readyToPass(Pose2d pose) {
    if (hasPassedX(pose.getX(), entranceTarget.getReference().getX())) {
      return true;
    }
    return hasPassedX(pose.getX(), passTransitionX) && isAlignedWithTrench(pose);
  }

  /**
   * The fastest speed the drivetrain can hold while tracking the remainder of Autopilot's
   * entry-angle arc. The arc's tightest turn is at its very end, with radius roughly (distance to
   * target) / (2 * offset angle seen from the entry direction); the classic v = sqrt(a * r) limit
   * is applied there. A nearly-straight approach therefore runs at full pass speed, and only a
   * near-perpendicular one is slowed to what the turn actually allows.
   */
  private double maxTrackableArcSpeed(Pose2d pose, APTarget target) {
    Rotation2d entryAngle = passHeading.plus(Rotation2d.kPi);
    Translation2d offset =
        target
            .getReference()
            .getTranslation()
            .minus(pose.getTranslation())
            .rotateBy(entryAngle.unaryMinus());
    double offsetAngle = Math.abs(Math.atan2(offset.getY(), offset.getX()));
    if (offsetAngle < 1e-3) {
      return AutopilotTrenchCommandConstants.PASS_SPEED_METERS_PER_SEC;
    }
    double endRadius = offset.getNorm() / (2.0 * offsetAngle);
    double cap =
        Math.sqrt(AutopilotTrenchCommandConstants.MAX_ACCELERATION_METERS_PER_SEC_SQ * endRadius);
    return MathUtil.clamp(
        cap,
        AutopilotTrenchCommandConstants.APPROACH_MIN_SPEED_METERS_PER_SEC,
        AutopilotTrenchCommandConstants.PASS_SPEED_METERS_PER_SEC);
  }

  private boolean isAlignedWithTrench(Pose2d pose) {
    boolean lateralOk =
        Math.abs(pose.getY() - trenchY) <= AutopilotTrenchCommandConstants.ALIGN_Y_TOLERANCE_METERS;
    boolean headingOk =
        Math.abs(pose.getRotation().minus(passHeading).getDegrees())
            <= AutopilotTrenchCommandConstants.ALIGN_HEADING_TOLERANCE_DEGREES;
    return lateralOk && headingOk;
  }

  /** Whether the given X is at or past the given plane, in the direction of travel. */
  private boolean hasPassedX(double x, double planeX) {
    return travelingPositiveX ? x >= planeX : x <= planeX;
  }

  private void logOutputs(double vx, double vy, double omega) {
    Logger.recordOutput("AutopilotTrench/Stage", stage.toString());
    Logger.recordOutput("AutopilotTrench/EntrancePose", entranceTarget.getReference());
    Logger.recordOutput("AutopilotTrench/GoalPose", goalTarget.getReference());
    Logger.recordOutput("AutopilotTrench/HandoffVelocity", handoffVelocityMetersPerSec);
    Logger.recordOutput(
        "AutopilotTrench/LateralError", drive.getPose().getY() - goalTarget.getReference().getY());
    Logger.recordOutput("AutopilotTrench/Vx", vx);
    Logger.recordOutput("AutopilotTrench/Vy", vy);
    Logger.recordOutput("AutopilotTrench/Omega", omega);
    Logger.recordOutput("AutopilotTrench/CommandedSpeed", Math.hypot(vx, vy));
    ChassisSpeeds measured = drive.getChassisSpeeds();
    Logger.recordOutput(
        "AutopilotTrench/MeasuredSpeed",
        Math.hypot(measured.vxMetersPerSecond, measured.vyMetersPerSecond));
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

  private static PathPlannerPath loadPath(String pathName) {
    try {
      return PathPlannerPath.fromChoreoTrajectory(pathName);
    } catch (Exception e) {
      throw new IllegalArgumentException("Failed to load next Choreo path: " + pathName, e);
    }
  }

  private static Pose2d startPoseOf(PathPlannerPath path, String pathName) {
    return path.getStartingHolonomicPose()
        .orElseThrow(() -> new IllegalArgumentException("Path has no starting pose: " + pathName));
  }

  /**
   * The speed to carry across the handoff: the next path's ideal starting speed, capped at the
   * trench pass speed so Autopilot's own velocity constraint can actually deliver it.
   */
  private static double handoffVelocityOf(PathPlannerPath path) {
    IdealStartingState idealStart = path.getIdealStartingState();
    return idealStart == null
        ? 0.0
        : Math.min(
            idealStart.velocityMPS(), AutopilotTrenchCommandConstants.PASS_SPEED_METERS_PER_SEC);
  }

  private Pose2d getAllianceAdjustedNextPathStartPose() {
    if (nextPathStartPose == null) {
      return null;
    }
    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    return isRed ? FlippingUtil.flipFieldPose(nextPathStartPose) : nextPathStartPose;
  }
}
