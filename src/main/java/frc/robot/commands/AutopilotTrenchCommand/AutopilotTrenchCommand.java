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
 * Passes through the closest trench using the Autopilot library, in three stages.
 *
 * <p>ALIGN first moves to a centerline waypoint in the open area before the trench. APPROACH then
 * moves straight along the trench centerline to its entrance, and PASS continues on that same line
 * to the exit. No entry-angle target is used: Autopilot's entry-angle behavior intentionally makes
 * a spiral, which is undesirable beside the REBUILT hub and bump obstacles.
 *
 * <p>Heading is locked opposite to the travel direction (back of the robot leads through the
 * trench) and held through the pass.
 *
 * <p>The finish depends on the next path's ideal starting state, supporting three handoff layouts:
 *
 * <ul>
 *   <li><b>At rest</b> (default StopPoint on the first waypoint): the exit target has zero end
 *       velocity and the command settles there before finishing — the original behavior.
 *   <li><b>Moving, past the trench exit</b>: the exit target carries the path's start velocity and
 *       the command finishes the moment the robot crosses the handoff plane in PASS.
 *   <li><b>Moving, before the trench entrance</b> (the path itself drives through the trench):
 *       APPROACH targets the path's start pose directly at its start velocity. The command finishes
 *       when aligned with the trench and across the handoff plane; the trench pass itself belongs
 *       to the path.
 * </ul>
 *
 * <p>In every layout the approach starts from wherever the previous action left the robot — ALIGN
 * never retreats for an acceleration runway (entering slow costs less time than backward travel),
 * and its target X tracks the robot so strafe drift is kept rather than corrected backward. The
 * only floor is a short settle runway before the point where alignment must be complete.
 */
public class AutopilotTrenchCommand extends Command {
  private enum Stage {
    ALIGN,
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
   * is no next path or it starts at rest — the command then settles at the exit target instead.
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

  private APTarget alignmentTarget = new APTarget(Pose2d.kZero);
  private APTarget entranceTarget = new APTarget(Pose2d.kZero);
  private APTarget exitTarget = new APTarget(Pose2d.kZero);
  private Stage stage = Stage.ALIGN;
  private double entranceX = 0.0;
  private double trenchY = 0.0;
  private Rotation2d lockedHeading = Rotation2d.kZero;
  private boolean travelingPositiveX = true;
  // True when the next path starts before the trench entrance (the path drives the trench itself):
  // APPROACH then targets the path start directly and the command hands off there.
  private boolean nearSideHandoff = false;
  // True when the alignment target's X tracks the robot each cycle (clamped at
  // alignmentFollowLimitX). The strafe+spin onto the centerline drifts X uncontrollably; a fixed
  // alignment X would make Autopilot drag the robot back to it, which reads as pointless
  // reversing.
  private boolean alignmentFollowsRobot = false;
  // Forward-most X the alignment target may reach: the handoff plane for a near-side handoff,
  // or a short settle runway before the entrance target otherwise (the APPROACH->PASS gate at
  // the entrance plane requires alignment to already be done).
  private double alignmentFollowLimitX = 0.0;
  // Autopilot ramps its output from the speeds we feed it (output = fed speed + accel*dt). Feeding
  // measured speeds throttles the ramp to the drivetrain's tracking lag, so we feed back our own
  // previous setpoint instead and let the drive's closed loop chase it. Field-relative.
  private Translation2d lastSetpointFieldRel = Translation2d.kZero;

  public AutopilotTrenchCommand() {
    this(null);
  }

  /**
   * Creates a trench pass that finishes exactly at the next Choreo path's starting pose, arriving
   * at that path's ideal starting velocity. A trajectory that begins at a zero-velocity StopPoint
   * gets the old stop-and-settle handoff; one that begins moving gets a full-speed chained handoff
   * (the pass never brakes — path following takes over with matching velocity).
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
    // deliver the robot to that pose, and after a shot the robot can easily sit closer to the
    // other trench. Only the no-path variant picks the trench closest to the robot.
    Pose2d nextStart = getAllianceAdjustedNextPathStartPose();
    double trenchX = getClosestTrenchX(nextStart != null ? nextStart.getX() : pose.getX());
    trenchY = getClosestTrenchY(nextStart != null ? nextStart.getY() : pose.getY());

    // Travel direction along X decides which side of the trench each target sits on and the
    // entry angle Autopilot funnels along. The locked heading faces opposite to travel, so the
    // robot always passes the trench back-first.
    travelingPositiveX = pose.getX() < trenchX;
    lockedHeading = travelingPositiveX ? Rotation2d.kPi : Rotation2d.kZero;
    double direction = travelingPositiveX ? 1.0 : -1.0;
    // Entrance/exit targets sit a margin outside the physical entry/exit faces of the 1.2 m
    // structure (faces are at center +/- half length).
    double halfLength = AutoTrenchCommandConstants.TRENCH_HALF_LENGTH_METERS;
    entranceX =
        trenchX - direction * (halfLength + AutopilotTrenchCommandConstants.ENTRANCE_MARGIN_METERS);
    double exitX =
        trenchX + direction * (halfLength + AutopilotTrenchCommandConstants.EXIT_MARGIN_METERS);
    Pose2d entrancePose = new Pose2d(entranceX, trenchY, lockedHeading);
    // No entry angle on either target. Translation therefore uses an ordinary direct approach
    // rather than Autopilot's spiral. ALIGN ends at rest; APPROACH accelerates straight down the
    // centerline and carries its velocity into PASS.
    entranceTarget =
        new APTarget(entrancePose)
            .withVelocity(AutopilotTrenchCommandConstants.PASS_SPEED_METERS_PER_SEC);
    Pose2d finalPose = nextStart;
    if (finalPose == null) {
      finalPose = new Pose2d(exitX, trenchY, lockedHeading);
    }
    exitTarget = new APTarget(finalPose);
    if (handoffVelocityMetersPerSec > 0.0) {
      exitTarget = exitTarget.withVelocity(handoffVelocityMetersPerSec);
    }
    nearSideHandoff =
        handoffVelocityMetersPerSec > 0.0
            && (travelingPositiveX ? finalPose.getX() <= entranceX : finalPose.getX() >= entranceX);

    // The approach starts from wherever the shot left the robot, projected onto the centerline —
    // backing up for an acceleration runway costs more time than entering slow and letting the
    // speed build later. The only constraint is where alignment must be DONE: at the handoff
    // plane for a near-side handoff (finishing ahead of the trajectory start would make the path
    // controller pull backward), or a short settle runway before the entrance target otherwise
    // (the APPROACH->PASS gate at the entrance plane requires alignment). Only a start at/past
    // that limit backs off to it.
    alignmentFollowLimitX =
        nearSideHandoff
            ? finalPose.getX()
            : entranceX - direction * AutopilotTrenchCommandConstants.MIN_HANDOFF_RUNWAY_METERS;
    boolean pastAlignmentLimit =
        travelingPositiveX
            ? pose.getX() >= alignmentFollowLimitX
            : pose.getX() <= alignmentFollowLimitX;
    double alignmentX;
    if (pastAlignmentLimit) {
      alignmentX =
          nearSideHandoff
              ? finalPose.getX()
                  - direction * AutopilotTrenchCommandConstants.MIN_HANDOFF_RUNWAY_METERS
              : alignmentFollowLimitX;
      alignmentFollowsRobot = false;
    } else {
      alignmentX = pose.getX();
      alignmentFollowsRobot = true;
    }
    alignmentTarget = new APTarget(new Pose2d(alignmentX, trenchY, lockedHeading));

    if (hasCrossedEntrancePlane(pose) && isAlignedWithTrench(pose)) {
      stage = Stage.PASS;
    } else if (isReadyForStraightApproach(pose, alignmentX)) {
      stage = Stage.APPROACH;
    } else {
      stage = Stage.ALIGN;
    }
    angleController.reset(drive.getRotation().getRadians());

    // Seed the velocity setpoint from the measured speed so a moving start ramps smoothly.
    ChassisSpeeds measured = drive.getChassisSpeeds();
    lastSetpointFieldRel =
        new Translation2d(measured.vxMetersPerSecond, measured.vyMetersPerSecond)
            .rotateBy(drive.getRotation());
  }

  @Override
  public void execute() {
    Pose2d pose = drive.getPose();
    if (stage == Stage.ALIGN && alignmentFollowsRobot) {
      // Pin the alignment X to the robot (clamped at the follow limit): only Y and heading are
      // corrected during ALIGN, so forward drift from the strafe+spin is kept instead of being
      // dragged back to where the strafe happened to start.
      double alignX =
          travelingPositiveX
              ? Math.min(pose.getX(), alignmentFollowLimitX)
              : Math.max(pose.getX(), alignmentFollowLimitX);
      alignmentTarget = alignmentTarget.withReference(new Pose2d(alignX, trenchY, lockedHeading));
    }
    if (stage == Stage.ALIGN && autopilot.atTarget(pose, alignmentTarget)) {
      stage = Stage.APPROACH;
      lastSetpointFieldRel = Translation2d.kZero;
    }
    if (stage == Stage.APPROACH && hasCrossedEntrancePlane(pose)) {
      if (isAlignedWithTrench(pose)) {
        stage = Stage.PASS;
      }
    }
    APTarget target;
    if (stage == Stage.PASS) {
      target = exitTarget;
    } else if (stage == Stage.APPROACH) {
      // Near-side handoff: the path start (with its velocity) is the approach target itself.
      target = nearSideHandoff ? exitTarget : entranceTarget;
    } else {
      target = alignmentTarget;
    }
    // Feed the previous velocity setpoint (as robot-relative speeds) instead of measured speeds:
    // Autopilot's acceleration ramp then runs at the configured rate regardless of how much the
    // drivetrain lags, and the modules' closed loop closes the gap.
    Translation2d setpointRobotRel =
        lastSetpointFieldRel.rotateBy(drive.getRotation().unaryMinus());
    Autopilot.APResult output =
        autopilot.calculate(
            pose, new ChassisSpeeds(setpointRobotRel.getX(), setpointRobotRel.getY(), 0.0), target);

    double vx = output.vx().in(MetersPerSecond);
    double vy = output.vy().in(MetersPerSecond);
    lastSetpointFieldRel = new Translation2d(vx, vy);
    double omega =
        MathUtil.clamp(
            angleController.calculate(
                drive.getRotation().getRadians(), output.targetAngle().getRadians()),
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
    if (nearSideHandoff) {
      // The path drives the trench itself: hand off once lined up on the centerline and across
      // the path-start plane. ALIGN is excluded so the handoff can't fire while still swinging
      // onto the centerline.
      return stage != Stage.ALIGN && isAlignedWithTrench(pose) && hasCrossedHandoffPlane(pose);
    }
    if (stage != Stage.PASS) {
      return false;
    }
    if (handoffVelocityMetersPerSec > 0.0) {
      // Moving handoff: at pass speed the robot covers ~8 cm per cycle, so Autopilot's positional
      // atTarget tolerance can be skipped over entirely. Finish on crossing the handoff plane and
      // let the next path's feedback controller absorb the residual error.
      return hasCrossedHandoffPlane(pose);
    }
    return autopilot.atTarget(pose, exitTarget);
  }

  private boolean hasCrossedHandoffPlane(Pose2d pose) {
    double handoffX = exitTarget.getReference().getX();
    return travelingPositiveX ? pose.getX() >= handoffX : pose.getX() <= handoffX;
  }

  private boolean hasCrossedEntrancePlane(Pose2d pose) {
    return travelingPositiveX ? pose.getX() >= entranceX : pose.getX() <= entranceX;
  }

  private boolean isAlignedWithTrench(Pose2d pose) {
    boolean lateralOk =
        Math.abs(pose.getY() - trenchY) <= AutopilotTrenchCommandConstants.ALIGN_Y_TOLERANCE_METERS;
    boolean headingOk =
        Math.abs(pose.getRotation().minus(lockedHeading).getDegrees())
            <= AutopilotTrenchCommandConstants.ALIGN_HEADING_TOLERANCE_DEGREES;
    return lateralOk && headingOk;
  }

  private boolean isReadyForStraightApproach(Pose2d pose, double alignmentX) {
    return Math.abs(pose.getY() - trenchY)
            <= AutopilotTrenchCommandConstants.ALIGN_Y_TOLERANCE_METERS
        && Math.abs(pose.getX() - alignmentX)
            <= AutopilotTrenchCommandConstants.ALIGN_X_TOLERANCE_METERS;
  }

  @Override
  public void end(boolean interrupted) {
    Logger.recordOutput("AutopilotTrench/EndReason", interrupted ? "interrupted" : "finished");
    Logger.recordOutput("AutopilotTrench/EndPose", drive.getPose());
    Logger.recordOutput("AutopilotTrench/EndHandoffX", exitTarget.getReference().getX());
    // On a moving handoff the next path command initializes on the following cycle; commanding a
    // stop here would insert one cycle of hard braking between the two. The drive holds the last
    // velocity request until the path takes over.
    if (interrupted || handoffVelocityMetersPerSec == 0.0) {
      drive.stop();
    }
  }

  private void logOutputs(double vx, double vy, double omega) {
    Logger.recordOutput("AutopilotTrench/Stage", stage.toString());
    Logger.recordOutput("AutopilotTrench/HandoffVelocity", handoffVelocityMetersPerSec);
    Logger.recordOutput("AutopilotTrench/AlignmentPose", alignmentTarget.getReference());
    Logger.recordOutput("AutopilotTrench/EntrancePose", entranceTarget.getReference());
    Logger.recordOutput("AutopilotTrench/ExitPose", exitTarget.getReference());
    Logger.recordOutput(
        "AutopilotTrench/LateralError", drive.getPose().getY() - exitTarget.getReference().getY());
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
   * The speed to carry across the handoff: the next path's ideal starting speed (Choreo's first
   * trajectory sample), capped at the trench pass speed so Autopilot's own velocity constraint can
   * actually deliver it.
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
