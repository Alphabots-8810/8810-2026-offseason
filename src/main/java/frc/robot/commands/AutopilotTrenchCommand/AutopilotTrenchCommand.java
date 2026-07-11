package frc.robot.commands.AutopilotTrenchCommand;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

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
 * trench) and held through the pass. Finishes when Autopilot reports the exit target reached.
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
  // Autopilot ramps its output from the speeds we feed it (output = fed speed + accel*dt). Feeding
  // measured speeds throttles the ramp to the drivetrain's tracking lag, so we feed back our own
  // previous setpoint instead and let the drive's closed loop chase it. Field-relative.
  private Translation2d lastSetpointFieldRel = Translation2d.kZero;

  public AutopilotTrenchCommand() {
    this(null);
  }

  /**
   * Creates a trench pass that finishes exactly at the next Choreo path's starting pose. The
   * current Path2/Path3 trajectories begin at a zero-velocity StopPoint, so Autopilot stops there
   * for a safe and continuous handoff.
   */
  public AutopilotTrenchCommand(String nextPathName) {
    this.drive = Drive.mInstance;
    this.nextPathStartPose = nextPathName == null ? null : loadPathStartPose(nextPathName);
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    Pose2d pose = drive.getPose();
    double trenchX = getClosestTrenchX(pose.getX());
    trenchY = getClosestTrenchY(pose.getY());

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
    double alignmentX =
        entranceX - direction * AutopilotTrenchCommandConstants.ALIGNMENT_STANDOFF_METERS;
    alignmentTarget = new APTarget(new Pose2d(alignmentX, trenchY, lockedHeading));
    // No entry angle on either target. Translation therefore uses an ordinary direct approach
    // rather than Autopilot's spiral. ALIGN ends at rest; APPROACH accelerates straight down the
    // centerline and carries its velocity into PASS.
    entranceTarget =
        new APTarget(entrancePose)
            .withVelocity(AutopilotTrenchCommandConstants.PASS_SPEED_METERS_PER_SEC);
    Pose2d finalPose = getAllianceAdjustedNextPathStartPose();
    if (finalPose == null) {
      finalPose = new Pose2d(exitX, trenchY, lockedHeading);
    }
    exitTarget = new APTarget(finalPose);

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
      target = entranceTarget;
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
    return stage == Stage.PASS && autopilot.atTarget(drive.getPose(), exitTarget);
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
    drive.stop();
  }

  private void logOutputs(double vx, double vy, double omega) {
    Logger.recordOutput("AutopilotTrench/Stage", stage.toString());
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

  private static Pose2d loadPathStartPose(String pathName) {
    try {
      return PathPlannerPath.fromChoreoTrajectory(pathName)
          .getStartingHolonomicPose()
          .orElseThrow(
              () -> new IllegalArgumentException("Path has no starting pose: " + pathName));
    } catch (Exception e) {
      throw new IllegalArgumentException("Failed to load next Choreo path: " + pathName, e);
    }
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
