package frc.robot.commands.AutopilotTrenchCommand;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

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
 * Passes through the closest trench using the Autopilot library, in two stages.
 *
 * <p>Autopilot's entry-angle spiral only finishes converging onto the entry line at its target, so
 * a single target past the trench would leave the robot correcting diagonally while inside the
 * structure. Instead, stage 1 (APPROACH) targets the trench <i>entrance</i> with a nonzero end
 * velocity, so the robot is centered on the trench line before the structure and carries full pass
 * speed through the entrance instead of decelerating. Once the entrance plane is crossed, stage 2
 * (PASS) retargets to the exit pose; the robot is already on the centerline, so this leg is a
 * straight line that decelerates to a stop past the structure.
 *
 * <p>Heading is locked opposite to the travel direction (back of the robot leads through the
 * trench) and held through the pass. Finishes when Autopilot reports the exit target reached.
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
  private final ProfiledPIDController angleController =
      new ProfiledPIDController(
          AutopilotTrenchCommandConstants.ANGLE_KP,
          0.0,
          AutopilotTrenchCommandConstants.ANGLE_KD,
          new TrapezoidProfile.Constraints(
              AutopilotTrenchCommandConstants.ANGLE_MAX_VELOCITY_RAD_PER_SEC,
              AutopilotTrenchCommandConstants.ANGLE_MAX_ACCELERATION_RAD_PER_SEC_SQ));

  private APTarget entranceFlowTarget = new APTarget(Pose2d.kZero);
  private APTarget entranceStopTarget = new APTarget(Pose2d.kZero);
  private APTarget exitTarget = new APTarget(Pose2d.kZero);
  private Stage stage = Stage.APPROACH;
  private double entranceX = 0.0;
  private double trenchY = 0.0;
  private Rotation2d lockedHeading = Rotation2d.kZero;
  private boolean travelingPositiveX = true;
  // Flow-through: entry-angle spiral carrying pass speed across the entrance. Cleared when there
  // isn't enough runway for the spiral to converge, or if the plane is crossed misaligned; the
  // approach then beelines to the entrance point and stops there instead.
  private boolean flowThroughApproach = true;

  // Autopilot ramps its output from the speeds we feed it (output = fed speed + accel*dt). Feeding
  // measured speeds throttles the ramp to the drivetrain's tracking lag, so we feed back our own
  // previous setpoint instead and let the drive's closed loop chase it. Field-relative.
  private Translation2d lastSetpointFieldRel = Translation2d.kZero;

  public AutopilotTrenchCommand() {
    this.drive = Drive.mInstance;
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    Pose2d pose = drive.getPose();
    boolean isRedAlliance =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    double trenchX =
        isRedAlliance
            ? AutoTrenchCommandConstants.RED_TRENCH_X_METERS
            : AutoTrenchCommandConstants.BLUE_TRENCH_X_METERS;
    trenchY = getClosestTrenchY(pose.getY());

    // Travel direction along X decides which side of the trench each target sits on and the
    // entry angle Autopilot funnels along. The locked heading faces opposite to travel, so the
    // robot always passes the trench back-first.
    travelingPositiveX = pose.getX() < trenchX;
    lockedHeading = travelingPositiveX ? Rotation2d.kPi : Rotation2d.kZero;
    double direction = travelingPositiveX ? 1.0 : -1.0;
    entranceX = trenchX - direction * AutopilotTrenchCommandConstants.ENTRANCE_DISTANCE_METERS;
    double exitX = trenchX + direction * AutopilotTrenchCommandConstants.EXIT_DISTANCE_METERS;
    Rotation2d entryAngle = travelingPositiveX ? Rotation2d.kZero : Rotation2d.kPi;

    Pose2d entrancePose = new Pose2d(entranceX, trenchY, lockedHeading);
    // Nonzero end velocity keeps the commanded speed at the cruise cap through the entrance
    // instead of profiling down to a stop there.
    entranceFlowTarget =
        new APTarget(entrancePose)
            .withEntryAngle(entryAngle)
            .withVelocity(AutopilotTrenchCommandConstants.PASS_SPEED_METERS_PER_SEC);
    // No entry angle: beeline straight to the entrance point and stop there. Used when there is
    // no runway for the spiral to converge before the structure.
    entranceStopTarget = new APTarget(entrancePose);
    exitTarget = new APTarget(new Pose2d(exitX, trenchY, lockedHeading)).withEntryAngle(entryAngle);

    double runway = direction * (entranceX - pose.getX());
    flowThroughApproach = runway >= AutopilotTrenchCommandConstants.MIN_RUNWAY_METERS;
    stage =
        hasCrossedEntrancePlane(pose) && isAlignedWithTrench(pose) ? Stage.PASS : Stage.APPROACH;
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
    if (stage == Stage.APPROACH && hasCrossedEntrancePlane(pose)) {
      if (isAlignedWithTrench(pose)) {
        stage = Stage.PASS;
      } else {
        // Crossed the plane misaligned: never enter the trench like this. Fall back to driving
        // straight to the entrance point (which is now behind/off to the side) and settling there.
        flowThroughApproach = false;
      }
    }
    APTarget target;
    if (stage == Stage.PASS) {
      target = exitTarget;
    } else {
      target = flowThroughApproach ? entranceFlowTarget : entranceStopTarget;
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

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  private void logOutputs(double vx, double vy, double omega) {
    Logger.recordOutput("AutopilotTrench/Stage", stage.toString());
    Logger.recordOutput("AutopilotTrench/FlowThroughApproach", flowThroughApproach);
    Logger.recordOutput("AutopilotTrench/EntrancePose", entranceFlowTarget.getReference());
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

  private static double getClosestTrenchY(double robotY) {
    if (robotY >= AutoTrenchCommandConstants.FIELD_MID_Y_METERS) {
      return AutoTrenchCommandConstants.UP_TRENCH_Y_METERS;
    }
    return AutoTrenchCommandConstants.DOWN_TRENCH_Y_METERS;
  }
}
