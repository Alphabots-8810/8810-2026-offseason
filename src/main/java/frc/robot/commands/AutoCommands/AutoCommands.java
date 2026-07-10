package frc.robot.commands.AutoCommands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.IntakeCommand.IntakeCommand;
import frc.robot.commands.ShootingCommand.Shooting;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;

/**
 * Autonomous routines built in Java as a sequential command group.
 *
 * <p>{@code leftThreePieceAuto()} runs: Left1_path1 (intake opens 1 s in), timed Shooting,
 * Left1_path2 while intaking, timed Shooting, Left1_path3 while intaking.
 */
public final class AutoCommands {

  private AutoCommands() {}

  static Command followChoreo(String name) {
    return AutoBuilder.followPath(loadChoreoPath(name));
  }

  static PathPlannerPath loadChoreoPath(String name) {
    try {
      return PathPlannerPath.fromChoreoTrajectory(name);
    } catch (Exception e) {
      throw new RuntimeException("Failed to load Choreo trajectory '" + name + "'", e);
    }
  }

  /**
   * Reset odometry to a Choreo path's holonomic start pose, mirrored to the red alliance side when
   * on red (Choreo paths are authored on the blue origin). No-op if the path has no start pose.
   */
  static Command resetToStart(PathPlannerPath path) {
    Optional<Pose2d> start = path.getStartingHolonomicPose();
    if (start.isEmpty()) {
      return Commands.none();
    }
    Pose2d pose = start.get();
    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    Pose2d startPose = isRed ? FlippingUtil.flipFieldPose(pose) : pose;
    return Commands.runOnce(() -> Drive.mInstance.setPose(startPose), Drive.mInstance);
  }

  /**
   * Path followed while the intake deploys and runs the whole time, stopping when the path ends.
   */
  static Command pathWithIntake(String name) {
    return followChoreo(name).raceWith(new IntakeCommand());
  }

  /**
   * First path of an auto: reset odometry to the path's start pose, then follow it with the intake
   * deploying and running from {@code INTAKE_START_DELAY_SEC} in until the path ends.
   */
  static Command firstPathWithIntake(String name) {
    PathPlannerPath path = loadChoreoPath(name);
    return Commands.sequence(
        resetToStart(path),
        followChoreo(name)
            .raceWith(
                Commands.sequence(
                    Commands.waitSeconds(AutoCommandsConstants.INTAKE_START_DELAY_SEC),
                    new IntakeCommand())));
  }

  /** Timed shooting burst: aim + shoot for a fixed duration, then cleanly retract. */
  static Command timedShoot() {
    return new Shooting(false, () -> 0.0, () -> 0.0)
        .withTimeout(AutoCommandsConstants.SHOOTING_DURATION_SEC);
  }

  /**
   * Left three-piece auto:
   *
   * <ol>
   *   <li>Reset pose to Left1_path1 start, then follow Left1_path1; intake opens 1 s after the path
   *       starts and runs until the path finishes.
   *   <li>Timed Shooting.
   *   <li>Follow Left1_path2 while intaking.
   *   <li>Timed Shooting.
   *   <li>Follow Left1_path3 while intaking.
   * </ol>
   */
  public static Command leftThreePieceAuto() {
    try {
      return buildLeftThreePieceAuto();
    } catch (RuntimeException e) {
      // A missing/renamed .traj must never crash the robot program: report it and hand the
      // chooser a no-op so everything else (teleop, other autos) still works.
      DriverStation.reportError("Left three-piece auto failed to load: " + e.getMessage(), false);
      return Commands.print("Left three-piece auto unavailable: missing Choreo trajectory");
    }
  }

  private static Command buildLeftThreePieceAuto() {
    PathPlannerPath path1 = loadChoreoPath(AutoCommandsConstants.LEFT_PATH_1);

    return Commands.sequence(
        // 1) Drive Left1_path1, opening the intake 1 s in. The race ends with the path, so the
        //    intake's end() stops the roller/indexer exactly when the path finishes.
        Commands.sequence(
            resetToStart(path1),
            followChoreo(AutoCommandsConstants.LEFT_PATH_1)
                .raceWith(
                    Commands.sequence(
                        Commands.waitSeconds(AutoCommandsConstants.INTAKE_START_DELAY_SEC),
                        new IntakeCommand()))),

        // 2) Timed shooting.
        timedShoot(),

        // 3) Left1_path2 while intaking.
        pathWithIntake(AutoCommandsConstants.LEFT_PATH_2),

        // 4) Timed shooting.
        timedShoot(),

        // 5) Left1_path3 while intaking.
        pathWithIntake(AutoCommandsConstants.LEFT_PATH_3));
  }
}
