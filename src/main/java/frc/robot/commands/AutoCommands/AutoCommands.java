package frc.robot.commands.AutoCommands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.events.Event;
import com.pathplanner.lib.events.OneShotTriggerEvent;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutopilotTrenchCommand.AutopilotTrenchCommand;
import frc.robot.commands.IntakeCommand.IntakeCommand;
import frc.robot.commands.ShootingCommand.Shooting;
import frc.robot.subsystems.Drum.Drum;
import frc.robot.subsystems.drive.Drive;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

/**
 * Building blocks for autonomous routines: Choreo path following, odometry reset, intake/shoot
 * helpers. Assembled into full autos by {@link BlockAutoBuilder}.
 */
public final class AutoCommands {
  private static final String PID_TEST_PATH = "PIDtest";

  /**
   * Choreo event marker name that ends the intake phase of the first path: the intake command is
   * cut at the marker's timestamp and the drum starts ramping to its stow velocity, so the drum
   * stays off while intaking but is already rolling when the shot after the path spins up.
   */
  private static final String DRUM_STOW_EVENT = "Drum_stow";

  /**
   * Choreo trajectories already parsed from deploy storage. Loading a .traj is disk I/O + JSON
   * parsing (slow on the RIO), so each path is read once per boot no matter how many commands
   * reference it or how often the block auto is rebuilt.
   */
  private static final Map<String, PathPlannerPath> pathCache = new HashMap<>();

  private AutoCommands() {}

  static Command followChoreo(String name) {
    return AutoBuilder.followPath(loadChoreoPath(name));
  }

  public static PathPlannerPath loadChoreoPath(String name) {
    PathPlannerPath cached = pathCache.get(name);
    if (cached != null) {
      return cached;
    }
    try {
      PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(name);
      pathCache.put(name, path);
      return path;
    } catch (Exception e) {
      throw new RuntimeException("Failed to load Choreo trajectory '" + name + "'", e);
    }
  }

  /**
   * Reset odometry to a Choreo path's holonomic start pose, mirrored to the red alliance side when
   * on red (Choreo paths are authored on the blue origin). No-op if the path has no start pose.
   *
   * <p>The alliance check runs when the command executes, not when it is constructed — autos are
   * now pre-built while disabled, possibly before the Driver Station has reported the alliance.
   */
  static Command resetToStart(PathPlannerPath path) {
    Optional<Pose2d> start = path.getStartingHolonomicPose();
    if (start.isEmpty()) {
      return Commands.none();
    }
    Pose2d pose = start.get();
    return Commands.runOnce(
        () -> {
          boolean isRed =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          Drive.mInstance.setPose(isRed ? FlippingUtil.flipFieldPose(pose) : pose);
        },
        Drive.mInstance);
  }

  // Commented out 2026-07-11: unused — kept for reference in case a path+intake race without a
  // trench pass is needed again.
  //
  // /**
  //  * Path followed while the intake deploys and runs the whole time, stopping when the path
  //  * ends.
  //  */
  // static Command pathWithIntake(String name) {
  //   return followChoreo(name).raceWith(new IntakeCommand());
  // }

  /**
   * Autopilot trench pass followed by a path, with the intake deployed and running across the whole
   * span — starting the moment the previous shot ends, so fuel sitting in and around the trench is
   * collected too. The race ends with the path, stopping the roller/indexer.
   */
  static Command trenchThenPathWithIntake(String name) {
    return Commands.sequence(new AutopilotTrenchCommand(name), followChoreo(name))
        .raceWith(new IntakeCommand(100, 500, false));
  }

  /**
   * First path of an auto: outward-zero the intake deploy before moving, reset odometry to the
   * path's start pose, then follow the path while the normal intake command deploys and runs the
   * roller/indexer. The path ends the intake command when it finishes.
   *
   * <p>If the trajectory carries a {@value #DRUM_STOW_EVENT} event marker, the intake command is
   * cut at the marker and the drum starts ramping to its stow velocity from there (see {@link
   * #intakeUntilDrumStow}); without the marker the intake simply runs for the whole path.
   *
   * <p>This zero used to be the path's t=0 "IntakeZeroOut" event marker bound to an EventTrigger,
   * but EventTrigger commands are scheduled on the main scheduler, so the zero's IntakeDeploy
   * requirement conflicted with the deferred block auto (which holds every subsystem) and cancelled
   * the whole auto one cycle after the pose reset. The {@value #DRUM_STOW_EVENT} marker is handled
   * by timestamp inside the composition for the same reason.
   */
  static Command firstPathWithIntake(String name) {
    PathPlannerPath path = loadChoreoPath(name);
    return Commands.sequence(
        resetToStart(path), AutoBuilder.followPath(path).deadlineFor(intakeUntilDrumStow(path)));
  }

  /**
   * The intake-phase command run alongside a path follow (as the non-deadline side, so the path
   * ending always cuts it): intake for the whole path, or — when the path carries a {@value
   * #DRUM_STOW_EVENT} marker — intake only until the marker's timestamp, then set the drum to its
   * stow velocity so it is already rolling when the first shot spins up. The timeout measures from
   * path-follow start, which is exactly how PathPlanner's own event scheduler fires Choreo markers.
   */
  private static Command intakeUntilDrumStow(PathPlannerPath path) {
    Optional<Double> stowTime = choreoEventTime(path, DRUM_STOW_EVENT);
    if (stowTime.isEmpty()) {
      return new IntakeCommand();
    }
    return Commands.sequence(
        new IntakeCommand(60, 400, false).withTimeout(stowTime.get()),
        Commands.runOnce(
            () -> {
              Drum.mInstance.setCurrent(10);
            },
            Drum.mInstance));
  }

  /**
   * Timestamp of the named one-shot event marker in a Choreo trajectory, if present. Only call on
   * paths from {@link #loadChoreoPath}: Choreo paths already carry their ideal trajectory (and its
   * events), so {@code getIdealTrajectory} never consults the RobotConfig argument.
   */
  private static Optional<Double> choreoEventTime(PathPlannerPath path, String eventName) {
    return path.getIdealTrajectory(null)
        .flatMap(
            trajectory ->
                trajectory.getEvents().stream()
                    .filter(
                        event ->
                            event instanceof OneShotTriggerEvent oneShot
                                && oneShot.getEventName().equals(eventName))
                    .findFirst()
                    .map(Event::getTimestampSeconds));
  }

  /** Timed shooting burst: aim + shoot for a fixed duration, then cleanly retract. */
  static Command timedShoot(double duration) {
    return new Shooting(false, () -> 0.0, () -> 0.0).withTimeout(duration);
  }

  /** Runs only the PID test trajectory after resetting odometry to its start pose. */
  public static Command pidTestPathAuto() {
    try {
      PathPlannerPath path = loadChoreoPath(PID_TEST_PATH);
      return Commands.sequence(resetToStart(path), AutoBuilder.followPath(path));
    } catch (RuntimeException e) {
      DriverStation.reportError("PID test path failed to load: " + e.getMessage(), false);
      return Commands.print("PID test path unavailable: missing Choreo trajectory");
    }
  }

  // Commented out 2026-07-11: the Left1_path1/2/3 Choreo trajectories were deleted when the
  // Swap-trajectory BlockAutoBuilder scheme replaced this hand-built auto (commits b9b9530 and
  // 69b2537), so this always failed to load at startup. Restore together with the LEFT_PATH_*
  // constants in AutoCommandsConstants and the "Left Three-Piece" chooser option in
  // RobotContainer if the .traj files are recreated.
  //
  // /**
  //  * Left three-piece auto:
  //  *
  //  * <ol>
  //  *   <li>Reset pose to Left1_path1 start, then follow it immediately; the path's
  //  *       "IntakeDeploy" event marker inward-zeroes the intake deploy while driving, then
  //  *       deploys and intakes.
  //  *   <li>Timed Shooting.
  //  *   <li>Follow Left1_path2 while intaking.
  //  *   <li>Timed Shooting.
  //  *   <li>Follow Left1_path3 while intaking.
  //  * </ol>
  //  */
  // public static Command leftThreePieceAuto() {
  //   try {
  //     return buildLeftThreePieceAuto();
  //   } catch (RuntimeException e) {
  //     // A missing/renamed .traj must never crash the robot program: report it and hand the
  //     // chooser a no-op so everything else (teleop, other autos) still works.
  //     DriverStation.reportError(
  //         "Left three-piece auto failed to load: " + e.getMessage(), false);
  //     return Commands.print("Left three-piece auto unavailable: missing Choreo trajectory");
  //   }
  // }
  //
  // private static Command buildLeftThreePieceAuto() {
  //   return Commands.sequence(
  //       // 1) Reset pose and drive Left1_path1 immediately; its "IntakeDeploy" marker zeroes
  //       //    the intake deploy while driving, then deploys and intakes.
  //       firstPathWithIntake(AutoCommandsConstants.LEFT_PATH_1),
  //
  //       // 2) Timed shooting.
  //       timedShoot(),
  //
  //       // 3) Left1_path2 while intaking.
  //       followChoreo(AutoCommandsConstants.LEFT_PATH_2).raceWith(new IntakeCommand()),
  //
  //       // 4) Timed shooting.
  //       timedShoot(),
  //
  //       // 5) Left1_path3 while intaking.
  //       followChoreo(AutoCommandsConstants.LEFT_PATH_3).raceWith(new IntakeCommand()));
  // }
}
