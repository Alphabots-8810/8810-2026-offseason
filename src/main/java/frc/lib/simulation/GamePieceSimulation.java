package frc.lib.simulation;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import java.util.List;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

/**
 * Maple-sim fuel intake and shooter simulation. Follows the official pattern: {@link
 * IntakeSimulation} holds collected fuel, and {@link #tryLaunchFuel} only spawns a projectile after
 * {@link IntakeSimulation#obtainGamePieceFromIntake()} succeeds.
 */
public class GamePieceSimulation {
  private final SwerveDriveSimulation driveSimulation;
  private final IntakeSimulation intakeSimulation;

  public GamePieceSimulation(SwerveDriveSimulation driveSimulation) {
    this(driveSimulation, Meters.of(0.5), Meters.of(0.15), IntakeSide.FRONT, 50);
  }

  public GamePieceSimulation(
      SwerveDriveSimulation driveSimulation,
      Distance intakeWidth,
      Distance intakeExtension,
      IntakeSide intakeSide,
      int intakeCapacity) {
    this.driveSimulation = driveSimulation;
    intakeSimulation =
        IntakeSimulation.OverTheBumperIntake(
            "Fuel", driveSimulation, intakeWidth, intakeExtension, intakeSide, intakeCapacity);
  }

  public void startIntake() {
    intakeSimulation.startIntake();
  }

  public void stopIntake() {
    intakeSimulation.stopIntake();
  }

  public int getGamePiecesAmount() {
    return intakeSimulation.getGamePiecesAmount();
  }

  public boolean isIntakeRunning() {
    return intakeSimulation.isRunning();
  }

  public void clearIntake() {
    intakeSimulation.setGamePiecesCount(0);
  }

  public boolean hasGamePiece() {
    return intakeSimulation.getGamePiecesAmount() > 0;
  }

  /**
   * Consumes one fuel from intake stock and launches it. Returns false when the robot has no fuel
   * stored (official maple-sim shooter gate).
   */
  public boolean tryLaunchFuel(
      Translation2d shooterOffsetOnRobot,
      Distance initialHeight,
      LinearVelocity launchingSpeed,
      Angle shooterAngle) {
    if (!intakeSimulation.obtainGamePieceFromIntake()) {
      Logger.recordOutput("FieldSimulation/FuelLaunchRejectedNoStock", true);
      return false;
    }

    Logger.recordOutput("FieldSimulation/FuelLaunchRejectedNoStock", false);

    var robotPose = driveSimulation.getSimulatedDriveTrainPose();
    RebuiltFuelOnFly fuel =
        new RebuiltFuelOnFly(
            robotPose.getTranslation(),
            shooterOffsetOnRobot,
            driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
            robotPose.getRotation(),
            initialHeight,
            launchingSpeed,
            shooterAngle);

    fuel.withProjectileTrajectoryDisplayCallBack(
        (List<Pose3d> poses) ->
            Logger.recordOutput("Shooter/Trajectory_Hit", poses.toArray(Pose3d[]::new)),
        (List<Pose3d> poses) ->
            Logger.recordOutput("Shooter/Trajectory_Miss", poses.toArray(Pose3d[]::new)));

    SimulatedArena.getInstance().addGamePieceProjectile(fuel);
    return true;
  }

  public Pose3d[] getFuelPoses() {
    return SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel");
  }
}
