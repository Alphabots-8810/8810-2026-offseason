package frc.robot.simulation;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.simulation.FuelShooterSimulation;
import frc.lib.simulation.GamePieceSimulation;
import frc.robot.commands.ShootingCommand.ShootingConstants;
import frc.robot.subsystems.Drum.Drum;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.Hood.Hood;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

/**
 * Maple-sim fuel intake and shooter glue for this robot. Follows the official intake + projectile
 * pattern via {@link GamePieceSimulation}; configured once from {@link RobotContainer} in SIM mode.
 */
public final class FuelSimulation {
  private static FuelSimulation instance;

  private final GamePieceSimulation gamePieceSimulation;
  private final FuelShooterSimulation fuelShooterSimulation;

  private double lastShooterUpdateSec = Timer.getFPGATimestamp();

  private FuelSimulation(SwerveDriveSimulation driveSimulation) {
    gamePieceSimulation =
        new GamePieceSimulation(
            driveSimulation, Meters.of(0.7), Meters.of(0.25), IntakeSide.BACK, 20);
    fuelShooterSimulation =
        new FuelShooterSimulation(
            gamePieceSimulation,
            new Translation2d(ShootingConstants.EXIT_FORWARD_OFFSET_M, 0.0),
            Meters.of(ShootingConstants.EXIT_HEIGHT_M),
            ShootingConstants.FEEDER_SURFACE_M_PER_MOTOR_ROT,
            ShootingConstants.FEED_BALL_SPACING_M,
            () -> ShootingConstants.SimLaunchSpeedPerDrumRotps.getAsDouble(),
            () -> ShootingConstants.SimLaunchAngleOffsetDeg.getAsDouble());
  }

  public static void configure(SwerveDriveSimulation driveSimulation) {
    instance = new FuelSimulation(driveSimulation);
  }

  public static FuelSimulation getInstance() {
    return instance;
  }

  public static void setIntakeRunning(boolean running) {
    FuelSimulation fuelSimulation = getInstance();
    if (fuelSimulation == null) {
      return;
    }
    if (running) {
      fuelSimulation.gamePieceSimulation.startIntake();
    } else {
      fuelSimulation.gamePieceSimulation.stopIntake();
    }
  }

  public void reset() {
    gamePieceSimulation.stopIntake();
    gamePieceSimulation.clearIntake();
    fuelShooterSimulation.reset();
    lastShooterUpdateSec = Timer.getFPGATimestamp();
  }

  public void simulationPeriodic() {
    double nowSec = Timer.getFPGATimestamp();
    double dtSec = nowSec - lastShooterUpdateSec;
    lastShooterUpdateSec = nowSec;

    if (dtSec > 0.0 && Drum.mInstance != null) {
      fuelShooterSimulation.periodic(
          Drum.mInstance.getVelocitySetpointRotPerSec(),
          Hood.mInstance.getPositionSetpointRot() * 360.0,
          Feeder.mInstance.getVelocitySetpointRotPerSec(),
          dtSec);
    }

    Logger.recordOutput("FieldSimulation/FuelInIntake", gamePieceSimulation.getGamePiecesAmount());
    Logger.recordOutput("FieldSimulation/FuelIntakeRunning", gamePieceSimulation.isIntakeRunning());
  }
}
