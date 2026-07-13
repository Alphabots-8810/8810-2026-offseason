package frc.lib.simulation;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Simulates feeding fuel into the shooter while the feeder runs. Each time the feeder advances by
 * {@code ballSpacingM}, one launch is attempted via {@link GamePieceSimulation#tryLaunchFuel},
 * which only succeeds when intake stock is available.
 */
public class FuelShooterSimulation {
  private static final double MIN_FEEDER_SETPOINT_ROTPS = 5.0;

  private final GamePieceSimulation gamePieceSimulation;
  private final Translation2d shooterOffsetOnRobot;
  private final Distance exitHeight;
  private final double feederSurfaceMPerRot;
  private final double ballSpacingM;
  private final DoubleSupplier launchSpeedPerDrumRotps;
  private final DoubleSupplier launchAngleOffsetDeg;

  private double feederTravelAccumulatorM = 0.0;

  public FuelShooterSimulation(
      GamePieceSimulation gamePieceSimulation,
      Translation2d shooterOffsetOnRobot,
      Distance exitHeight,
      double feederSurfaceMPerRot,
      double ballSpacingM,
      DoubleSupplier launchSpeedPerDrumRotps,
      DoubleSupplier launchAngleOffsetDeg) {
    this.gamePieceSimulation = gamePieceSimulation;
    this.shooterOffsetOnRobot = shooterOffsetOnRobot;
    this.exitHeight = exitHeight;
    this.feederSurfaceMPerRot = feederSurfaceMPerRot;
    this.ballSpacingM = ballSpacingM;
    this.launchSpeedPerDrumRotps = launchSpeedPerDrumRotps;
    this.launchAngleOffsetDeg = launchAngleOffsetDeg;
  }

  public void reset() {
    feederTravelAccumulatorM = 0.0;
  }

  /**
   * @param drumTargetRotps commanded drum motor speed (rot/s)
   * @param hoodTargetDeg commanded hood mechanism angle (deg)
   * @param feederTargetRotps commanded feeder motor speed (rot/s)
   * @param dtSec loop period in seconds
   */
  public void periodic(
      double drumTargetRotps, double hoodTargetDeg, double feederTargetRotps, double dtSec) {
    if (Math.abs(feederTargetRotps) < MIN_FEEDER_SETPOINT_ROTPS) {
      feederTravelAccumulatorM = 0.0;
      return;
    }

    feederTravelAccumulatorM += Math.abs(feederTargetRotps) * feederSurfaceMPerRot * dtSec;

    int launchedCount = 0;
    int rejectedCount = 0;
    while (feederTravelAccumulatorM >= ballSpacingM) {
      if (tryLaunchFuel(drumTargetRotps, hoodTargetDeg)) {
        feederTravelAccumulatorM -= ballSpacingM;
        launchedCount++;
      } else {
        rejectedCount++;
        break;
      }
    }

    Logger.recordOutput("FieldSimulation/FuelLaunchedThisLoop", launchedCount);
    Logger.recordOutput("FieldSimulation/FuelLaunchRejectedThisLoop", rejectedCount);
    Logger.recordOutput("FieldSimulation/FeederTravelAccumulatorM", feederTravelAccumulatorM);
    Logger.recordOutput("FieldSimulation/FuelStock", gamePieceSimulation.getGamePiecesAmount());
  }

  private boolean tryLaunchFuel(double drumTargetRotps, double hoodTargetDeg) {
    double launchSpeedMps = Math.abs(drumTargetRotps) * launchSpeedPerDrumRotps.getAsDouble();
    double launchAngleDeg = 90.0 - hoodTargetDeg - launchAngleOffsetDeg.getAsDouble();

    boolean launched =
        gamePieceSimulation.tryLaunchFuel(
            shooterOffsetOnRobot,
            exitHeight,
            MetersPerSecond.of(launchSpeedMps),
            Degrees.of(launchAngleDeg));

    if (launched) {
      Logger.recordOutput("FieldSimulation/LastLaunchSpeedMps", launchSpeedMps);
      Logger.recordOutput("FieldSimulation/LastLaunchElevationDeg", launchAngleDeg);
      Logger.recordOutput("FieldSimulation/LastDrumTargetRotps", drumTargetRotps);
      Logger.recordOutput("FieldSimulation/LastHoodTargetDeg", hoodTargetDeg);
    }

    return launched;
  }
}
