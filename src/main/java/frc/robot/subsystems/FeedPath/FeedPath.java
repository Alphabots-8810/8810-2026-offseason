package frc.robot.subsystems.FeedPath;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.SensorIO;
import frc.lib.io.SensorIOCANRange;
import frc.lib.io.SensorIOCANRange.SensorIOCANRangeConfig;
import frc.lib.io.SensorIOInputsAutoLogged;
import frc.lib.io.SensorIOSim;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class FeedPath extends SubsystemBase {
  public static final FeedPath mInstance = new FeedPath();

  private final SensorIO hopperIO;
  private final SensorIOInputsAutoLogged hopperInputs = new SensorIOInputsAutoLogged();

  private final SensorIO ballTunnelIO;
  private final SensorIOInputsAutoLogged ballTunnelInputs = new SensorIOInputsAutoLogged();

  private FeedPath() {
    if (Constants.currentMode == Constants.Mode.REAL) {
      hopperIO =
          new SensorIOCANRange(
              new SensorIOCANRangeConfig(FeedPathConstants.HOPPER_CAN_ID, FeedPathConstants.CAN_BUS)
                  .withMaxDistance(FeedPathConstants.HOPPER_MAX_DISTANCE_M)
                  .withMinSignalStrength(FeedPathConstants.MIN_SIGNAL_STRENGTH)
                  .withProximityThreshold(
                      FeedPathConstants.HOPPER_PROXIMITY_THRESHOLD_M,
                      FeedPathConstants.HOPPER_PROXIMITY_HYSTERESIS_M)
                  .withFov(
                      0,
                      0,
                      FeedPathConstants.HOPPER_FOV_RANGE_X_DEG,
                      FeedPathConstants.HOPPER_FOV_RANGE_Y_DEG),
              FeedPathConstants.HOPPER_DEBOUNCE_SECS);

      ballTunnelIO =
          new SensorIOCANRange(
              new SensorIOCANRangeConfig(
                      FeedPathConstants.INDEXER_CAN_ID, FeedPathConstants.CAN_BUS)
                  .withMaxDistance(FeedPathConstants.INDEXER_MAX_DISTANCE_M)
                  .withMinSignalStrength(FeedPathConstants.MIN_SIGNAL_STRENGTH)
                  .withProximityThreshold(
                      FeedPathConstants.HOPPER_PROXIMITY_THRESHOLD_M,
                      FeedPathConstants.HOPPER_PROXIMITY_HYSTERESIS_M)
                  .withFov(
                      0,
                      0,
                      FeedPathConstants.INDEXER_FOV_RANGE_X_DEG,
                      FeedPathConstants.INDEXER_FOV_RANGE_Y_DEG),
              FeedPathConstants.INDEXER_DEBOUNCE_SECS);
    } else {
      hopperIO = new SensorIOSim(() -> false, FeedPathConstants.HOPPER_DEBOUNCE_SECS);
      ballTunnelIO = new SensorIOSim(() -> false, FeedPathConstants.INDEXER_DEBOUNCE_SECS);
    }
  }

  @Override
  public void periodic() {
    hopperIO.updateInputs(hopperInputs);
    ballTunnelIO.updateInputs(ballTunnelInputs);
    Logger.processInputs("FeedPath/Hopper", hopperInputs);
    Logger.processInputs("FeedPath/BallTunnel", ballTunnelInputs);
  }

  /** True when a game piece is staged at the indexer sensor. */
  public boolean IndexerFilled() {
    return ballTunnelInputs.isTriggered;
  }

  /** True when a game piece is detected at the hopper sensor. */
  public boolean HopperFilled() {
    return hopperInputs.isTriggered;
  }
}
