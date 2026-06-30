package frc.robot.subsystems.BallSensors;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.SensorIO;
import frc.lib.io.SensorIOCANRange;
import frc.lib.io.SensorIOCANRange.SensorIOCANRangeConfig;
import frc.lib.io.SensorIOInputsAutoLogged;
import frc.lib.io.SensorIOSim;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class BallSensors extends SubsystemBase {
  public static final BallSensors mInstance = new BallSensors();

  private final SensorIO hopperIO;
  private final SensorIOInputsAutoLogged hopperInputs = new SensorIOInputsAutoLogged();

  private final SensorIO ballTunnelIO;
  private final SensorIOInputsAutoLogged ballTunnelInputs = new SensorIOInputsAutoLogged();

  private BallSensors() {
    if (Constants.currentMode == Constants.Mode.REAL) {
      hopperIO =
          new SensorIOCANRange(
              new SensorIOCANRangeConfig(
                      BallSensorsConstants.HOPPER_CAN_ID, BallSensorsConstants.CAN_BUS)
                  .withMaxDistance(BallSensorsConstants.HOPPER_MAX_DISTANCE_M)
                  .withMinSignalStrength(BallSensorsConstants.MIN_SIGNAL_STRENGTH)
                  .withProximityThreshold(
                      BallSensorsConstants.PROXIMITY_THRESHOLD_M,
                      BallSensorsConstants.PROXIMITY_HYSTERESIS_M)
                  .withFov(
                      0, 0,
                      BallSensorsConstants.HOPPER_FOV_RANGE_X_DEG,
                      BallSensorsConstants.HOPPER_FOV_RANGE_Y_DEG),
              BallSensorsConstants.HOPPER_DEBOUNCE_SECS);

      ballTunnelIO =
          new SensorIOCANRange(
              new SensorIOCANRangeConfig(
                      BallSensorsConstants.BALL_TUNNEL_CAN_ID, BallSensorsConstants.CAN_BUS)
                  .withMaxDistance(BallSensorsConstants.BALL_TUNNEL_MAX_DISTANCE_M)
                  .withMinSignalStrength(BallSensorsConstants.MIN_SIGNAL_STRENGTH)
                  .withProximityThreshold(
                      BallSensorsConstants.PROXIMITY_THRESHOLD_M,
                      BallSensorsConstants.PROXIMITY_HYSTERESIS_M)
                  .withFov(
                      0, 0,
                      BallSensorsConstants.BALL_TUNNEL_FOV_RANGE_X_DEG,
                      BallSensorsConstants.BALL_TUNNEL_FOV_RANGE_Y_DEG),
              BallSensorsConstants.BALL_TUNNEL_DEBOUNCE_SECS);
    } else {
      hopperIO = new SensorIOSim(() -> false, BallSensorsConstants.HOPPER_DEBOUNCE_SECS);
      ballTunnelIO = new SensorIOSim(() -> false, BallSensorsConstants.BALL_TUNNEL_DEBOUNCE_SECS);
    }
  }

  @Override
  public void periodic() {
    hopperIO.updateInputs(hopperInputs);
    ballTunnelIO.updateInputs(ballTunnelInputs);
    Logger.processInputs("BallSensors/Hopper", hopperInputs);
    Logger.processInputs("BallSensors/BallTunnel", ballTunnelInputs);
  }

  /** 球已到达通道并稳定（防抖后）。射击 Command 用此判断是否有球可发。 */
  public boolean hasBallInTunnel() {
    return ballTunnelInputs.isTriggered;
  }

  /** 球正在进入料斗（防抖后）。可用于提前启动 Indexer。 */
  public boolean hasBallInHopper() {
    return hopperInputs.isTriggered;
  }
}
