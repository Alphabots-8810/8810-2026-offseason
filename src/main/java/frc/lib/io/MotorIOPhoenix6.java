// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.io;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import java.util.ArrayList;
import java.util.List;

public class MotorIOPhoenix6 implements MotorIO {
  public static class MotorIOPhoenix6Config {
    public final int canId;
    public final String canBus;
    public double rotorToMechanismRatio = 1.0;
    public boolean inverted = false;
    public NeutralMode neutralMode = NeutralMode.BRAKE;
    public double supplyCurrentLimitAmps = 40.0;
    public boolean supplyCurrentLimitEnabled = true;
    public double statorCurrentLimitAmps = 80.0;
    public boolean statorCurrentLimitEnabled = true;
    public double kP = 0.0;
    public double kI = 0.0;
    public double kD = 0.0;
    public double kS = 0.0;
    public double kV = 0.0;
    public double kA = 0.0;
    public double kG = 0.0;
    public ClosedLoopOutput closedLoopOutput = ClosedLoopOutput.VOLTAGE;
    public GravityMode gravityMode = GravityMode.NONE;
    public boolean useMotionMagic = false;
    // When true, position moves use Dynamic Motion Magic: the cruise/accel/jerk ride in the control
    // request and can be changed per call via setMotionMagicConstraints(). Requires a CAN FD bus
    // (CANivore); not supported on the roboRIO CAN bus.
    public boolean useDynamicMotionMagic = false;
    public double motionMagicCruiseVelocityRotPerSec = 0.0;
    public double motionMagicAccelerationRotPerSecSq = 0.0;
    public double motionMagicJerkRotPerSecCubed = 0.0;
    public double peakReverseTorqueCurrent = -800;
    public double peakForwardTorqueCurrent = 800;
    public List<FollowerConfig> followers = new ArrayList<>();

    public MotorIOPhoenix6Config(int canId, String canBus) {
      this.canId = canId;
      this.canBus = canBus;
    }

    public MotorIOPhoenix6Config withPeakTorqueLimit(
        double forwardTorqueCurrent, double reverseTorqueCurrent) {
      this.peakForwardTorqueCurrent = forwardTorqueCurrent;
      this.peakReverseTorqueCurrent = reverseTorqueCurrent;
      return this;
    }

    public MotorIOPhoenix6Config withRotorToMechanismRatio(double rotorToMechanismRatio) {
      this.rotorToMechanismRatio = rotorToMechanismRatio;
      return this;
    }

    public MotorIOPhoenix6Config withInverted(boolean inverted) {
      this.inverted = inverted;
      return this;
    }

    public MotorIOPhoenix6Config withNeutralMode(NeutralMode neutralMode) {
      this.neutralMode = neutralMode;
      return this;
    }

    public MotorIOPhoenix6Config withCurrentLimits(
        double supplyCurrentLimitAmps, double statorCurrentLimitAmps) {
      this.supplyCurrentLimitAmps = supplyCurrentLimitAmps;
      this.statorCurrentLimitAmps = statorCurrentLimitAmps;
      return this;
    }

    public MotorIOPhoenix6Config withSlot0(
        double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
      this.kP = kP;
      this.kI = kI;
      this.kD = kD;
      this.kS = kS;
      this.kV = kV;
      this.kA = kA;
      this.kG = kG;
      return this;
    }

    public MotorIOPhoenix6Config withClosedLoopOutput(ClosedLoopOutput closedLoopOutput) {
      this.closedLoopOutput = closedLoopOutput;
      return this;
    }

    public MotorIOPhoenix6Config withGravityMode(GravityMode gravityMode) {
      this.gravityMode = gravityMode;
      return this;
    }

    public MotorIOPhoenix6Config withMotionMagic(
        double cruiseVelocityRotPerSec, double accelerationRotPerSecSq, double jerkRotPerSecCubed) {
      useMotionMagic = true;
      motionMagicCruiseVelocityRotPerSec = cruiseVelocityRotPerSec;
      motionMagicAccelerationRotPerSecSq = accelerationRotPerSecSq;
      motionMagicJerkRotPerSecCubed = jerkRotPerSecCubed;
      return this;
    }

    /**
     * Enables Dynamic Motion Magic. The given values are the initial cruise velocity, acceleration,
     * and jerk; they can be changed at runtime with {@link #setMotionMagicConstraints}. Requires a
     * CAN FD bus (CANivore).
     */
    public MotorIOPhoenix6Config withDynamicMotionMagic(
        double cruiseVelocityRotPerSec, double accelerationRotPerSecSq, double jerkRotPerSecCubed) {
      useMotionMagic = true;
      useDynamicMotionMagic = true;
      motionMagicCruiseVelocityRotPerSec = cruiseVelocityRotPerSec;
      motionMagicAccelerationRotPerSecSq = accelerationRotPerSecSq;
      motionMagicJerkRotPerSecCubed = jerkRotPerSecCubed;
      return this;
    }

    public MotorIOPhoenix6Config withFollower(int id, boolean opposeMasterDirection) {
      followers.add(new FollowerConfig(id, opposeMasterDirection));
      return this;
    }
  }

  public static record FollowerConfig(int id, boolean opposeMasterDirection) {}

  public static enum NeutralMode {
    COAST,
    BRAKE
  }

  public static enum ClosedLoopOutput {
    VOLTAGE,
    TORQUE_CURRENT_FOC
  }

  public static enum GravityMode {
    NONE,
    ELEVATOR_STATIC,
    ARM_COSINE
  }

  private final MotorIOPhoenix6Config config;
  private final TalonFX talon;
  private final List<TalonFX> followers = new ArrayList<>();

  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final TorqueCurrentFOC currentRequest = new TorqueCurrentFOC(0.0);
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0);
  private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
  private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
      new PositionTorqueCurrentFOC(0.0);
  private final MotionMagicVoltage motionMagicVoltageRequest = new MotionMagicVoltage(0.0);
  private final MotionMagicTorqueCurrentFOC motionMagicTorqueCurrentRequest =
      new MotionMagicTorqueCurrentFOC(0.0);
  private final DynamicMotionMagicVoltage dynamicMotionMagicVoltageRequest =
      new DynamicMotionMagicVoltage(0.0, 0.0, 0.0);
  private final DynamicMotionMagicTorqueCurrentFOC dynamicMotionMagicTorqueCurrentRequest =
      new DynamicMotionMagicTorqueCurrentFOC(0.0, 0.0, 0.0);

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Temperature> tempCelsius;
  // Follower currents are polled too: a follower fighting the leader (wrong invert, dragging
  // bearing, brownout) is invisible in the leader's current alone.
  private final List<StatusSignal<Current>> followerSupplyCurrents = new ArrayList<>();
  private final List<StatusSignal<Current>> followerStatorCurrents = new ArrayList<>();
  private final BaseStatusSignal[] allSignals;

  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private ControlMode controlMode = ControlMode.DISABLED;
  private double voltageSetpoint = 0.0;
  private double currentSetpoint = 0.0;
  private double velocitySetpointRadPerSec = 0.0;
  private double positionSetpointRad = 0.0;

  // Live constraints used by Dynamic Motion Magic; changeable at runtime.
  private double dynamicCruiseVelocityRotPerSec;
  private double dynamicAccelerationRotPerSecSq;
  private double dynamicJerkRotPerSecCubed;

  public MotorIOPhoenix6(MotorIOPhoenix6Config config) {
    this.config = config;
    dynamicCruiseVelocityRotPerSec = config.motionMagicCruiseVelocityRotPerSec;
    dynamicAccelerationRotPerSecSq = config.motionMagicAccelerationRotPerSecSq;
    dynamicJerkRotPerSecCubed = config.motionMagicJerkRotPerSecCubed;
    var canBus = createCANBus(config.canBus);
    talon = new TalonFX(config.canId, canBus);

    var talonConfig = new TalonFXConfiguration();
    talonConfig.MotorOutput.NeutralMode = toPhoenixNeutralMode(config.neutralMode);
    talonConfig.MotorOutput.Inverted =
        config.inverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    talonConfig.Feedback.SensorToMechanismRatio = config.rotorToMechanismRatio;
    talonConfig.CurrentLimits.SupplyCurrentLimit = config.supplyCurrentLimitAmps;
    talonConfig.CurrentLimits.SupplyCurrentLimitEnable = config.supplyCurrentLimitEnabled;
    talonConfig.CurrentLimits.StatorCurrentLimit = config.statorCurrentLimitAmps;
    talonConfig.CurrentLimits.StatorCurrentLimitEnable = config.statorCurrentLimitEnabled;
    talonConfig.TorqueCurrent.PeakForwardTorqueCurrent = config.peakForwardTorqueCurrent;
    talonConfig.TorqueCurrent.PeakReverseTorqueCurrent = config.peakReverseTorqueCurrent;
    talonConfig.Slot0.kP = config.kP;
    talonConfig.Slot0.kI = config.kI;
    talonConfig.Slot0.kD = config.kD;
    talonConfig.Slot0.kS = config.kS;
    talonConfig.Slot0.kV = config.kV;
    talonConfig.Slot0.kA = config.kA;
    talonConfig.Slot0.kG = config.kG;
    talonConfig.Slot0.GravityType = toPhoenixGravityMode(config.gravityMode);
    talonConfig.MotionMagic.MotionMagicCruiseVelocity = config.motionMagicCruiseVelocityRotPerSec;
    talonConfig.MotionMagic.MotionMagicAcceleration = config.motionMagicAccelerationRotPerSecSq;
    talonConfig.MotionMagic.MotionMagicJerk = config.motionMagicJerkRotPerSecCubed;
    tryUntilOk(5, () -> talon.getConfigurator().apply(talonConfig, 0.25));

    for (FollowerConfig followerConfig : config.followers) {
      var follower = new TalonFX(followerConfig.id(), canBus);
      tryUntilOk(5, () -> follower.getConfigurator().apply(talonConfig, 0.25));
      follower.setControl(
          new Follower(
              config.canId,
              followerConfig.opposeMasterDirection()
                  ? MotorAlignmentValue.Opposed
                  : MotorAlignmentValue.Aligned));
      followers.add(follower);
    }

    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVolts = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    statorCurrent = talon.getStatorCurrent();
    tempCelsius = talon.getDeviceTemp();
    for (TalonFX follower : followers) {
      followerSupplyCurrents.add(follower.getSupplyCurrent());
      followerStatorCurrents.add(follower.getStatorCurrent());
    }

    var signals =
        new ArrayList<BaseStatusSignal>(
            List.of(position, velocity, appliedVolts, supplyCurrent, statorCurrent, tempCelsius));
    signals.addAll(followerSupplyCurrents);
    signals.addAll(followerStatorCurrents);
    allSignals = signals.toArray(BaseStatusSignal[]::new);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, allSignals);

    var devices = new ArrayList<ParentDevice>();
    devices.add(talon);
    devices.addAll(followers);
    ParentDevice.optimizeBusUtilizationForAll(devices.toArray(ParentDevice[]::new));
  }

  @Override
  public void updateInputs(MotorIOInputs inputs) {
    var status = BaseStatusSignal.refreshAll(allSignals);

    inputs.connected = connectedDebounce.calculate(status.isOK());
    inputs.positionRad = Units.rotationsToRadians(position.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    // Reuse the arrays across cycles (same inputs object every loop) to avoid per-loop garbage
    if (inputs.perMotorSupplyCurrentAmps.length != 1 + followers.size()) {
      inputs.perMotorSupplyCurrentAmps = new double[1 + followers.size()];
      inputs.perMotorStatorCurrentAmps = new double[1 + followers.size()];
    }
    inputs.perMotorSupplyCurrentAmps[0] = inputs.supplyCurrentAmps;
    inputs.perMotorStatorCurrentAmps[0] = inputs.statorCurrentAmps;
    double totalSupply = inputs.supplyCurrentAmps;
    for (int i = 0; i < followers.size(); i++) {
      double followerSupply = followerSupplyCurrents.get(i).getValueAsDouble();
      inputs.perMotorSupplyCurrentAmps[1 + i] = followerSupply;
      inputs.perMotorStatorCurrentAmps[1 + i] = followerStatorCurrents.get(i).getValueAsDouble();
      totalSupply += followerSupply;
    }
    inputs.totalSupplyCurrentAmps = totalSupply;
    inputs.tempCelsius = tempCelsius.getValueAsDouble();
    inputs.controlMode = controlMode;
    inputs.voltageSetpoint = voltageSetpoint;
    inputs.velocitySetpointRadPerSec = velocitySetpointRadPerSec;
    inputs.positionSetpointRad = positionSetpointRad;
  }

  @Override
  public void setVoltage(double volts) {
    controlMode = ControlMode.VOLTAGE;
    voltageSetpoint = volts;
    velocitySetpointRadPerSec = 0.0;
    positionSetpointRad = 0.0;
    talon.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setCurrent(double current) {
    controlMode = ControlMode.CURRENT;
    currentSetpoint = current;
    velocitySetpointRadPerSec = 0.0;
    positionSetpointRad = 0.0;
    talon.setControl(currentRequest.withOutput(current));
  }

  @Override
  public void setVelocity(double velocityRadPerSec) {
    controlMode = ControlMode.VELOCITY;
    voltageSetpoint = 0.0;
    velocitySetpointRadPerSec = velocityRadPerSec;
    positionSetpointRad = 0.0;
    double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
    talon.setControl(
        switch (config.closedLoopOutput) {
          case VOLTAGE -> velocityVoltageRequest.withVelocity(velocityRotPerSec);
          case TORQUE_CURRENT_FOC -> velocityTorqueCurrentRequest.withVelocity(velocityRotPerSec);
        });
  }

  @Override
  public void setPosition(double positionRad) {
    controlMode = ControlMode.POSITION;
    voltageSetpoint = 0.0;
    velocitySetpointRadPerSec = 0.0;
    positionSetpointRad = positionRad;
    double positionRotations = Units.radiansToRotations(positionRad);
    if (config.useDynamicMotionMagic) {
      talon.setControl(dynamicPositionRequest(positionRotations));
    } else if (config.useMotionMagic) {
      talon.setControl(
          switch (config.closedLoopOutput) {
            case VOLTAGE -> motionMagicVoltageRequest.withPosition(positionRotations);
            case TORQUE_CURRENT_FOC -> motionMagicTorqueCurrentRequest.withPosition(
                positionRotations);
          });
    } else {
      talon.setControl(
          switch (config.closedLoopOutput) {
            case VOLTAGE -> positionVoltageRequest.withPosition(positionRotations);
            case TORQUE_CURRENT_FOC -> positionTorqueCurrentRequest.withPosition(positionRotations);
          });
    }
  }

  @Override
  public void setNeutralMode(boolean brake) {
    talon.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    for (TalonFX follower : followers) {
      follower.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }
  }

  @Override
  public void setEncoderPosition(double positionRad) {
    tryUntilOk(5, () -> talon.setPosition(Units.radiansToRotations(positionRad), 0.25));
  }

  @Override
  public void stop() {
    controlMode = ControlMode.DISABLED;
    voltageSetpoint = 0.0;
    velocitySetpointRadPerSec = 0.0;
    positionSetpointRad = 0.0;
    talon.setControl(voltageRequest.withOutput(0.0));
  }

  @Override
  public double getPositionRot() {
    return position.getValueAsDouble();
  }

  @Override
  public double getVelocityRotPerSec() {
    return velocity.getValueAsDouble();
  }

  @Override
  public void setVelocityRotPerSec(double velocityRotPerSec) {
    controlMode = ControlMode.VELOCITY;
    voltageSetpoint = 0.0;
    velocitySetpointRadPerSec = Units.rotationsToRadians(velocityRotPerSec);
    positionSetpointRad = 0.0;
    talon.setControl(
        switch (config.closedLoopOutput) {
          case VOLTAGE -> velocityVoltageRequest.withVelocity(velocityRotPerSec);
          case TORQUE_CURRENT_FOC -> velocityTorqueCurrentRequest.withVelocity(velocityRotPerSec);
        });
  }

  @Override
  public void setPositionRot(double positionRot) {
    controlMode = ControlMode.POSITION;
    voltageSetpoint = 0.0;
    velocitySetpointRadPerSec = 0.0;
    positionSetpointRad = Units.rotationsToRadians(positionRot);
    if (config.useDynamicMotionMagic) {
      talon.setControl(dynamicPositionRequest(positionRot));
    } else if (config.useMotionMagic) {
      talon.setControl(
          switch (config.closedLoopOutput) {
            case VOLTAGE -> motionMagicVoltageRequest.withPosition(positionRot);
            case TORQUE_CURRENT_FOC -> motionMagicTorqueCurrentRequest.withPosition(positionRot);
          });
    } else {
      talon.setControl(
          switch (config.closedLoopOutput) {
            case VOLTAGE -> positionVoltageRequest.withPosition(positionRot);
            case TORQUE_CURRENT_FOC -> positionTorqueCurrentRequest.withPosition(positionRot);
          });
    }
  }

  /** Builds a Dynamic Motion Magic request to the given position using the live constraints. */
  private ControlRequest dynamicPositionRequest(double positionRot) {
    return switch (config.closedLoopOutput) {
      case VOLTAGE -> dynamicMotionMagicVoltageRequest
          .withPosition(positionRot)
          .withVelocity(dynamicCruiseVelocityRotPerSec)
          .withAcceleration(dynamicAccelerationRotPerSecSq)
          .withJerk(dynamicJerkRotPerSecCubed);
      case TORQUE_CURRENT_FOC -> dynamicMotionMagicTorqueCurrentRequest
          .withPosition(positionRot)
          .withVelocity(dynamicCruiseVelocityRotPerSec)
          .withAcceleration(dynamicAccelerationRotPerSecSq)
          .withJerk(dynamicJerkRotPerSecCubed);
    };
  }

  @Override
  public void setMotionMagicConstraints(
      double cruiseVelocityRotPerSec, double accelerationRotPerSecSq, double jerkRotPerSecCubed) {
    dynamicCruiseVelocityRotPerSec = cruiseVelocityRotPerSec;
    dynamicAccelerationRotPerSecSq = accelerationRotPerSecSq;
    dynamicJerkRotPerSecCubed = jerkRotPerSecCubed;
  }

  @Override
  public void setEncoderPositionRot(double positionRot) {
    tryUntilOk(5, () -> talon.setPosition(positionRot, 0.25));
  }

  private static NeutralModeValue toPhoenixNeutralMode(NeutralMode neutralMode) {
    return switch (neutralMode) {
      case COAST -> NeutralModeValue.Coast;
      case BRAKE -> NeutralModeValue.Brake;
    };
  }

  private static GravityTypeValue toPhoenixGravityMode(GravityMode gravityMode) {
    return switch (gravityMode) {
      case NONE -> GravityTypeValue.Elevator_Static;
      case ELEVATOR_STATIC -> GravityTypeValue.Elevator_Static;
      case ARM_COSINE -> GravityTypeValue.Arm_Cosine;
    };
  }

  private static CANBus createCANBus(String canBus) {
    return canBus.isBlank() ? CANBus.roboRIO() : new CANBus(canBus);
  }
}
