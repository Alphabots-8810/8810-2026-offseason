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
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
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
    public double motionMagicCruiseVelocityRotPerSec = 0.0;
    public double motionMagicAccelerationRotPerSecSq = 0.0;
    public double motionMagicJerkRotPerSecCubed = 0.0;
    public List<FollowerConfig> followers = new ArrayList<>();

    public MotorIOPhoenix6Config(int canId, String canBus) {
      this.canId = canId;
      this.canBus = canBus;
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
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0);
  private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
  private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
      new PositionTorqueCurrentFOC(0.0);
  private final MotionMagicVoltage motionMagicVoltageRequest = new MotionMagicVoltage(0.0);
  private final MotionMagicTorqueCurrentFOC motionMagicTorqueCurrentRequest =
      new MotionMagicTorqueCurrentFOC(0.0);

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Temperature> tempCelsius;

  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private ControlMode controlMode = ControlMode.DISABLED;
  private double voltageSetpoint = 0.0;
  private double velocitySetpointRadPerSec = 0.0;
  private double positionSetpointRad = 0.0;

  public MotorIOPhoenix6(MotorIOPhoenix6Config config) {
    this.config = config;
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

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVolts, supplyCurrent, statorCurrent, tempCelsius);

    var devices = new ArrayList<ParentDevice>();
    devices.add(talon);
    devices.addAll(followers);
    ParentDevice.optimizeBusUtilizationForAll(devices.toArray(ParentDevice[]::new));
  }

  @Override
  public void updateInputs(MotorIOInputs inputs) {
    var status =
        BaseStatusSignal.refreshAll(
            position, velocity, appliedVolts, supplyCurrent, statorCurrent, tempCelsius);

    inputs.connected = connectedDebounce.calculate(status.isOK());
    inputs.positionRad = Units.rotationsToRadians(position.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
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
    if (config.useMotionMagic) {
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
