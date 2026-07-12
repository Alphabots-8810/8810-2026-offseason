// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.io;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class MotorIOSim implements MotorIO {
  public static class MotorIOSimConfig {
    public DCMotor gearbox = DCMotor.getKrakenX60Foc(1);
    public double gearing = 1.0;
    public double momentOfInertiaKgMetersSquared = 0.01;
    public double velocityKp = 0.1;
    public double velocityKd = 0.0;
    public double positionKp = 4.0;
    public double positionKd = 0.0;
    public boolean instantSetpoint = false;

    public MotorIOSimConfig withGearbox(DCMotor gearbox) {
      this.gearbox = gearbox;
      return this;
    }

    public MotorIOSimConfig withGearing(double gearing) {
      this.gearing = gearing;
      return this;
    }

    public MotorIOSimConfig withMomentOfInertia(double momentOfInertiaKgMetersSquared) {
      this.momentOfInertiaKgMetersSquared = momentOfInertiaKgMetersSquared;
      return this;
    }

    public MotorIOSimConfig withVelocityGains(double kP, double kD) {
      velocityKp = kP;
      velocityKd = kD;
      return this;
    }

    public MotorIOSimConfig withPositionGains(double kP, double kD) {
      positionKp = kP;
      positionKd = kD;
      return this;
    }

    /** Snaps measured state to the active setpoint each loop instead of running plant physics. */
    public MotorIOSimConfig withInstantSetpoint(boolean instantSetpoint) {
      this.instantSetpoint = instantSetpoint;
      return this;
    }
  }

  private final DCMotorSim sim;
  private final PIDController velocityController;
  private final PIDController positionController;
  private final boolean instantSetpoint;

  private boolean velocityClosedLoop = false;
  private boolean positionClosedLoop = false;
  private double appliedVolts = 0.0;

  private ControlMode controlMode = ControlMode.DISABLED;
  private double voltageSetpoint = 0.0;
  private double velocitySetpointRadPerSec = 0.0;
  private double positionSetpointRad = 0.0;

  public MotorIOSim(MotorIOSimConfig config) {
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                config.gearbox, config.momentOfInertiaKgMetersSquared, config.gearing),
            config.gearbox);
    velocityController = new PIDController(config.velocityKp, 0.0, config.velocityKd);
    positionController = new PIDController(config.positionKp, 0.0, config.positionKd);
    instantSetpoint = config.instantSetpoint;
  }

  @Override
  public void updateInputs(MotorIOInputs inputs) {
    if (instantSetpoint) {
      if (velocityClosedLoop) {
        sim.setState(sim.getAngularPositionRad(), velocitySetpointRadPerSec);
        appliedVolts = 0.0;
      } else if (positionClosedLoop) {
        sim.setState(positionSetpointRad, 0.0);
        appliedVolts = 0.0;
      } else if (controlMode == ControlMode.VOLTAGE) {
        sim.setState(sim.getAngularPositionRad(), 0.0);
        appliedVolts = voltageSetpoint;
      } else {
        sim.setState(sim.getAngularPositionRad(), 0.0);
        appliedVolts = 0.0;
      }
    } else {
      if (velocityClosedLoop) {
        appliedVolts = velocityController.calculate(sim.getAngularVelocityRadPerSec());
      } else {
        velocityController.reset();
      }

      if (positionClosedLoop) {
        appliedVolts = positionController.calculate(sim.getAngularPositionRad());
      } else {
        positionController.reset();
      }

      sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
      sim.update(0.02);
    }

    inputs.connected = true;
    inputs.positionRad = sim.getAngularPositionRad();
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrentAmps = Math.abs(sim.getCurrentDrawAmps());
    inputs.statorCurrentAmps = Math.abs(sim.getCurrentDrawAmps());
    inputs.totalSupplyCurrentAmps = inputs.supplyCurrentAmps;
    inputs.perMotorSupplyCurrentAmps = new double[] {inputs.supplyCurrentAmps};
    inputs.perMotorStatorCurrentAmps = new double[] {inputs.statorCurrentAmps};
    inputs.tempCelsius = 25.0;
    inputs.controlMode = controlMode;
    inputs.voltageSetpoint = voltageSetpoint;
    inputs.velocitySetpointRadPerSec = velocitySetpointRadPerSec;
    inputs.positionSetpointRad = positionSetpointRad;
  }

  @Override
  public void setVoltage(double volts) {
    velocityClosedLoop = false;
    positionClosedLoop = false;
    appliedVolts = volts;
    controlMode = ControlMode.VOLTAGE;
    voltageSetpoint = volts;
    velocitySetpointRadPerSec = 0.0;
    positionSetpointRad = 0.0;
  }

  @Override
  public void setVelocity(double velocityRadPerSec) {
    velocityClosedLoop = true;
    positionClosedLoop = false;
    velocityController.setSetpoint(velocityRadPerSec);
    controlMode = ControlMode.VELOCITY;
    voltageSetpoint = 0.0;
    velocitySetpointRadPerSec = velocityRadPerSec;
    positionSetpointRad = 0.0;
  }

  @Override
  public void setPosition(double positionRad) {
    velocityClosedLoop = false;
    positionClosedLoop = true;
    positionController.setSetpoint(positionRad);
    controlMode = ControlMode.POSITION;
    voltageSetpoint = 0.0;
    velocitySetpointRadPerSec = 0.0;
    positionSetpointRad = positionRad;
  }

  @Override
  public void setEncoderPosition(double positionRad) {
    sim.setState(positionRad, 0.0);
  }

  @Override
  public void stop() {
    velocityClosedLoop = false;
    positionClosedLoop = false;
    appliedVolts = 0.0;
    controlMode = ControlMode.DISABLED;
    voltageSetpoint = 0.0;
    velocitySetpointRadPerSec = 0.0;
    positionSetpointRad = 0.0;
  }

  @Override
  public double getPositionRot() {
    return sim.getAngularPositionRad() / (2.0 * Math.PI);
  }

  @Override
  public double getVelocityRotPerSec() {
    return sim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
  }

  @Override
  public void setVelocityRotPerSec(double velocityRotPerSec) {
    velocityClosedLoop = true;
    positionClosedLoop = false;
    double velocityRadPerSec = velocityRotPerSec * 2.0 * Math.PI;
    velocityController.setSetpoint(velocityRadPerSec);
    controlMode = ControlMode.VELOCITY;
    voltageSetpoint = 0.0;
    velocitySetpointRadPerSec = velocityRadPerSec;
    positionSetpointRad = 0.0;
  }

  @Override
  public void setPositionRot(double positionRot) {
    velocityClosedLoop = false;
    positionClosedLoop = true;
    double positionRad = positionRot * 2.0 * Math.PI;
    positionController.setSetpoint(positionRad);
    controlMode = ControlMode.POSITION;
    voltageSetpoint = 0.0;
    velocitySetpointRadPerSec = 0.0;
    positionSetpointRad = positionRad;
  }

  @Override
  public void setEncoderPositionRot(double positionRot) {
    sim.setState(positionRot * 2.0 * Math.PI, 0.0);
  }
}
