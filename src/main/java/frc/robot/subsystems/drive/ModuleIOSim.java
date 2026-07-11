// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

/** Physics sim implementation of module IO backed by MapleSim swerve module physics. */
public class ModuleIOSim implements ModuleIO {
  private static final double DRIVE_KP = 0.05;
  private static final double DRIVE_KD = 0.0;
  private static final double DRIVE_KS = 0.0;
  private static final double DRIVE_KV = 0.91035 / (2.0 * Math.PI);
  private static final double TURN_KP = 6.0;
  private static final double TURN_KD = 0.15;

  private final SwerveModuleSimulation moduleSimulation;
  private final SimulatedMotorController.GenericMotorController driveMotor;
  private final SimulatedMotorController.GenericMotorController turnMotor;

  private final PIDController driveController = new PIDController(DRIVE_KP, 0.0, DRIVE_KD);
  private final PIDController turnController = new PIDController(TURN_KP, 0.0, TURN_KD);

  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;
  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public ModuleIOSim(SwerveModuleSimulation moduleSimulation) {
    this.moduleSimulation = moduleSimulation;
    this.driveMotor =
        moduleSimulation.useGenericMotorControllerForDrive().withCurrentLimit(Amps.of(120));
    this.turnMotor = moduleSimulation.useGenericControllerForSteer().withCurrentLimit(Amps.of(60));

    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    double measuredDriveVelocity =
        finiteOrZero(moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond));
    Rotation2d measuredTurnPosition = finiteOrZero(moduleSimulation.getSteerAbsoluteFacing());

    if (driveClosedLoop) {
      driveAppliedVolts = driveFFVolts + driveController.calculate(measuredDriveVelocity);
    } else {
      driveController.reset();
    }

    if (turnClosedLoop) {
      turnAppliedVolts = turnController.calculate(measuredTurnPosition.getRadians());
    } else {
      turnController.reset();
    }

    driveMotor.requestVoltage(Volts.of(clampVoltage(driveAppliedVolts)));
    turnMotor.requestVoltage(Volts.of(clampVoltage(turnAppliedVolts)));

    inputs.driveConnected = true;
    inputs.drivePositionRad =
        finiteOrZero(moduleSimulation.getDriveWheelFinalPosition().in(Radians));
    inputs.driveVelocityRadPerSec = measuredDriveVelocity;
    inputs.driveAppliedVolts =
        finiteOrZero(moduleSimulation.getDriveMotorAppliedVoltage().in(Volts));
    inputs.driveCurrentAmps =
        Math.abs(finiteOrZero(moduleSimulation.getDriveMotorSupplyCurrent().in(Amps)));
    inputs.driveSupplyCurrentAmps = inputs.driveCurrentAmps;

    inputs.turnConnected = true;
    inputs.turnEncoderConnected = true;
    inputs.turnAbsolutePosition = measuredTurnPosition;
    inputs.turnPosition = measuredTurnPosition;
    inputs.turnVelocityRadPerSec =
        finiteOrZero(moduleSimulation.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond));
    inputs.turnAppliedVolts =
        finiteOrZero(moduleSimulation.getSteerMotorAppliedVoltage().in(Volts));
    inputs.turnCurrentAmps =
        Math.abs(finiteOrZero(moduleSimulation.getSteerMotorSupplyCurrent().in(Amps)));
    inputs.turnSupplyCurrentAmps = inputs.turnCurrentAmps;

    Angle[] cachedDrivePositions = moduleSimulation.getCachedDriveWheelFinalPositions();
    inputs.odometryTimestamps = makeOdometryTimestamps(cachedDrivePositions.length);
    inputs.odometryDrivePositionsRad = new double[cachedDrivePositions.length];
    for (int i = 0; i < cachedDrivePositions.length; i++) {
      inputs.odometryDrivePositionsRad[i] = finiteOrZero(cachedDrivePositions[i].in(Radians));
    }

    Rotation2d[] cachedTurnPositions = moduleSimulation.getCachedSteerAbsolutePositions();
    inputs.odometryTurnPositions = new Rotation2d[cachedTurnPositions.length];
    for (int i = 0; i < cachedTurnPositions.length; i++) {
      inputs.odometryTurnPositions[i] = finiteOrZero(cachedTurnPositions[i]);
    }
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveClosedLoop = false;
    driveAppliedVolts = finiteOrZero(output);
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnClosedLoop = false;
    turnAppliedVolts = finiteOrZero(output);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double safeVelocityRadPerSec = finiteOrZero(velocityRadPerSec);
    driveClosedLoop = true;
    driveFFVolts = DRIVE_KS * Math.signum(safeVelocityRadPerSec) + DRIVE_KV * safeVelocityRadPerSec;
    driveController.setSetpoint(safeVelocityRadPerSec);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnClosedLoop = true;
    turnController.setSetpoint(finiteOrZero(rotation).getRadians());
  }

  private static double[] makeOdometryTimestamps(int sampleCount) {
    double[] timestamps = new double[sampleCount];
    double now = Timer.getFPGATimestamp();
    double dt = SimulatedArena.getSimulationDt().in(Seconds);
    for (int i = 0; i < sampleCount; i++) {
      timestamps[i] = now - (sampleCount - 1 - i) * dt;
    }
    return timestamps;
  }

  private static double clampVoltage(double volts) {
    return MathUtil.clamp(finiteOrZero(volts), -12.0, 12.0);
  }

  private static double finiteOrZero(double value) {
    return Double.isFinite(value) ? value : 0.0;
  }

  private static Rotation2d finiteOrZero(Rotation2d rotation) {
    return Double.isFinite(rotation.getRadians()) ? rotation : Rotation2d.kZero;
  }
}
