// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.GyroSimulation;

/** Gyro IO implementation backed by MapleSim's drivetrain gyro simulation. */
public class GyroIOSim implements GyroIO {
  private final GyroSimulation gyroSimulation;

  public GyroIOSim(GyroSimulation gyroSimulation) {
    this.gyroSimulation = gyroSimulation;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    Rotation2d[] cachedReadings = gyroSimulation.getCachedGyroReadings();

    inputs.connected = true;
    inputs.yawPosition = finiteOrZero(gyroSimulation.getGyroReading());
    inputs.yawVelocityRadPerSec =
        finiteOrZero(gyroSimulation.getMeasuredAngularVelocity().in(RadiansPerSecond));
    inputs.odometryYawTimestamps = makeOdometryTimestamps(cachedReadings.length);
    inputs.odometryYawPositions = new Rotation2d[cachedReadings.length];
    for (int i = 0; i < cachedReadings.length; i++) {
      inputs.odometryYawPositions[i] = finiteOrZero(cachedReadings[i]);
    }
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

  private static double finiteOrZero(double value) {
    return Double.isFinite(value) ? value : 0.0;
  }

  private static Rotation2d finiteOrZero(Rotation2d rotation) {
    return Double.isFinite(rotation.getRadians()) ? rotation : Rotation2d.kZero;
  }
}
