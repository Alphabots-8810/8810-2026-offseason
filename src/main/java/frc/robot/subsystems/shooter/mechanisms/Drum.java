// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter.mechanisms;

import edu.wpi.first.math.util.Units;
import frc.lib.bases.MotorSubsystem;
import frc.lib.io.MotorIO;
import frc.lib.io.MotorIOPhoenix6;
import frc.lib.io.MotorIOSim;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterConstants;
import org.littletonrobotics.junction.Logger;

public class Drum extends MotorSubsystem {
  public static enum State {
    STOPPED,
    SPINNING_UP,
    AT_SPEED
  }

  private State state = State.STOPPED;
  private double goalVelocityRps = 0.0;
  private double openLoopVolts = 0.0;
  private boolean openLoop = false;

  public Drum() {
    this(createIO());
  }

  public Drum(MotorIO io) {
    super(io, "Shooter/Drum", "Disconnected shooter drum motor.");
  }

  @Override
  public void periodic() {
    super.periodic();

    if (state == State.STOPPED) {
      stopMotor();
    } else if (openLoop) {
      setVoltageOut(openLoopVolts);
      state = Math.abs(openLoopVolts) < 1e-6 ? State.STOPPED : State.SPINNING_UP;
    } else {
      setVelocityRadPerSec(Units.rotationsToRadians(goalVelocityRps));
      state = atGoal() ? State.AT_SPEED : State.SPINNING_UP;
    }

    Logger.recordOutput("Shooter/Drum/State", state);
    Logger.recordOutput("Shooter/Drum/GoalRps", goalVelocityRps);
  }

  public void setGoal(double rps) {
    setVelocity(rps);
  }

  public void setVelocity(double rps) {
    goalVelocityRps = rps;
    openLoop = false;
    state = Math.abs(rps) < 1e-6 ? State.STOPPED : State.SPINNING_UP;
  }

  public void setVoltage(double volts) {
    openLoopVolts = volts;
    openLoop = true;
    state = Math.abs(volts) < 1e-6 ? State.STOPPED : State.SPINNING_UP;
  }

  public void stop() {
    goalVelocityRps = 0.0;
    openLoopVolts = 0.0;
    openLoop = false;
    state = State.STOPPED;
  }

  public boolean atGoal() {
    return Math.abs(goalVelocityRps - getVelocityRps())
        <= ShooterConstants.DRUM_VELOCITY_TOLERANCE_RPS;
  }

  public double getVelocityRps() {
    return Units.radiansToRotations(getVelocityRadPerSec());
  }

  public State getState() {
    return state;
  }

  private static MotorIO createIO() {
    return switch (Constants.currentMode) {
      case REAL -> new MotorIOPhoenix6(ShooterConstants.DRUM_CONFIG);
      case SIM -> new MotorIOSim(ShooterConstants.DRUM_SIM_CONFIG);
      case REPLAY -> new MotorIO() {};
    };
  }
}
