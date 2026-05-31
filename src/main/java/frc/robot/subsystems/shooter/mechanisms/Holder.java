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
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Holder extends MotorSubsystem {
  public static enum State {
    EMPTY,
    INTAKING,
    HOLDING,
    FEEDING
  }

  private final BooleanSupplier pieceSupplier;

  private State state = State.EMPTY;
  private double goalVelocityRps = 0.0;
  private double goalVolts = 0.0;
  private boolean velocityControl = false;
  private boolean simHasPiece = false;

  public Holder() {
    this(createIO(), () -> false);
  }

  public Holder(MotorIO io) {
    this(io, () -> false);
  }

  public Holder(MotorIO io, BooleanSupplier pieceSupplier) {
    super(io, "Shooter/Holder", "Disconnected shooter holder motor.");
    this.pieceSupplier = pieceSupplier;
  }

  @Override
  public void periodic() {
    super.periodic();

    switch (state) {
      case EMPTY:
        stopMotor();
        if (hasPiece()) {
          state = State.HOLDING;
        }
        break;

      case INTAKING:
        runGoal();
        if (hasPiece()) {
          state = State.HOLDING;
        }
        break;

      case HOLDING:
        setVoltageOut(ShooterConstants.HOLDER_HOLD_VOLTS);
        if (!hasPiece()) {
          state = State.EMPTY;
        }
        break;

      case FEEDING:
        runGoal();
        if (!hasPiece()) {
          state = State.EMPTY;
        }
        break;
    }

    Logger.recordOutput("Shooter/Holder/State", state);
    Logger.recordOutput("Shooter/Holder/HasPiece", hasPiece());
    Logger.recordOutput("Shooter/Holder/GoalRps", goalVelocityRps);
    Logger.recordOutput("Shooter/Holder/GoalVolts", goalVolts);
  }

  public void intake() {
    goalVolts = ShooterConstants.HOLDER_INTAKE_VOLTS;
    velocityControl = false;
    state = State.INTAKING;
  }

  public void hold() {
    state = hasPiece() ? State.HOLDING : State.EMPTY;
  }

  public void feed() {
    setVoltage(ShooterConstants.HOLDER_FEED_VOLTS);
    state = State.FEEDING;
  }

  public void setVelocity(double rps) {
    goalVelocityRps = rps;
    velocityControl = true;
    state = Math.abs(rps) < 1e-6 ? State.EMPTY : State.FEEDING;
  }

  public void setVoltage(double volts) {
    goalVolts = volts;
    velocityControl = false;
    state = Math.abs(volts) < 1e-6 ? State.EMPTY : State.INTAKING;
  }

  public void stop() {
    goalVelocityRps = 0.0;
    goalVolts = 0.0;
    velocityControl = false;
    state = State.EMPTY;
  }

  public boolean hasPiece() {
    return pieceSupplier.getAsBoolean() || simHasPiece;
  }

  public void setSimHasPiece(boolean hasPiece) {
    simHasPiece = hasPiece;
  }

  public double getVelocityRps() {
    return Units.radiansToRotations(getVelocityRadPerSec());
  }

  public State getState() {
    return state;
  }

  private void runGoal() {
    if (velocityControl) {
      setVelocityRadPerSec(Units.rotationsToRadians(goalVelocityRps));
    } else {
      setVoltageOut(goalVolts);
    }
  }

  private static MotorIO createIO() {
    return switch (Constants.currentMode) {
      case REAL -> new MotorIOPhoenix6(ShooterConstants.HOLDER_CONFIG);
      case SIM -> new MotorIOSim(ShooterConstants.HOLDER_SIM_CONFIG);
      case REPLAY -> new MotorIO() {};
    };
  }
}
