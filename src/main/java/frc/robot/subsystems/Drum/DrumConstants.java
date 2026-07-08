// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.Drum;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.io.MotorIOPhoenix6.ClosedLoopOutput;
import frc.lib.io.MotorIOPhoenix6.MotorIOPhoenix6Config;
import frc.lib.io.MotorIOPhoenix6.NeutralMode;
import frc.lib.io.MotorIOSim.MotorIOSimConfig;

public final class DrumConstants {
  public static final String CAN_BUS = "rio";

  public static final int Drum_LU_ID = 40;
  public static final int Drum_LD_ID = 41;
  public static final int Drum_RU_ID = 42;
  public static final int Drum_RD_ID = 43;

  public static final double Drum_ROTOR_TO_MECHANISM_RATIO = 1.0;
  public static final double StowVelocity = 47.5;
  public static final MotorIOPhoenix6Config Drum_CONFIG =
      new MotorIOPhoenix6Config(Drum_LU_ID, CAN_BUS)
          .withRotorToMechanismRatio(Drum_ROTOR_TO_MECHANISM_RATIO)
          .withInverted(false)
          .withNeutralMode(NeutralMode.COAST)
          .withCurrentLimits(60.0, 80.0)
          .withSlot0(9.5, 0.0, 0.0, 3, 0.1, 0.0, 0.0)
          .withClosedLoopOutput(ClosedLoopOutput.TORQUE_CURRENT_FOC)
          .withFollower(Drum_LD_ID, false)
          .withFollower(Drum_RU_ID, true)
          .withFollower(Drum_RD_ID, true)
          .withPeakTorqueLimit(800, -60);

  public static final MotorIOSimConfig Drum_SIM_CONFIG =
      new MotorIOSimConfig()
          .withGearbox(DCMotor.getKrakenX60Foc(2))
          .withGearing(Drum_ROTOR_TO_MECHANISM_RATIO)
          .withMomentOfInertia(0.003)
          .withVelocityGains(0.12, 0.0);

  private DrumConstants() {}
}
