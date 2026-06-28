// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.Feeder;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.io.MotorIOPhoenix6.ClosedLoopOutput;
import frc.lib.io.MotorIOPhoenix6.MotorIOPhoenix6Config;
import frc.lib.io.MotorIOPhoenix6.NeutralMode;
import frc.lib.io.MotorIOSim.MotorIOSimConfig;

public final class FeederConstants {
  public static final String CAN_BUS = "rio";

  public static final int Feeder_L_ID = 30;
  public static final int Feeder_R_ID = 31;

  public static final double Feeder_ROTOR_TO_MECHANISM_RATIO = 1.0;

  public static final MotorIOPhoenix6Config Feeder_CONFIG =
      new MotorIOPhoenix6Config(Feeder_L_ID, CAN_BUS)
          .withRotorToMechanismRatio(Feeder_ROTOR_TO_MECHANISM_RATIO)
          .withInverted(false)
          .withNeutralMode(NeutralMode.COAST)
          .withCurrentLimits(30.0, 60.0)
          .withSlot0(10, 0.0, 0.0, 7, 0.0, 0.0, 0.0)
          .withClosedLoopOutput(ClosedLoopOutput.TORQUE_CURRENT_FOC)
          .withFollower(Feeder_R_ID, true);

  public static final MotorIOSimConfig Feeder_SIM_CONFIG =
      new MotorIOSimConfig()
          .withGearbox(DCMotor.getKrakenX60Foc(2))
          .withGearing(Feeder_ROTOR_TO_MECHANISM_RATIO)
          .withMomentOfInertia(0.003)
          .withVelocityGains(0.12, 0.0);

  private FeederConstants() {}
}
