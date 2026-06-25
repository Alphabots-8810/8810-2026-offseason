// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.IntakeDeploy;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.io.MotorIOPhoenix6.ClosedLoopOutput;
import frc.lib.io.MotorIOPhoenix6.MotorIOPhoenix6Config;
import frc.lib.io.MotorIOPhoenix6.NeutralMode;
import frc.lib.io.MotorIOSim.MotorIOSimConfig;

public final class IntakeDeployConstants {
  public static final String CAN_BUS = "rio";

  public static final int IntakeDeploy_ID = 22;

  public static final double IntakeDeploy_ROTOR_TO_MECHANISM_RATIO = 1.0;

  public static final MotorIOPhoenix6Config IntakeDeploy_CONFIG =
      new MotorIOPhoenix6Config(IntakeDeploy_ID, CAN_BUS)
          .withRotorToMechanismRatio(IntakeDeploy_ROTOR_TO_MECHANISM_RATIO)
          .withInverted(false)
          .withNeutralMode(NeutralMode.BRAKE)
          .withCurrentLimits(30.0, 60.0)
          .withSlot0(0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
          .withClosedLoopOutput(ClosedLoopOutput.TORQUE_CURRENT_FOC);

  public static final MotorIOSimConfig IntakeDeploy_SIM_CONFIG =
      new MotorIOSimConfig()
          .withGearbox(DCMotor.getKrakenX60Foc(2))
          .withGearing(IntakeDeploy_ROTOR_TO_MECHANISM_RATIO)
          .withMomentOfInertia(0.003)
          .withVelocityGains(0.12, 0.0);

  private IntakeDeployConstants() {}
}


