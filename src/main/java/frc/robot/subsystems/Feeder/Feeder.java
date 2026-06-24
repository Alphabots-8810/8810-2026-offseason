// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.Feeder;

import frc.lib.bases.MotorSubsystem;
import frc.lib.io.MotorIO;
import frc.lib.io.MotorIOPhoenix6;
import frc.lib.io.MotorIOSim;
import frc.robot.Constants;

public class Feeder extends MotorSubsystem {
  public static final Feeder mInstance = new Feeder();

  private Feeder() {
    super(
        switch (Constants.currentMode) {
          case REAL -> new MotorIOPhoenix6(FeederConstants.Feeder_CONFIG);
          case SIM -> new MotorIOSim(FeederConstants.Feeder_SIM_CONFIG);
          case REPLAY -> new MotorIO() {};
        },
        "Feeder",
        "Disconnected Feeder motor.");
  }
}
