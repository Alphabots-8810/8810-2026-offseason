package frc.robot.subsystems.SuperStructure;

import static frc.robot.subsystems.SuperStructure.SuperStrucutrerConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drum.Drum;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.IntakeDeploy.IntakeDeploy;
import frc.robot.subsystems.IntakeRoller.IntakeRoller;
import org.littletonrobotics.junction.Logger;

public class SuperStructure extends SubsystemBase {
  public static final SuperStructure mInstance = new SuperStructure();

  // ── Drive mode (1.1) ─────────────────────────────────────────────────────────

  public enum DriveMode {
    FOLLOW_JOYSTICKS,
    AIM_AND_LOCK
  }

  // ── Hood control mode (4.2 / 4.3) ────────────────────────────────────────────

  public enum HoodMode {
    DOWN, // fixed down position
    CALCULATED, // TODO: auto-calculated angle + rpm
    MANUAL // manual rpm from setManualRpm()
  }

  // ── State table ──────────────────────────────────────────────────────────────
  //                         1.1                       2.2      3.1      4.1
  // 4.2/4.3
  public enum State {
    STOW(DriveMode.FOLLOW_JOYSTICKS, 0.0, 0.0, 0.0, HoodMode.DOWN),
    SHOOTING(DriveMode.AIM_AND_LOCK, INTAKE_8V, FEED_12V, DRUM_SHOOT_ROTPS, HoodMode.CALCULATED),
    INTAKING(DriveMode.FOLLOW_JOYSTICKS, INTAKE_12V, 0.0, 0.0, HoodMode.DOWN),
    FERRYING(DriveMode.AIM_AND_LOCK, INTAKE_8V, FEED_12V, DRUM_SHOOT_ROTPS, HoodMode.CALCULATED),
    OVERRIDE(DriveMode.FOLLOW_JOYSTICKS, INTAKE_8V, FEED_12V, DRUM_SHOOT_ROTPS, HoodMode.MANUAL);

    public final DriveMode driveMode;
    public final double intakeRollerVolts; // 2.2
    public final double feederVolts; // 3.1
    public final double drumRotPerSec; // 4.1 — 0 means coast/stop
    public final HoodMode hoodMode; // 4.2 / 4.3

    State(DriveMode dm, double ir, double fv, double drv, HoodMode hm) {
      driveMode = dm;
      intakeRollerVolts = ir;
      feederVolts = fv;
      drumRotPerSec = drv;
      hoodMode = hm;
    }
  }

  // ── Subsystem references ─────────────────────────────────────────────────────

  private final IntakeDeploy intakeDeploy = IntakeDeploy.mInstance; // 2.1 (TODO)
  private final IntakeRoller intakeRoller = IntakeRoller.mInstance; // 2.2
  private final Feeder feeder = Feeder.mInstance; // 3.1
  private final Drum drum = Drum.mInstance; // 4.1
  private final Hood hood = Hood.mInstance; // 4.2 / 4.3

  // ── Runtime fields ───────────────────────────────────────────────────────────

  private State wantedState = State.STOW;
  private DriveMode driveMode = DriveMode.FOLLOW_JOYSTICKS;
  private double manualRpm = 0.0;

  private SuperStructure() {}

  // ── Public API ───────────────────────────────────────────────────────────────

  public void setWantedState(State state) {
    wantedState = state;
  }

  public State getWantedState() {
    return wantedState;
  }

  public DriveMode getDriveMode() {
    return driveMode;
  }

  public void setManualRpm(double rpm) {
    manualRpm = rpm;
  }

  // ── Periodic ─────────────────────────────────────────────────────────────────

  @Override
  public void periodic() {
    // 1.1 – drive heading
    driveMode = wantedState.driveMode;

    // 2.1 – two-stage (TODO: setpoint)

    // 2.2 – intake roller
    intakeRoller.setV(wantedState.intakeRollerVolts);

    // 3.1 – feeder
    feeder.setV(wantedState.feederVolts);

    // 4.1 – drum
    if (wantedState.drumRotPerSec > 0.0) {
      drum.setVelocityRotPerSec(wantedState.drumRotPerSec);
    } else {
      drum.setV(0.0);
    }

    // 4.2 / 4.3 – hood
    switch (wantedState.hoodMode) {
      case DOWN -> hood.setPositionRot(HOOD_DOWN_POSITION_ROT);
      case CALCULATED -> {} // TODO: calculated setpoint
      case MANUAL -> hood.setVelocityRotPerSec(manualRpm / 60.0);
    }

    Logger.recordOutput("SuperStructure/State", wantedState.toString());
    Logger.recordOutput("SuperStructure/DriveMode", driveMode.toString());
  }
}
