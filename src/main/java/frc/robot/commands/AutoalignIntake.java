package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeRoller.IntakeRoller;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

public class AutoalignIntake extends Command {
  private Drive drive;
  private IntakeRoller roller;
  private DoubleSupplier xSupplier;
  private DoubleSupplier ySupplier;

  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 5.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  public AutoalignIntake(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.drive = Drive.mInstance;
    this.roller = IntakeRoller.mInstance;
    addRequirements(drive, roller);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    roller.setVelocityRotPerSec(-20);
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            xSupplier.getAsDouble(),
            ySupplier.getAsDouble(),
            Math.atan(ySupplier.getAsDouble() / xSupplier.getAsDouble()));
    drive.runVelocity(speeds);
  }

  @Override
  public void end(boolean interrupted) {
    roller.setV(0);
  }
}
