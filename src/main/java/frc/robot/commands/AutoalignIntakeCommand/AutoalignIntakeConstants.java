package frc.robot.commands.AutoalignIntakeCommand;

public final class AutoalignIntakeConstants {
  // Joystick deadband applied to the translation magnitude.
  public static final double DEADBAND = 0.1;

  // Heading controller gains (same as DriveCommands.joystickDriveAtAngle).
  public static final double ANGLE_KP = 5.0;
  public static final double ANGLE_KD = 1.;
  public static final double ANGLE_MAX_VELOCITY = 12.0;
  public static final double ANGLE_MAX_ACCELERATION = 48.0;

  // Hard cap on the heading-controller output so rotation never eats the whole module speed budget
  // and starves translation.
  public static final double MAX_ANGULAR_VELOCITY_RAD_PER_SEC = 6.0;
  public static final double MANUAL_FIX_SCALOR = Math.PI / 10.;
  // Roller speed while aligning. Negative is the current (outtake) direction; the right-trigger/'a'
  // intake bindings use positive, so flip the sign to +20.0 if this should intake instead.
  public static final double ROLLER_VELOCITY_ROT_PER_SEC = 80.0;

  private AutoalignIntakeConstants() {}
}
