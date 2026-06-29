package frc.robot.commands.AutoalignIntakeCommand;

public final class AutoalignIntakeConstants {
  // Joystick deadband applied to the translation magnitude.
  public static final double DEADBAND = 0.1;

  // Heading controller gains (same as DriveCommands.joystickDriveAtAngle).
  public static final double ANGLE_KP = 5.0;
  public static final double ANGLE_KD = 0.4;
  public static final double ANGLE_MAX_VELOCITY = 8.0;
  public static final double ANGLE_MAX_ACCELERATION = 20.0;

  // Hard cap on the heading-controller output so rotation never eats the whole module speed budget
  // and starves translation.
  public static final double MAX_ANGULAR_VELOCITY_RAD_PER_SEC = 3.0;

  // Roller speed while aligning. Negative is the current (outtake) direction; the right-trigger/'a'
  // intake bindings use positive, so flip the sign to +20.0 if this should intake instead.
  public static final double ROLLER_VELOCITY_ROT_PER_SEC = 30.0;

  private AutoalignIntakeConstants() {}
}
