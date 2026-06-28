package frc.robot.subsystems.SuperStructure;

public final class SuperStrucutrerConstants {

  // 2.2 – IntakeRoller voltages
  public static final double INTAKE_8V = 8.0;
  public static final double INTAKE_12V = 12.0;

  // 3.1 – Feeder voltage
  public static final double FEED_12V = 12.0;

  // 4.1 – Drum: 6 000 RPM converted to rad/s
  public static final double DRUM_SHOOT_RADS = 6000.0 * (2.0 * Math.PI) / 60.0;

  // 4.3 – Hood down position (radians — tune on robot)
  public static final double HOOD_DOWN_POSITION_RAD = 0.0;

  private SuperStrucutrerConstants() {}
}
