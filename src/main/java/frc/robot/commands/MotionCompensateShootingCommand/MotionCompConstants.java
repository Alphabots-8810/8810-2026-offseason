package frc.robot.commands.MotionCompensateShootingCommand;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.util.LoggedTunableNumber;

public final class MotionCompConstants {
  public static final double MaxShootWhileMovingSpeedMpsTunable = 1.5;
  public static final LoggedTunableNumber CompensationScaleTunable =
      new LoggedTunableNumber("Shooting/MotionComp/CompensationScale", 1.0);

public static final InterpolatingDoubleTreeMap distanceToFlyTime = new InterpolatingDoubleTreeMap();

	// add example data to the tree (distance in meters, time in seconds)
	static {

		// 12.067
		// 13.113
		distanceToFlyTime.put(1.967387, 1.046);

		// 2.033
		// 3.133
		distanceToFlyTime.put(2.207408, 1.08);

		// 29.7
		// 30.800
		distanceToFlyTime.put(2.57, 1.1);

		// 30.933
		// 32.1
		distanceToFlyTime.put(2.663911, 1.107);

		// 14.900
		// 16.1
		distanceToFlyTime.put(2.75, 1.2);

		// 41.367
		// 42.600
		distanceToFlyTime.put(2.80631, 1.233);

		// 27.33
		// 28.567
		distanceToFlyTime.put(3.0, 1.237);

		// 17.333
		// 18.600
		distanceToFlyTime.put(3.5696633, 1.267);

		// distanceToFlyTime.put(4.18, 1.3);

		// 6.193
		// 7.659
		// distanceToFlyTime.put(4.5, 1.466);
	}


  public static double getFlightTimeSec(double distanceMeters) {
    return distanceToFlyTime.get(distanceMeters);
  }

  private MotionCompConstants() {}
}
