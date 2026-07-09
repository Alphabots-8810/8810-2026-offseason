package frc.robot.simulation;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import java.lang.reflect.Field;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltHub;
import org.littletonrobotics.junction.Logger;

/**
 * Maple-sim field arena and swerve drivetrain physics. Owns {@link Arena2026Rebuilt} and {@link
 * SwerveDriveSimulation}; fuel intake/shooter logic lives in {@link FuelSimulation}.
 */
public final class MapleSimArena {
  private static final boolean EFFICIENCY_MODE = true;
  private static final Pose2d INITIAL_POSE = new Pose2d(1.5, 4.0, Rotation2d.kZero);

  // Arena2026Rebuilt field extents. maple-sim 0.4.0-beta FieldMirroringUtils still uses the
  // Reefscape 17.548 x 8.052 size when building RebuiltHub.redShootPoses.
  private static final double REBUILT_FIELD_X_MAX = 16.54105;
  private static final double REBUILT_FIELD_Y_MAX = 8.06926;

  private static MapleSimArena instance;

  private final Arena2026Rebuilt arena;
  private final SwerveDriveSimulation driveSimulation;

  private MapleSimArena() {
    // Must run after RebuiltHub class init (static redShootPoses) and before scoring.
    patchRedHubRecyclePoses();

    arena = new Arena2026Rebuilt();
    arena.setEfficiencyMode(EFFICIENCY_MODE);
    SimulatedArena.overrideInstance(arena);

    driveSimulation = new SwerveDriveSimulation(createDriveConfig(), INITIAL_POSE);
    arena.addDriveTrainSimulation(driveSimulation);
  }

  /**
   * maple-sim 0.4.0-beta mirrors blue hub recycle chutes with Reefscape field length (17.548 m). On
   * the 16.54 m Rebuilt field that puts red recycle spawns on the wrong side of the red hub, so
   * returned fuel immediately re-enters the hub and vanishes. Blue is unaffected (no mirror).
   */
  private static void patchRedHubRecyclePoses() {
    try {
      Field blueField = RebuiltHub.class.getDeclaredField("blueShootPoses");
      blueField.setAccessible(true);
      Pose3d[] blueShootPoses = (Pose3d[]) blueField.get(null);
      Pose3d[] redShootPoses = RebuiltHub.redShootPoses;

      for (int i = 0; i < redShootPoses.length; i++) {
        Pose3d blue = blueShootPoses[i];
        redShootPoses[i] =
            new Pose3d(
                new Translation3d(
                    REBUILT_FIELD_X_MAX - blue.getX(),
                    REBUILT_FIELD_Y_MAX - blue.getY(),
                    blue.getZ()),
                blue.getRotation().plus(new Rotation3d(0.0, 0.0, Math.PI)));
      }
    } catch (ReflectiveOperationException e) {
      DriverStation.reportWarning(
          "Failed to patch maple-sim red hub recycle poses: " + e.getMessage(), false);
    }
  }

  public static synchronized MapleSimArena getInstance() {
    if (Constants.currentMode != Constants.Mode.SIM) {
      throw new IllegalStateException("MapleSimArena can only be created in SIM mode.");
    }
    if (instance == null) {
      instance = new MapleSimArena();
    }
    return instance;
  }

  public SwerveDriveSimulation getDriveSimulation() {
    return driveSimulation;
  }

  public Pose2d getRobotPose() {
    Pose2d pose = driveSimulation.getSimulatedDriveTrainPose();
    return isFinite(pose) ? pose : INITIAL_POSE;
  }

  public void setRobotPose(Pose2d pose) {
    Pose2d safePose = isFinite(pose) ? pose : INITIAL_POSE;
    driveSimulation.setSimulationWorldPose(safePose);
    driveSimulation.getGyroSimulation().setRotation(safePose.getRotation());
  }

  public void resetField() {
    arena.resetFieldForAuto();
    setRobotPose(INITIAL_POSE);
    if (Drive.mInstance != null) {
      Drive.mInstance.setPoseFromSimulation(INITIAL_POSE);
    }
    FuelSimulation fuelSimulation = FuelSimulation.getInstance();
    if (fuelSimulation != null) {
      fuelSimulation.reset();
    }
  }

  /** Advances maple-sim physics and syncs odometry to the simulated chassis pose. */
  public void simulationPeriodic() {
    arena.simulationPeriodic();

    Pose2d robotPose = driveSimulation.getSimulatedDriveTrainPose();
    boolean recoveredPose = !isFinite(robotPose);
    if (recoveredPose) {
      setRobotPose(INITIAL_POSE);
      robotPose = INITIAL_POSE;
    }

    if (Drive.mInstance != null) {
      Drive.mInstance.setPoseFromSimulation(robotPose);
    }

    Logger.recordOutput("FieldSimulation/Robot", robotPose);
    Logger.recordOutput("FieldSimulation/RecoveredNaNPose", recoveredPose);
    Logger.recordOutput("FieldSimulation/Fuel", arena.getGamePiecesArrayByType("Fuel"));
  }

  private static DriveTrainSimulationConfig createDriveConfig() {
    return DriveTrainSimulationConfig.Default()
        .withRobotMass(Kilograms.of(74.088))
        .withBumperSize(Meters.of(0.85), Meters.of(0.85))
        .withCustomModuleTranslations(Drive.getModuleTranslations())
        .withGyro(COTS.ofPigeon2())
        .withSwerveModule(createModuleConfig());
  }

  private static SwerveModuleSimulationConfig createModuleConfig() {
    return new SwerveModuleSimulationConfig(
        DCMotor.getKrakenX60Foc(1),
        DCMotor.getKrakenX60Foc(1),
        TunerConstants.FrontLeft.DriveMotorGearRatio,
        TunerConstants.FrontLeft.SteerMotorGearRatio,
        Volts.of(0.2),
        Volts.of(0.2),
        Meters.of(TunerConstants.FrontLeft.WheelRadius),
        KilogramSquareMeters.of(0.03),
        1.2);
  }

  private static boolean isFinite(Pose2d pose) {
    return pose != null
        && Double.isFinite(pose.getX())
        && Double.isFinite(pose.getY())
        && Double.isFinite(pose.getRotation().getRadians());
  }
}
