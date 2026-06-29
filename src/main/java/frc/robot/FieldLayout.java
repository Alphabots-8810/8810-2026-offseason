package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

public class FieldLayout {
  public static final Distance BLUE_ALLIANCE_LINE = Units.Feet.of(13.0).plus(Units.Inches.of(0.06));
  public static final Distance FIELD_LENGTH = Units.Feet.of(54.0).plus(Units.Inches.of(2.12));
  public static final Distance FIELD_WIDTH = Units.Feet.of(26.0).plus(Units.Inches.of(4.64));
  public static final Distance Y_MIDDLE_LINE = FIELD_WIDTH.div(2.0);

  public static Pose2d rotateAboutCentre(Pose2d pose, Rotation2d rot) {
    return new Pose2d(
        pose.getTranslation()
            .rotateAround(
                new Translation2d(FIELD_LENGTH.div(2), FIELD_WIDTH.div(2)),
                pose.getRotation().plus(rot)),
        rot);
  }

  public static Pose2d handleAllianceFlip(Pose2d blue_pose, boolean is_red_alliance) {
    if (is_red_alliance) {
      blue_pose = rotateAboutCentre(blue_pose, Rotation2d.k180deg);
    }
    return blue_pose;
  }

  public static Translation2d handleAllianceFlip(
      Translation2d blue_translation, boolean is_red_alliance) {
    if (is_red_alliance) {
      blue_translation =
          blue_translation.rotateAround(
              new Translation2d(FIELD_LENGTH.div(2.0), FIELD_WIDTH.div(2.0)), Rotation2d.k180deg);
    }
    return blue_translation;
  }

  public static Rotation2d handleAllianceFlip(Rotation2d blue_rotation, boolean is_red_alliance) {
    if (is_red_alliance) {
      blue_rotation = blue_rotation.plus(Rotation2d.k180deg);
    }
    return blue_rotation;
  }

  public static Distance distanceFromAllianceWall(Distance x_coordinate, boolean is_red_alliance) {
    if (is_red_alliance) {
      return FIELD_LENGTH.minus(x_coordinate);
    }
    return x_coordinate;
  }

  public static boolean isPoseWithinAllianceZone(boolean isRedAlliance, Pose2d pose) {
    return distanceFromAllianceWall(pose.getMeasureX(), isRedAlliance)
        .lte(BLUE_ALLIANCE_LINE.plus(Units.Feet.of(4)));
  }

  public static Pose2d rotateAboutPose(Pose2d startPose, Translation2d point, Rotation2d rotation) {
    return new Pose2d(
        startPose.getTranslation().rotateAround(point, rotation),
        startPose.getRotation().plus(rotation));
  }

  public static boolean isPoseOnLeftSide(boolean isRedAlliance, Distance y_coordinate) {
    if (isRedAlliance) {
      return y_coordinate.lte(FIELD_WIDTH.div(2.0));
    } else {
      return y_coordinate.gte(FIELD_WIDTH.div(2.0));
    }
  }
}
