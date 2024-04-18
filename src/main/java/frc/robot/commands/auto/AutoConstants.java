package frc.robot.commands.auto;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.TunableNumberWrapper;
import java.lang.invoke.MethodHandles;
import java.util.List;
import java.util.Optional;
import java.util.stream.Stream;

public class AutoConstants {
  private static final TunableNumberWrapper tunableTable =
      new TunableNumberWrapper(MethodHandles.lookup().lookupClass());
  public static final LoggedTunableNumber DISTANCE_TO_TRUST_CAMERA =
      tunableTable.makeField("camera trust m", 1);
  public static final LoggedTunableNumber PATHFIND_UNTIL_DISTANCE =
      tunableTable.makeField("pathfind to note end m", 2);
  public static final LoggedTunableNumber SHOOTING_DISTANCE_OFFSET_TOLERANCE =
      tunableTable.makeField("align distance tolerance m", 0.1);
  public static final LoggedTunableNumber SHOOTING_ANGLE_OFFSET_TOLERANCE =
      tunableTable.makeField("align angle tolerance deg", 3);
  public static final Translation2d[] AUTO_NOTES =
      Stream.concat(
              Stream.concat(
                  Stream.of(FieldConstants.StagingLocations.spikeTranslations),
                  Stream.of(FieldConstants.StagingLocations.centerlineTranslations)),
              Stream.of(
                  new Translation2d(-1, 5),
                  new Translation2d(1, 9),
                  new Translation2d(17, 1),
                  new Translation2d(0, -1)))
          .toArray(Translation2d[]::new);

  public static class AutoNoteOffsetThresholds {
    public static final LoggedTunableNumber WHILE_ROUTING =
        tunableTable.makeField("auto note tolerance while routing", 0.5);
    public static final LoggedTunableNumber WHILE_ATTEMPTING_PICKUP =
        tunableTable.makeField("auto note tolerance while pickup", 1);
    public static final LoggedTunableNumber FALLBACK_MAX_PAST_CENTER =
        tunableTable.makeField("auto fallback past middle tolerance m", 1);
  }

  public static final LoggedTunableNumber PICKUP_TIMEOUT = tunableTable.makeField("pickup time", 3);

  private static final double BETWEEN_SPIKE_POSE_X = 1.9;

  public static class ShootingTranslations {
    public static final Translation2d SPEAKER_AMP_SIDE = new Translation2d(1, 6.7);
    public static final Translation2d SPEAKER_CENTER =
        new Translation2d(1.35, FieldConstants.Speaker.centerSpeakerOpening.getY());
    public static final Translation2d SPEAKER_SOURCE_SIDE = new Translation2d(0.9, 4.3);
    public static final Translation2d STAGE_AMP_SIDE = new Translation2d(2.8, 5.7);
    public static final Translation2d STAGE_SOURCE_SIDE = new Translation2d(2.4, 3.5);
    public static final Translation2d BETWEEN_1_2 =
        new Translation2d(
            BETWEEN_SPIKE_POSE_X,
            (FieldConstants.StagingLocations.spikeTranslations[1].getY()
                    + FieldConstants.StagingLocations.spikeTranslations[2].getY())
                / 2);
    public static final Translation2d BETWEEN_0_1 =
        new Translation2d(
            BETWEEN_SPIKE_POSE_X,
            (FieldConstants.StagingLocations.spikeTranslations[0].getY()
                    + FieldConstants.StagingLocations.spikeTranslations[1].getY())
                / 2);
  }

  public static Pose2d getShootingPose2dFromTranslation(Translation2d translation) {
    return new Pose2d(translation, AutoConstants.getShootingAngleFromTranslation(translation));
  }

  public static Rotation2d getShootingAngleFromTranslation(Translation2d translation) {
    return translation
        .minus(AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening.getTranslation()))
        .getAngle()
        .plus(
            Rotation2d.fromDegrees(
                DriveConstants.HeadingControllerConstants.SHOOTING_ANGLE_OFFSET_DEG.get()));
  }

  public static class AvoidanceZones {
    public static final Pair<Translation2d, Translation2d> STAGE =
        new Pair<>(new Translation2d(4.3, 4.7), new Translation2d(5.3, 3.7));
    public static final Pair<Translation2d, Translation2d> SOURCE_SIDE_NEXT_TO_STAGE =
        new Pair<>(new Translation2d(5.7, 1), new Translation2d(2.1, 3.8));
    public static final Pair<Translation2d, Translation2d> CLOSE_NOTES =
        new Pair<>(new Translation2d(2, 9), new Translation2d(3.2, 3.8));
    public static final Pair<Translation2d, Translation2d> AMP_SIDE_FAR_STAGE =
        new Pair<>(new Translation2d(5.5, 5.5), new Translation2d(6, 8.5));
    public static final Pair<Translation2d, Translation2d> MIDDLE_FAR_STAGE =
        new Pair<>(new Translation2d(5.5, 3), new Translation2d(6, 5.5));
    public static final Pair<Translation2d, Translation2d> SOURCE_SIDE_FAR_STAGE =
        new Pair<>(new Translation2d(5.5, 3), new Translation2d(6, -1));
    public static final Pair<Translation2d, Translation2d> AMP_SIDE_MIDDLE_STAGE =
        new Pair<>(new Translation2d(5.5, 5.5), new Translation2d(3.5, 5));
    public static final Pair<Translation2d, Translation2d> SOURCE_SIDE_MIDDLE_STAGE =
        new Pair<>(new Translation2d(5.5, 2), new Translation2d(3.5, 3.5));
    public static final Pair<Translation2d, Translation2d> MIDDLE_SUBWOOFER =
        new Pair<>(new Translation2d(1, 6.2), new Translation2d(2.3, 4.8));
  }

  public static List<Pair<Translation2d, Translation2d>> createDynamicObstaclesList(
      List<Pair<Translation2d, Translation2d>> zones) {
    return Stream.concat(
            zones.stream(),
            zones.stream()
                .map(
                    zone ->
                        new Pair<>(
                            AllianceFlipUtil.convertToRed(zone.getFirst()),
                            AllianceFlipUtil.convertToRed(zone.getSecond()))))
        .toList();
  }

  private static final Pose2d[] startingPoses = {
    new Pose2d(1.35, FieldConstants.Speaker.centerSpeakerOpening.getY(), Rotation2d.fromDegrees(0)),
    new Pose2d(0.72, 6.67, Rotation2d.fromDegrees(60)),
    new Pose2d(0.72, 4.44, Rotation2d.fromDegrees(-60)),
  };

  public static Optional<Pose2d> convertPoseIdToPose(int id) {
    try {
      return Optional.of(AllianceFlipUtil.apply(startingPoses[id]));
    } catch (Exception e) {
      return Optional.empty();
    }
  }
}
