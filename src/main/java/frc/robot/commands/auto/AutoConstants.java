package frc.robot.commands.auto;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
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
      tunableTable.makeField("camera trust m", 3);
  ;
  public static final LoggedTunableNumber DRIVE_TO_PICKUP_INTERRUPT_DISTANCE =
      tunableTable.makeField("drive to pickup interupt m", 0);
  public static final LoggedTunableNumber SHOOTING_DISTANCE_OFFSET_TOLERANCE =
      tunableTable.makeField("align distance tolerance m", 0.1);
  public static final LoggedTunableNumber SHOOTING_ANGLE_OFFSET_TOLERANCE =
      tunableTable.makeField("align angle tolerance deg", Units.degreesToRadians(5));
  public static final Translation2d[] AUTO_NOTES =
      Stream.concat(
              Stream.of(FieldConstants.StagingLocations.spikeTranslations),
              Stream.of(FieldConstants.StagingLocations.centerlineTranslations))
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

  public static class ShootingTranslations {
    public static final Translation2d A = new Translation2d(1, 6.7);
    public static final Translation2d B =
        new Translation2d(1.4, FieldConstants.Speaker.centerSpeakerOpening.getY());
    public static final Translation2d C = new Translation2d(0.9, 4.3);
    public static final Translation2d D = new Translation2d(3.7, 5.7);
    public static final Translation2d E = new Translation2d(4.8, 6.2);
    public static final Translation2d F = new Translation2d(3.1, 2.9);
    public static final Translation2d G = new Translation2d(4.3, 2.3);
  }

  public static Pose2d getShootingPose2dFromTranslation(Translation2d translation) {
    return new Pose2d(translation, AutoConstants.getShootingAngleFromTranslation(translation));
  }

  public static Rotation2d getShootingAngleFromTranslation(Translation2d translation) {
    return translation
        .minus(AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening.getTranslation()))
        .getAngle();
  }

  public static class NotePickupLocations {
    public static final Pose2d X =
        new Pose2d(new Translation2d(5.9, 6.5), Rotation2d.fromDegrees(-10));
    public static final Pose2d Y =
        new Pose2d(
            new Translation2d(5.9, FieldConstants.fieldWidth / 2), Rotation2d.fromDegrees(0));
    public static final Pose2d Z =
        new Pose2d(new Translation2d(5.9, 1.7), Rotation2d.fromDegrees(10));
  }

  public static class AvoidanceZones {
    public static final Pair<Translation2d, Translation2d> STAGE =
        new Pair<>(new Translation2d(4.3, 4.7), new Translation2d(5.3, 3.7));
    public static final Pair<Translation2d, Translation2d> SOURCE_SIDE_NEXT_TO_STAGE =
        new Pair<>(new Translation2d(5.7, 1.8), new Translation2d(2.1, 3.8));
    public static final Pair<Translation2d, Translation2d> CLOSE_NOTES =
        new Pair<>(new Translation2d(2, 9), new Translation2d(3.2, 3.8));
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
