package frc.robot.util;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commands.auto.AutoConstants;
import java.util.*;

public class AutoConfigParser {
  public record AutoPart(
      Optional<Translation2d> note,
      Translation2d shootingTranslation,
      Optional<Pose2d> notePickupPose,
      Optional<List<Pair<Translation2d, Translation2d>>> obstacles,
      Optional<Integer> time) {}

  public static final Map<Character, Pose2d> pickupPoseMap = new HashMap<>();

  static {
    pickupPoseMap.put('x', AutoConstants.NotePickupLocations.X);
    pickupPoseMap.put('y', AutoConstants.NotePickupLocations.Y);
    pickupPoseMap.put('z', AutoConstants.NotePickupLocations.Z);
  }

  public static final Map<Character, Translation2d> shootingPoseMap = new HashMap<>();

  static {
    shootingPoseMap.put('a', AutoConstants.ShootingTranslations.SPEAKER_AMP_SIDE);
    shootingPoseMap.put('b', AutoConstants.ShootingTranslations.SPEAKER_CENTER);
    shootingPoseMap.put('c', AutoConstants.ShootingTranslations.SPEAKER_SOURCE_SIDE);
    shootingPoseMap.put('d', AutoConstants.ShootingTranslations.BETWEEN_0_1);
    shootingPoseMap.put('e', AutoConstants.ShootingTranslations.BETWEEN_1_2);
    shootingPoseMap.put('f', AutoConstants.ShootingTranslations.STAGE_AMP_SIDE);
    shootingPoseMap.put('g', AutoConstants.ShootingTranslations.STAGE_SOURCE_SIDE);
  }

  public static final Map<Character, Pair<Translation2d, Translation2d>> obstacleMap =
      new HashMap<>();

  static {
    obstacleMap.put('r', AutoConstants.AvoidanceZones.SOURCE_SIDE_NEXT_TO_STAGE);
    obstacleMap.put('s', AutoConstants.AvoidanceZones.STAGE);
    obstacleMap.put('t', AutoConstants.AvoidanceZones.CLOSE_NOTES);
  }

  /**
   * this method flips the positions based on the driverstation state when it is called
   *
   * @param config the config string
   * @return a list of AutoParts. is empty if the string is invalid
   */
  public static Optional<List<AutoPart>> parseAutoConfig(String config) {
    try {
      final ArrayList<AutoPart> output = new ArrayList<>();
      final String[] parts = config.split("-");

      Optional<List<Pair<Translation2d, Translation2d>>> currentObstacles = Optional.empty();
      Optional<Integer> time = Optional.empty();

      for (String part : parts) {
        final var newTime = getTimeFromStringPart(part);

        // section was just configuring obstacles, no notes here
        if (newTime.isPresent()) {
          time = newTime;
          continue;
        }

        final var newObstacles = getObstaclesFromStringPart(part);

        // section was just configuring obstacles, no notes here
        if (newObstacles.isPresent()) {
          currentObstacles = newObstacles;
          continue;
        }

        if (part.length() < 2) {
          return Optional.empty();
        }
        final var shootingPose =
            Optional.ofNullable(shootingPoseMap.get(part.charAt(part.length() - 1)));
        final var pickupPose = Optional.ofNullable(pickupPoseMap.get(part.charAt(0)));
        if (shootingPose.isEmpty()) {
          return Optional.empty();
        }
        for (int noteCharInt : part.substring(1, part.length() - 1).chars().toArray()) {
          final var finalCurrentObstacles = currentObstacles;

          Optional<Translation2d> note;

          if (noteCharInt == '?') {
            note = Optional.empty();
          } else {
            note =
                Optional.of(
                    AllianceFlipUtil.apply(
                        AutoConstants.AUTO_NOTES[Character.getNumericValue(noteCharInt)]));
          }

          output.add(
              new AutoPart(
                  note,
                  AllianceFlipUtil.apply(shootingPose.get()),
                  pickupPose.map(AllianceFlipUtil::apply),
                  finalCurrentObstacles,
                  time));

          // we set obstacles on last note, don't do it again for the next one
          currentObstacles = Optional.empty();
        }
      }
      return Optional.of(output);
    } catch (Exception e) {
      return Optional.empty();
    }
  }

  private static Optional<Integer> getTimeFromStringPart(String part) {
    if (part.charAt(0) != '@') {
      return Optional.empty();
    }

    return Optional.of(Integer.parseInt(part.substring(1)));
  }

  private static Optional<List<Pair<Translation2d, Translation2d>>> getObstaclesFromStringPart(
      String part) {
    if (part.charAt(0) != '!') {
      return Optional.empty();
    }

    return Optional.of(
        part.substring(1)
            .chars()
            .mapToObj(i -> Optional.ofNullable(obstacleMap.get((char) i)))
            .filter(Optional::isPresent)
            .map(Optional::get)
            .toList());
  }
}
