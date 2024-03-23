package frc.robot.util;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commands.auto.AutoConstants;
import java.util.*;

public class AutoConfigParser {
  public record AutoPart(
      Translation2d note,
      Translation2d shootingTranslation,
      Optional<Pose2d> notePickupPose,
      Optional<List<Pair<Translation2d, Translation2d>>> obstacles) {}

  public static final Map<Character, Pose2d> pickupPoseMap = new HashMap<>();

  static {
    pickupPoseMap.put('x', AutoConstants.NotePickupLocations.X);
    pickupPoseMap.put('y', AutoConstants.NotePickupLocations.Y);
    pickupPoseMap.put('z', AutoConstants.NotePickupLocations.Z);
  }

  public static final Map<Character, Translation2d> shootingPoseMap = new HashMap<>();

  static {
    shootingPoseMap.put('a', AutoConstants.ShootingTranslations.A);
    shootingPoseMap.put('b', AutoConstants.ShootingTranslations.B);
    shootingPoseMap.put('c', AutoConstants.ShootingTranslations.C);
    shootingPoseMap.put('d', AutoConstants.ShootingTranslations.D);
    shootingPoseMap.put('e', AutoConstants.ShootingTranslations.E);
    shootingPoseMap.put('f', AutoConstants.ShootingTranslations.F);
    shootingPoseMap.put('g', AutoConstants.ShootingTranslations.G);
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

      for (String part : parts) {
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
        for (Translation2d note :
            part.substring(1, part.length() - 1)
                .chars()
                .map(Character::getNumericValue)
                .mapToObj(i -> AutoConstants.AUTO_NOTES[i])
                .toList()) {
          final var finalCurrentObstacles = currentObstacles;

          output.add(
              new AutoPart(
                  AllianceFlipUtil.apply(note),
                  AllianceFlipUtil.apply(shootingPose.get()),
                  pickupPose.map(AllianceFlipUtil::apply),
                  finalCurrentObstacles));

          // we set obstacles on last note, don't do it again for the next one
          currentObstacles = Optional.empty();
        }
      }
      return Optional.of(output);
    } catch (Exception e) {
      return Optional.empty();
    }
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
