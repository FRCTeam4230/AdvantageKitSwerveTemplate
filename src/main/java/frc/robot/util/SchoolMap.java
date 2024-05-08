package frc.robot.util;

import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class SchoolMap {
  private SchoolMap() {}

  public static final Translation2d OFFSET = new Translation2d(25, 0);

  private static Translation2d[] offsetTranslations(Translation2d[] translations) {
    return Arrays.stream(translations).map(t -> t.plus(OFFSET)).toArray(Translation2d[]::new);
  }

  public static final Translation2d[] PATROL_POINTS =
      offsetTranslations(
          new Translation2d[] {
            new Translation2d(-1.5, 1.5),
            new Translation2d(-19.5, 1.5),
            new Translation2d(-21.5, 28),
            new Translation2d(-10, 28),
          });

  private static final double BUFFER_DISTANCE = 0.5;

  public static final Translation2d[] PERIMETER_POINTS =
      offsetTranslations(
          new Translation2d[] {
            new Translation2d(0, 0),
            new Translation2d(-21, 0),
            new Translation2d(-21, 3.43),
            new Translation2d(-18, 3.43),
            new Translation2d(-18, 27),
            new Translation2d(-23, 27),
            new Translation2d(-23, 29),
            new Translation2d(-9, 29),
            new Translation2d(-9, 27),
            new Translation2d(-14.36, 27),
            new Translation2d(-14.36, 3.43),
            new Translation2d(-0, 3.43),
            new Translation2d(0, 0),
          });

  public static final List<Pair<Translation2d, Translation2d>> PERIMETER_OBSTACLES =
      new ArrayList<>();

  static {
    for (int i = 0; i < PERIMETER_POINTS.length - 1; i++) {
      final var a = PERIMETER_POINTS[i];
      final var b = PERIMETER_POINTS[i + 1];

      final boolean aAboveB = a.getY() > b.getY();
      final boolean aLeftOfB = a.getX() < b.getX();

      final Translation2d aBuffer =
          new Translation2d(
              aLeftOfB ? -BUFFER_DISTANCE : BUFFER_DISTANCE,
              aAboveB ? BUFFER_DISTANCE : -BUFFER_DISTANCE);
      final Translation2d bBuffer = aBuffer.times(-1);

      PERIMETER_OBSTACLES.add(new Pair<>(a.plus(aBuffer), b.plus(bBuffer)));
    }
  }

  public static void setupHallObstacles(Translation2d robotTranslation) {
    Logger.recordOutput("perimeter points", PERIMETER_POINTS);
    Pathfinding.setDynamicObstacles(PERIMETER_OBSTACLES, robotTranslation);
  }
}
