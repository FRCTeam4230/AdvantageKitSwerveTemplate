package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.SchoolMap;
import java.util.Set;
import org.littletonrobotics.junction.AutoLogOutput;

public class Patrol {
  private Patrol() {}

  @AutoLogOutput private static int currentPointIndex = 0;

  public static Command patrol(Drive drive) {
    return Commands.sequence(
            Commands.runOnce(() -> drive.setRotateTowardsEndOfPath(true)),
            Commands.defer(
                    () ->
                        DriveToPointBuilder.driveToNoFlip(
                                new Pose2d(
                                    SchoolMap.PATROL_POINTS[currentPointIndex], new Rotation2d()))
                            .andThen(
                                Commands.runOnce(
                                    () ->
                                        currentPointIndex =
                                            (currentPointIndex + 1)
                                                % SchoolMap.PATROL_POINTS.length)),
                    Set.of(drive))
                .repeatedly())
        .finallyDo(() -> drive.setRotateTowardsEndOfPath(false));
  }
}
