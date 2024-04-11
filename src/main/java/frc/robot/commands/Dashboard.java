package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.auto.AutoConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.AutoConfigParser;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.Supplier;

public class Dashboard {
  private Dashboard() {}

  public static Command logField(
      Supplier<Pose2d> robotPoseSupplier, Supplier<Translation2d[]> noteSupplier) {
    final var field = new Field2d();
    SmartDashboard.putData(field);
    return Commands.run(
            () -> {
              field.setRobotPose(robotPoseSupplier.get());
              field
                  .getObject("notes")
                  .setPoses(
                      Arrays.stream(noteSupplier.get())
                          .map(translation2d -> new Pose2d(translation2d, new Rotation2d()))
                          .toList());
            })
        .ignoringDisable(true);
  }

  public static Command showAutoPlan(Supplier<String> autoConfigSupplier) {
    final var autoPlanField = new Field2d();
    SmartDashboard.putData("auto plan", autoPlanField);
    final var targetNotes = autoPlanField.getObject("target notes");
    final var shootingSpots = autoPlanField.getObject("shooting spots");
    final var obstacles = autoPlanField.getObject("obstacles");
    return Commands.run(
            () -> {
              if (DriverStation.isDisabled()) {
                final String configString = autoConfigSupplier.get();

                final var autoConfig = AutoConfigParser.parseAutoConfig(configString);
                if (autoConfig.isEmpty()) {
                  targetNotes.setPoses();
                  obstacles.setPoses();
                  shootingSpots.setPoses();
                  return;
                }
                final var autoParts = autoConfig.get();

                targetNotes.setPoses(
                    autoParts.stream()
                        .map(AutoConfigParser.AutoPart::note)
                        .filter(Optional::isPresent)
                        .map(Optional::get)
                        .map(note -> new Pose2d(note, Rotation2d.fromDegrees(90)))
                        .toArray(Pose2d[]::new));
                obstacles.setPoses(
                    autoParts.stream()
                        .map(AutoConfigParser.AutoPart::obstacles)
                        .filter(Optional::isPresent)
                        .map(Optional::get)
                        .flatMap(
                            obstacleList ->
                                obstacleList.stream()
                                    .map(
                                        corners ->
                                            corners.getFirst().plus(corners.getSecond()).div(2)))
                        .map(obstacle -> new Pose2d(obstacle, Rotation2d.fromDegrees(-90)))
                        .map(AllianceFlipUtil::apply)
                        .toArray(Pose2d[]::new));
                shootingSpots.setPoses(
                    autoParts.stream()
                        .map(AutoConfigParser.AutoPart::shootingTranslation)
                        .map(AutoConstants::getShootingPose2dFromTranslation)
                        .toArray(Pose2d[]::new));
              }
            })
        .ignoringDisable(true);
  }
}
