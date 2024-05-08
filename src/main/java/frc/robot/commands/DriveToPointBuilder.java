// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import java.util.function.Supplier;

public class DriveToPointBuilder {
  private DriveToPointBuilder() {}

  public static Command driveTo(Pose2d targetPose) {
    return driveTo(targetPose, 0);
  }

  public static Command driveTo(Pose2d targetPose, double endVelocity) {
    return AutoBuilder.pathfindToPoseFlipped(
        targetPose, DriveConstants.pathPlannerConstraints, endVelocity, 0.0);
  }

  public static Command driveToNoFlip(Pose2d targetPose) {
    return driveToNoFlip(targetPose, 0);
  }

  public static Command driveToNoFlip(Pose2d targetPose, double endVelocity) {
    return AutoBuilder.pathfindToPose(
        targetPose, DriveConstants.pathPlannerConstraints, endVelocity, 0.2);
  }

  public static Command align(
      Drive drive,
      Pose2d targetPose,
      double distanceTolerance,
      double angleToleranceRad,
      boolean flip) {
    return Commands.runEnd(
            () -> {
              final var flippedTargetPose = targetPose;
              final var pos = drive.getPose();

              final var translationOffset =
                  flippedTargetPose.getTranslation().minus(pos.getTranslation());
              final var rotationOffset = flippedTargetPose.getRotation().minus(drive.getRotation());

              var omega = drive.getThetaController().calculate(0, rotationOffset.getRadians());
              final var speedX =
                  DriveConstants.PPtranslationConstants.kP * translationOffset.getX();
              final var speedY =
                  DriveConstants.PPtranslationConstants.kP * translationOffset.getY();

              if (drive.getThetaController().atSetpoint()) {
                omega = 0;
              }

              final var chassisSpeeds =
                  ChassisSpeeds.fromFieldRelativeSpeeds(speedX, speedY, omega, pos.getRotation());

              drive.runVelocity(chassisSpeeds);
            },
            drive::stop,
            drive)
        .until(
            () -> {
              final var flippedTargetPose = targetPose;
              final var pos = drive.getPose();
              return pos.getTranslation().getDistance(flippedTargetPose.getTranslation())
                      < distanceTolerance
                  && Math.abs(pos.getRotation().minus(flippedTargetPose.getRotation()).getRadians())
                      < angleToleranceRad;
            });
  }

  public static Command driveToAndAlign(
      Drive drive,
      Pose2d targetPose,
      double distanceTolerance,
      double angleToleranceRad,
      boolean flip) {
    final Command pathfindingPart = flip ? driveTo(targetPose) : driveToNoFlip(targetPose);
    return pathfindingPart.andThen(
        align(drive, targetPose, distanceTolerance, angleToleranceRad, flip));
  }

  public static Command waitUntilNearPose(
      Supplier<Pose2d> robotPoseSupplier, Supplier<Pose2d> targetPose, double distanceThreshold) {
    return Commands.waitUntil(
        () ->
            targetPose.get().getTranslation().getDistance(robotPoseSupplier.get().getTranslation())
                < distanceThreshold);
  }
}
