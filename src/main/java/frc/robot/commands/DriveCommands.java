// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands;

import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveController;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double X_RATE_LIMIT = 7.75;
  private static final double Y_RATE_LIMIT = 7.75;

  private DriveCommands() {}

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DriveController driveMode,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {

    final SlewRateLimiter translateXRateLimiter = new SlewRateLimiter(X_RATE_LIMIT);
    final SlewRateLimiter translateYRateLimiter = new SlewRateLimiter(Y_RATE_LIMIT);

    return Commands.run(
        () -> {
          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(
                      translateXRateLimiter.calculate(xSupplier.getAsDouble()),
                      translateYRateLimiter.calculate(ySupplier.getAsDouble())),
                  DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          final Optional<Supplier<Rotation2d>> headingSupplier = driveMode.getHeadingSupplier();
          double omega;
          Logger.recordOutput("heading control on", headingSupplier.isPresent());
          if (headingSupplier.isPresent()) {
            final var targetAngle = headingSupplier.get().get();
            Logger.recordOutput("drive heading", targetAngle);
            omega =
                drive
                    .getThetaController()
                    .calculate(
                        drive.getPose().getRotation().getRadians(),
                        headingSupplier.get().get().getRadians());
            if (drive.getThetaController().atGoal()) {
              omega = 0;
            }
          } else {
            omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);
            omega = Math.copySign(omega * omega, omega);
            omega *= drivetrainConfig.maxAngularVelocity();
          }
          Logger.recordOutput("omega", omega);

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          // Convert to field relative speeds & send command
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * drivetrainConfig.maxLinearVelocity(),
                  linearVelocity.getY() * drivetrainConfig.maxLinearVelocity(),
                  omega,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }
}
