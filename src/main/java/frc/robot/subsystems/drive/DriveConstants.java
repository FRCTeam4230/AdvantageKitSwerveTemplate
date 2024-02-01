package frc.robot.subsystems.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import java.util.function.DoubleFunction;
import lombok.Builder;
import lombok.Getter;

/** All Constants Measured in Meters and Radians (m/s, m/s^2, rad/s, rad/s^2) */
public final class DriveConstants {
  @Getter
  private static final DrivetrainConfig drivetrainConfig =
      // Everything is the same. if we have a different/sim/real then add more here
      switch (Constants.getRobot()) {
        default ->
            new DrivetrainConfig(
                Units.inchesToMeters(2.0),
                Units.inchesToMeters(26.0),
                Units.inchesToMeters(26.0),
                Units.feetToMeters(12.16),
                Units.feetToMeters(21.32),
                7.93,
                29.89);
      };

  public static final double wheelRadius = Units.inchesToMeters(2.0);
  static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(
            drivetrainConfig.trackwidthX() / 2.0, drivetrainConfig.trackwidthY() / 2.0),
        new Translation2d(
            drivetrainConfig.trackwidthX() / 2.0, -drivetrainConfig.trackwidthY() / 2.0),
        new Translation2d(
            -drivetrainConfig.trackwidthX() / 2.0, drivetrainConfig.trackwidthY() / 2.0),
        new Translation2d(
            -drivetrainConfig.trackwidthX() / 2.0, -drivetrainConfig.trackwidthY() / 2.0)
      };
  public static final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(moduleTranslations);
  public static final double odometryFrequency =
      switch (Constants.getRobot()) {
        case SIMBOT -> 50.0;
        case COMPBOT -> 250.0;
      };
  public static final Matrix<N3, N1> stateStdDevs =
      switch (Constants.getRobot()) {
        default -> new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.0002));
      };
  public static final double xyStdDevCoefficient =
      switch (Constants.getRobot()) {
        default -> 0.01;
      };
  public static final double thetaStdDevCoefficient =
      switch (Constants.getRobot()) {
        default -> 0.01;
      };

  public static final int gyroID = 13;

  // Turn to "" for no canbus name
  public static final String canbus = "chassis";

  private static final DoubleFunction<Rotation2d> calculateOffset =
      offset -> {
        return Rotation2d.fromRotations(offset).plus(Rotation2d.fromDegrees(180));
      };
  public static ModuleConfig[] moduleConfigs =
      switch (Constants.getRobot()) {
        case COMPBOT ->
            new ModuleConfig[] {
              ModuleConfig.builder()
                  .withDriveID(1)
                  .withTurnID(2)
                  .withAbsoluteEncoderChannel(9)
                  .withAbsoluteEncoderOffset(calculateOffset.apply(-0.383))
                  .withTurnMotorInverted(true)
                  .build(),
              ModuleConfig.builder()
                  .withDriveID(3)
                  .withTurnID(4)
                  .withAbsoluteEncoderChannel(10)
                  .withAbsoluteEncoderOffset(calculateOffset.apply(-0.251))
                  .withTurnMotorInverted(true)
                  .build(),
              ModuleConfig.builder()
                  .withDriveID(5)
                  .withTurnID(6)
                  .withAbsoluteEncoderChannel(11)
                  .withAbsoluteEncoderOffset(calculateOffset.apply(-0.057))
                  .withTurnMotorInverted(true)
                  .build(),
              ModuleConfig.builder()
                  .withDriveID(7)
                  .withTurnID(8)
                  .withAbsoluteEncoderChannel(12)
                  .withAbsoluteEncoderOffset(calculateOffset.apply(-0.470))
                  .withTurnMotorInverted(true)
                  .build()
            };
        case SIMBOT -> {
          ModuleConfig[] configs = new ModuleConfig[4];
          for (int i = 0; i < configs.length; i++)
            configs[i] =
                ModuleConfig.builder()
                    .withDriveID(0)
                    .withTurnID(0)
                    .withAbsoluteEncoderChannel(0)
                    .withAbsoluteEncoderOffset(new Rotation2d(0))
                    .withTurnMotorInverted(false)
                    .build();

          yield configs;
        }
      };

  public static final ModuleConstants moduleConstants =
      switch (Constants.getRobot()) {
        case COMPBOT ->
            new ModuleConstants(
                0.1,
                0.13,
                0.1,
                0.0,
                10.0,
                0.0,
                Mk4iReductions.L2.reduction,
                Mk4iReductions.TURN.reduction);
        case SIMBOT ->
            new ModuleConstants(
                0.014,
                0.134,
                0.1,
                0.0,
                10.0,
                0.0,
                Mk4iReductions.L2.reduction,
                Mk4iReductions.TURN.reduction);
      };

  @Getter
  private static final HeadingControllerConstants headingControllerConstants =
      switch (Constants.getRobot()) {
        case COMPBOT -> new HeadingControllerConstants(8.0, 0.0);
        case SIMBOT -> new HeadingControllerConstants(4.0, 0.0);
      };

  @Builder(setterPrefix = "with")
  public record DrivetrainConfig(
      double wheelRadius,
      double trackwidthX,
      double trackwidthY,
      double maxLinearVelocity,
      double maxLinearAcceleration,
      double maxAngularVelocity,
      double maxAngularAcceleration) {
    public double driveBaseRadius() {
      return Math.hypot(trackwidthX / 2.0, trackwidthY / 2.0);
    }
  }

  @Builder(setterPrefix = "with")
  public record ModuleConfig(
      int driveID,
      int turnID,
      int absoluteEncoderChannel,
      Rotation2d absoluteEncoderOffset,
      boolean turnMotorInverted) {}

  @Builder(setterPrefix = "with")
  public record ModuleConstants(
      double ffKs,
      double ffKv,
      double driveKp,
      double drivekD,
      double turnKp,
      double turnkD,
      double driveReduction,
      double turnReduction) {}

  @Builder(setterPrefix = "with")
  public record HeadingControllerConstants(double Kp, double Kd) {}

  private enum Mk4iReductions {
    L2((50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0)),
    L3((50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0)),
    TURN((150.0 / 7.0));

    final double reduction;

    Mk4iReductions(double reduction) {
      this.reduction = reduction;
    }
  }
}
