package frc.robot.subsystems.drive;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.TunableNumberWrapper;
import java.lang.invoke.MethodHandles;

/** All Constants Measured in Meters and Radians (m/s, m/s^2, rad/s, rad/s^2) */
public final class DriveConstants {
  private static final TunableNumberWrapper tunableTable =
      new TunableNumberWrapper(MethodHandles.lookup().lookupClass());

  public static final LoggedTunableNumber NOTE_PICKUP_MAX_SPEED =
      tunableTable.makeField("note pickup max speed", 3);
  public static final LoggedTunableNumber NOTE_PICKUP_MIN_SPEED =
      tunableTable.makeField("note pickup min speed", 1);

  public static final LoggedTunableNumber NOTE_PICKUP_FACING_NOTE_REQUIRED_DISTANCE =
      tunableTable.makeField("note pickup distance where need to be facing note", 1);
  public static final LoggedTunableNumber NOTE_PICKUP_SPEED_STARTING_DISTANCE =
      tunableTable.makeField("note pickup speed starting distance", 0.4);
  public static final LoggedTunableNumber NOTE_PICKUP_DISTANCE_TO_SPEED_MULT =
      tunableTable.makeField("note distance to speed mult", 1);
  public static DrivetrainConfig drivetrainConfig =
      switch (Constants.getRobot()) {
        default ->
            new DrivetrainConfig(
                Units.inchesToMeters(29.0), // Track width x
                Units.inchesToMeters(29.0), // Track width y
                1,
                1,
                Units.degreesToRadians(100), // Max angular velocity
                Units.degreesToRadians(100)); // Max angular acceleration
      };
  public static final PathConstraints pathPlannerConstraints =
      new PathConstraints(
          drivetrainConfig.maxLinearVelocity,
          drivetrainConfig.maxLinearAcceleration,
          drivetrainConfig.maxAngularVelocity,
          drivetrainConfig.maxAngularAcceleration);

  public static final Translation2d[] moduleTranslations =
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
        case SIMBOT -> 100;
        case COMPBOT -> 50.0;
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

  public static final boolean DISABLE_VISION_THETA = false; // true means we only use the gyro angle

  public static final int gyroID = 13;

  // Turn to "" for no canbus name
  public static final String canbus = "drive";

  public static ModuleConfig[] moduleConfigs =
      switch (Constants.getRobot()) {
        case COMPBOT ->
            new ModuleConfig[] {
              // Front left
              new ModuleConfig(
                  6,
                  5,
                  25,
                  Rotation2d.fromRotations(0.017578 + 0.5),
                  true,
                  Units.inchesToMeters(1.757)),
              // Front right
              new ModuleConfig(
                  8,
                  7,
                  27,
                  Rotation2d.fromRotations(-0.258301),
                  true,
                  Units.inchesToMeters(1.83625)),
              // Back left
              new ModuleConfig(
                  4,
                  3,
                  23,
                  Rotation2d.fromRotations(0.326904 + 0.5),
                  true,
                  Units.inchesToMeters(1.83625)),
              // Back right
              new ModuleConfig(
                  2, 1, 21, Rotation2d.fromRotations(0.133789), true, Units.inchesToMeters(1.78175))
            };
        case SIMBOT -> {
          ModuleConfig[] configs = new ModuleConfig[4];
          for (int i = 0; i < configs.length; i++)
            configs[i] =
                new ModuleConfig(0, 0, 0, new Rotation2d(0), false, Units.inchesToMeters(2.0));
          yield configs;
        }
      };

  public static final ModuleConstants moduleConstants =
      switch (Constants.getRobot()) {
        case COMPBOT ->
            new ModuleConstants(Mk4iReductions.L2.reduction, Mk4iReductions.TURN.reduction);
        case SIMBOT ->
            new ModuleConstants(Mk4iReductions.L2.reduction, Mk4iReductions.TURN.reduction);
      };

  public static class HeadingControllerConstants {
    public static final LoggedTunableNumber kP = tunableTable.makeField("headingController/kp", 1);
    public static final LoggedTunableNumber kD =
        tunableTable.makeField("headingController/kd", 0.0);
    public static final LoggedTunableNumber NOTE_PICKUP_MULT =
        tunableTable.makeField("headingController/note pickup mult", 1);
    public static final LoggedTunableNumber TOLERANCE =
        tunableTable.makeField("headingController/tolerance deg", 1);
    public static final LoggedTunableNumber NOTE_PICKUP_TOLERANCE =
        tunableTable.makeField("headingController/note pickup tolerance deg", 3);
    public static final LoggedTunableNumber SHOOTING_ANGLE_OFFSET_DEG =
        tunableTable.makeField("headingController/speaker shooting offset set", 1);

    public static final LoggedTunableNumber LOBBING_ANGLE_OFFSET_DEG =
        tunableTable.makeField("headingController/lobbing shooting offset set", 16);
  }

  public static final PIDConstants PPtranslationConstants =
      switch (Constants.getRobot()) {
        case COMPBOT -> new PIDConstants(3.0, 0.0, 0.0);
        case SIMBOT -> new PIDConstants(5, 0.0, 0.0);
      };

  public static final PIDConstants PProtationConstants =
      switch (Constants.getRobot()) {
        case COMPBOT -> new PIDConstants(1.0, 0.0, 0.0);
        case SIMBOT -> new PIDConstants(5, 0.0, 0.0);
      };

  public record DrivetrainConfig(
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

  public record ModuleConfig(
      int driveID,
      int turnID,
      int absoluteEncoderChannel,
      Rotation2d absoluteEncoderOffset,
      boolean turnMotorInverted,
      double wheelRadius) {}

  public record ModuleConstants(double driveReduction, double turnReduction) {}

  private enum Mk4iReductions {
    L1((50.0 / 14.0) * (19.0 / 25.0) * (45.0 / 15.0)),
    L2((50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0)),
    L3((50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0)),
    TURN((150.0 / 7.0));

    final double reduction;

    Mk4iReductions(double reduction) {
      this.reduction = reduction;
    }
  }
}
