package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.commands.auto.AutoConstants;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;

public class DriveToSpeaker {
  private DriveToSpeaker() {}

  private class Blue {
    public static Pose2d getShootingPose2dFromTranslation(Translation2d translation) {
      return new Pose2d(translation, getShootingAngleFromTranslation(translation));
    }

    public static Rotation2d getShootingAngleFromTranslation(Translation2d translation) {
      return translation
          .minus(FieldConstants.Speaker.centerSpeakerOpening.getTranslation())
          .getAngle();
    }

    private static final Pose2d LEFT =
        getShootingPose2dFromTranslation(AutoConstants.ShootingTranslations.A);
    private static final Pose2d CENTER =
        getShootingPose2dFromTranslation(AutoConstants.ShootingTranslations.B);
    private static final Pose2d RIGHT =
        getShootingPose2dFromTranslation(AutoConstants.ShootingTranslations.C);
  }

  private class Red {
    private static final Pose2d LEFT = AllianceFlipUtil.convertToRed(Blue.RIGHT);
    private static final Pose2d CENTER = AllianceFlipUtil.convertToRed(Blue.CENTER);
    private static final Pose2d RIGHT = AllianceFlipUtil.convertToRed(Blue.LEFT);
  }

  private static Command gotoShootingPose(Drive drive, Pose2d pose) {
    return DriveToPointBuilder.driveToAndAlign(
        drive,
        pose,
        AutoConstants.SHOOTING_DISTANCE_OFFSET_TOLERANCE.get(),
        AutoConstants.SHOOTING_ANGLE_OFFSET_TOLERANCE.get(),
        false);
  }

  public static Command left(Drive drive, ShooterSubsystem shooter, ArmSubsystem arm) {
    return builder(drive, shooter, arm, Red.LEFT, Blue.LEFT);
  }

  public static Command center(Drive drive, ShooterSubsystem shooter, ArmSubsystem arm) {
    return builder(drive, shooter, arm, Red.CENTER, Blue.CENTER);
  }

  public static Command right(Drive drive, ShooterSubsystem shooter, ArmSubsystem arm) {
    return builder(drive, shooter, arm, Red.RIGHT, Blue.RIGHT);
  }

  private static Command builder(
      Drive drive, ShooterSubsystem shooter, ArmSubsystem arm, Pose2d redPose, Pose2d bluePose) {
    return new ConditionalCommand(
            gotoShootingPose(drive, redPose),
            gotoShootingPose(drive, bluePose),
            AllianceFlipUtil::shouldFlip)
        .alongWith(
            ArmCommands.autoArmToPosition(arm, ArmConstants.Positions.SPEAKER_POS_RAD::get)
                .alongWith(
                    ShooterCommands.runSpeed(
                        shooter, ShooterConstants.SPEAKER_VELOCITY_RAD_PER_SEC::get)));
  }
}
