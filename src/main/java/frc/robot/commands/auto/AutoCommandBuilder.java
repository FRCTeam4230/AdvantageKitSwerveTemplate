package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.*;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.beamBreak.BeamBreak;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.NoteVisionConstants;
import frc.robot.subsystems.vision.NoteVisionSubsystem;
import frc.robot.util.ShooterStateHelpers;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AutoCommandBuilder {
  private final Drive drive;
  private final NoteVisionSubsystem noteVision;
  private final ShooterSubsystem shooter;
  private final Intake intake;
  private final ArmSubsystem arm;
  private final BeamBreak beamBreak;
  private final ShooterStateHelpers shooterStateHelpers;

  public AutoCommandBuilder(
      Drive drive,
      NoteVisionSubsystem noteVision,
      ShooterSubsystem shooter,
      Intake intake,
      ArmSubsystem arm,
      BeamBreak beamBreak,
      ShooterStateHelpers shooterStateHelpers) {
    this.drive = drive;
    this.noteVision = noteVision;
    this.shooter = shooter;
    this.intake = intake;
    this.arm = arm;
    this.beamBreak = beamBreak;
    this.shooterStateHelpers = shooterStateHelpers;
  }

  public Command driveIntoVisibleNote() {
    final Command output =
        new DriveIntoNoteCommand(
            drive,
            noteVision::getCurrentNote,
            beamBreak::detectNote,
            () -> arm.getPositionRad() < NoteVisionConstants.MAX_ARM_POS_RAD);
    output.addRequirements(noteVision);
    return output;
  }

  public Command intakeByVisionNote(Supplier<Optional<Translation2d>> noteSupplier) {
    return IntakeCommands.manualIntakeCommand(intake, IntakeConstants.INTAKE_VOLTAGE::get)
        .until(beamBreak::detectNote);
  }

  public Command pickupVisibleNote() {
    return driveIntoVisibleNote().alongWith(intakeByVisionNote(noteVision::getCurrentNote));
  }

  public Command pickupSuppliedNote(Supplier<Optional<Translation2d>> relativeNoteSupplier) {
    return pickupSuppliedNote(relativeNoteSupplier, 0);
  }

  public Command pickupSuppliedNoteFailWithNoNote(
      Supplier<Optional<Translation2d>> relativeNoteSupplier) {
    return pickupSuppliedNote(relativeNoteSupplier)
        .until(() -> relativeNoteSupplier.get().isEmpty());
  }

  public Command pickupSuppliedNote(
      Supplier<Optional<Translation2d>> relativeNoteSupplier, double scanRadPerSec) {
    final Command out =
        new DriveIntoNoteCommand(
                drive,
                relativeNoteSupplier,
                beamBreak::detectNote,
                () -> arm.getPositionRad() < NoteVisionConstants.MAX_ARM_POS_RAD,
                scanRadPerSec)
            .alongWith(intakeByVisionNote(relativeNoteSupplier));
    out.addRequirements(noteVision);
    return out;
  }

  private Optional<Translation2d> getVisionNoteByTranslation(Translation2d note, double threshold) {
    Logger.recordOutput("auto/targetNote", new Pose2d(note, new Rotation2d()));
    final var output = noteVision.getNoteByPosition(note, threshold);
    if (output.isPresent()) {
      Logger.recordOutput("auto/foundNote", new Pose2d(output.get(), Rotation2d.fromDegrees(45)));
    } else {
      Logger.recordOutput(
          "auto/foundNote", new Pose2d(new Translation2d(), Rotation2d.fromDegrees(45)));
    }
    return output;
  }

  public Command driveToPickup(Pose2d pickupLocation) {
    return DriveToPointBuilder.driveToNoFlip(pickupLocation, 4);
  }

  /** assumes the shooter is at the correct speed and the arm is in the correct position */
  public Command autoShoot() {
    return ShooterCommands.autoShoot(shooterStateHelpers, intake, beamBreak::detectNote);
  }

  private Command dropArm() {
    return ArmCommands.autoArmToPosition(arm, ArmConstants.Positions.INTAKE_POS_RAD::get);
  }

  public Command readyShooter() {
    return ArmCommands.autoArmToPosition(arm, ArmConstants.Positions.SPEAKER_POS_RAD::get)
        .andThen(
            Commands.runOnce(
                () -> shooter.runVelocity(ShooterConstants.SPEAKER_VELOCITY_RAD_PER_SEC.get()),
                shooter));
  }

  private Command logAutoState(String state) {
    return Commands.runOnce(() -> Logger.recordOutput("auto/state", state));
  }
}
