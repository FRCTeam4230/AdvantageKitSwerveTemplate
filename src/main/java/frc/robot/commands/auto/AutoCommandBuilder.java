package frc.robot.commands.auto;

import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.AutoConfigParser;
import frc.robot.util.FieldConstants;
import frc.robot.util.ShooterStateHelpers;
import java.util.List;
import java.util.Optional;
import java.util.Set;
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
    return IntakeCommands.manualIntakeCommand(
            intake,
            () -> {
              final var targetNote = noteSupplier.get();
              return (targetNote.isEmpty() || targetNote.get().getNorm() > 2)
                  ? 0
                  : IntakeConstants.INTAKE_VOLTAGE.get();
            })
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

  public Command fallbackPickup() {
    return pickupSuppliedNote(
        () ->
            noteVision.getClosestNoteFiltered(
                note ->
                    AllianceFlipUtil.apply(note).getX()
                        < FieldConstants.StagingLocations.centerlineX
                            + AutoConstants.AutoNoteOffsetThresholds.FALLBACK_MAX_PAST_CENTER
                                .get()),
        2);
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

  public Command pickupNoteAtTranslation(Translation2d note, double timeout) {
    return Commands.runOnce(
            () -> noteVision.setVirtualAutoNote(note, AutoConstants.DISTANCE_TO_TRUST_CAMERA.get()),
            noteVision)
        .andThen(
            pickupSuppliedNoteFailWithNoNote(
                () -> {
                  var trackedNote = noteVision.getVirtualAutoNote();

                  if (trackedNote.isEmpty()) {
                    trackedNote =
                        getVisionNoteByTranslation(
                            note,
                            AutoConstants.AutoNoteOffsetThresholds.WHILE_ATTEMPTING_PICKUP.get());
                  }

                  return trackedNote.map(
                      translation2d ->
                          NoteVisionSubsystem.deprojectProjectedNoteFromRobotPose(
                              translation2d, drive.getPose()));
                }))
        .withTimeout(timeout);
  }

  public Command driveToPickup(Pose2d pickupLocation) {
    return DriveToPointBuilder.driveToNoFlip(pickupLocation, 4);
  }

  public Command autoFromConfigPart(AutoConfigParser.AutoPart autoPart) {
    final SequentialCommandGroup output = new SequentialCommandGroup();

    output.addCommands(dropArm());

    final Command setObstacles =
        Commands.runOnce(
            () -> {
              if (autoPart.obstacles().isPresent()) {
                setObstacles(autoPart.obstacles().get());
              }
            });
    output.addCommands(setObstacles);

    if (autoPart.notePickupPose().isPresent()) {
      final Command driveToPickup = driveToPickup(autoPart.notePickupPose().get());

      output.addCommands(Commands.parallel(logAutoState("driving"), driveToPickup));
    }

    if (autoPart.note().isPresent()) {
      final Command pickupNote =
          pickupNoteAtTranslation(autoPart.note().get(), AutoConstants.PICKUP_TIMEOUT.get());

      output.addCommands(Commands.parallel(logAutoState("pickup"), pickupNote));
    }

    final Command fallbackPickup = fallbackPickup().onlyIf(() -> !beamBreak.detectNote());
    output.addCommands(Commands.parallel(logAutoState("fallback"), fallbackPickup));
    final Pose2d shootingPose =
        AutoConstants.getShootingPose2dFromTranslation(autoPart.shootingTranslation());
    final Command returnCommand =
        DriveToPointBuilder.driveToAndAlign(
                drive,
                shootingPose,
                AutoConstants.SHOOTING_DISTANCE_OFFSET_TOLERANCE.get(),
                AutoConstants.SHOOTING_ANGLE_OFFSET_TOLERANCE.get(),
                false)
            .deadlineWith(IntakeCommands.keepNoteInCenter(intake, beamBreak));
    output.addCommands(
        Commands.parallel(
            logAutoState("returning"),
            Commands.sequence(returnCommand, logAutoState("shooting"), autoShoot())
                .deadlineWith(readyShooterDistance(shootingPose))));

    return output;
  }

  public Command autoFromConfigString(Supplier<String> configStringSupplier) {
    return autoFromConfig(() -> AutoConfigParser.parseAutoConfig(configStringSupplier.get()));
  }

  public Command autoFromConfig(
      Supplier<Optional<List<AutoConfigParser.AutoPart>>> configSupplier) {
    return Commands.runOnce(this::clearObstacles)
        .andThen(initialFullShot())
        .andThen(
            new DeferredCommand(
                () -> {
                  final var config = configSupplier.get();
                  if (config.isEmpty()) {
                    return Commands.none();
                  }

                  final var commands =
                      config.get().stream().map(this::autoFromConfigPart).toArray(Command[]::new);

                  return Commands.sequence(commands).asProxy();
                },
                Set.of()));
  }

  public Command initialFullShot() {
    return readyShooter().andThen(autoShoot());
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

  public Command readyShooterDistance(Pose2d shootingPose) {
    return MultiDistanceShot.forSpeaker(() -> shootingPose, shooter, arm);
  }

  private void setObstacles(List<Pair<Translation2d, Translation2d>> zones) {
    Pathfinding.setDynamicObstacles(
        AutoConstants.createDynamicObstaclesList(zones), drive.getPose().getTranslation());
  }

  public void clearObstacles() {
    Pathfinding.setDynamicObstacles(
        AutoConstants.createDynamicObstaclesList(List.of()), drive.getPose().getTranslation());
  }

  private Command logAutoState(String state) {
    return Commands.runOnce(() -> Logger.recordOutput("auto/state", state));
  }
}
