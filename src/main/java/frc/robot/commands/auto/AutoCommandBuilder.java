package frc.robot.commands.auto;

import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.NoteVisionSubsystem;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.AutoConfigParser;
import frc.robot.util.FieldConstants;
import frc.robot.util.ShooterStateHelpers;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AutoCommandBuilder {
  private final Drive drive;
  private final NoteVisionSubsystem noteVision;
  private final ShooterSubsystem shooter;
  private final Intake intake;
  private final ArmSubsystem arm;
  private final BooleanSupplier hasNote;
  private final ShooterStateHelpers shooterStateHelpers;

  public AutoCommandBuilder(
      Drive drive,
      NoteVisionSubsystem noteVision,
      ShooterSubsystem shooter,
      Intake intake,
      ArmSubsystem arm,
      BooleanSupplier hasNote,
      ShooterStateHelpers shooterStateHelpers) {
    this.drive = drive;
    this.noteVision = noteVision;
    this.shooter = shooter;
    this.intake = intake;
    this.arm = arm;
    this.hasNote = hasNote;
    this.shooterStateHelpers = shooterStateHelpers;
  }

  public Command pickupNoteVisibleNote() {
    return new PickUpNoteCommand(drive, intake, noteVision::getCurrentNote, hasNote);
  }

  public Command pickupSuppliedNote(Supplier<Optional<Translation2d>> relativeNoteSupplier) {
    return new PickUpNoteCommand(drive, intake, relativeNoteSupplier, hasNote);
  }

  public Command fallbackPickup() {
    return pickupSuppliedNote(
        () ->
            noteVision.getClosestNoteFiltered(
                note ->
                    AllianceFlipUtil.apply(note).getX()
                        < FieldConstants.StagingLocations.centerlineX
                            + AutoConstants.AutoNoteOffsetThresholds.FALLBACK_MAX_PAST_CENTER
                                .get()));
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
            pickupSuppliedNote(
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

  public Command driveAndPickupNoteAuto(
      Translation2d note, Pose2d pickupLocation, double pickupTimeout) {
    final Command driveToPickup =
        DriveToPointBuilder.driveToNoFlip(pickupLocation, 4)
            .until(
                () ->
                    drive.getPose().getTranslation().getDistance(pickupLocation.getTranslation())
                            < AutoConstants.DRIVE_TO_PICKUP_INTERRUPT_DISTANCE.get()
                        && getVisionNoteByTranslation(
                                note, AutoConstants.AutoNoteOffsetThresholds.WHILE_ROUTING.get())
                            .isPresent());
    return driveToPickup.andThen(pickupNoteAtTranslation(note, pickupTimeout));
  }

  public Command autoFromConfigPart(AutoConfigParser.AutoPart autoPart) {
    final Command setObstacles =
        Commands.runOnce(
            () -> {
              if (autoPart.obstacles().isPresent()) {
                setObstacles(autoPart.obstacles().get());
              }
            });
    final Command pickupCommand =
        (autoPart.notePickupPose().isEmpty()
                ? pickupNoteAtTranslation(autoPart.note(), AutoConstants.PICKUP_TIMEOUT.get())
                : driveAndPickupNoteAuto(
                    autoPart.note(),
                    autoPart.notePickupPose().get(),
                    AutoConstants.PICKUP_TIMEOUT.get()))
            .andThen(fallbackPickup().onlyIf(() -> !hasNote.getAsBoolean()));
    final Command returnCommand =
        DriveToPointBuilder.driveToAndAlign(
            drive,
            AutoConstants.getShootingPose2dFromTranslation(autoPart.shootingTranslation()),
            AutoConstants.SHOOTING_DISTANCE_OFFSET_TOLERANCE.get(),
            AutoConstants.SHOOTING_ANGLE_OFFSET_TOLERANCE.get(),
            false);

    return Commands.sequence(
        setObstacles, pickupCommand, readyShooter(), returnCommand, autoShoot());
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
    return ShooterCommands.autoShoot(shooterStateHelpers, intake, hasNote, arm);
  }

  public Command readyShooter() {
    return ArmCommands.autoArmToPosition(arm, ArmConstants.Positions.SPEAKER_POS_RAD::get)
        .andThen(
            Commands.runOnce(
                () -> shooter.runVelocity(ShooterConstants.SPEAKER_VELOCITY_RAD_PER_SEC.get()),
                shooter));
  }

  private void setObstacles(List<Pair<Translation2d, Translation2d>> zones) {
    Pathfinding.setDynamicObstacles(
        AutoConstants.createDynamicObstaclesList(zones), drive.getPose().getTranslation());
  }

  public void clearObstacles() {
    Pathfinding.setDynamicObstacles(
        AutoConstants.createDynamicObstaclesList(List.of()), drive.getPose().getTranslation());
  }
}
