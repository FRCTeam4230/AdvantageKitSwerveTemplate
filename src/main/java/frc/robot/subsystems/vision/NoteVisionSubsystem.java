package frc.robot.subsystems.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.auto.AutoConstants;
import frc.robot.util.FieldConstants;
import frc.robot.util.PoseLog;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Predicate;
import java.util.function.Supplier;
import java.util.stream.IntStream;
import lombok.Data;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class NoteVisionSubsystem extends SubsystemBase {
  @Data
  private static class NoteCamera {
    private final NoteVisionIO io;
    private final NoteVisionIOInputsAutoLogged inputs;
    private final NoteVisionConstants.CameraConfig config;
    private double lastTimestamp;
  }

  private final NoteCamera[] noteVisionIOsAndInputs;

  private TimestampedNote[] notesInOdometrySpace = {};

  private final PoseLog noVisionPoseLog;
  private final Supplier<Pose2d> currentRobotPoseNoVisionSupplier;
  private final Supplier<Pose2d> currentRobotVisionFieldPoseSupplier;
  private final DoubleSupplier armPositionSupplierRad;

  /** Position of the simulated note for auto in field space */
  @Getter private Optional<Translation2d> virtualAutoNote = Optional.empty();

  private int virtualAutoNoteCoveredFrameCount = 0;

  private double virtualNoteDistanceToRemove;

  public record TimestampedNote(Translation2d pose, double timestamp) {}

  /**
   * @param noteVisionIOs expected to be in the same order and length as
   *     NoteVisionConstants.CAMERA_CONFIGS
   */
  public NoteVisionSubsystem(
      NoteVisionIO[] noteVisionIOs,
      PoseLog noVisionPoseLog,
      Supplier<Pose2d> currentRobotPoseNoVisionSupplier,
      Supplier<Pose2d> currentRobotVisionFieldPoseSupplier,
      DoubleSupplier armPositionSupplierRad) {
    this.noteVisionIOsAndInputs =
        IntStream.range(0, noteVisionIOs.length)
            .mapToObj(
                i ->
                    new NoteCamera(
                        noteVisionIOs[i],
                        new NoteVisionIOInputsAutoLogged(),
                        NoteVisionConstants.CAMERA_CONFIGS[i]))
            .toArray(NoteCamera[]::new);
    this.noVisionPoseLog = noVisionPoseLog;
    this.currentRobotPoseNoVisionSupplier = currentRobotPoseNoVisionSupplier;
    this.currentRobotVisionFieldPoseSupplier = currentRobotVisionFieldPoseSupplier;
    this.armPositionSupplierRad = armPositionSupplierRad;
  }

  @Override
  public void periodic() {
    for (var noteVisionIOAndInputs : noteVisionIOsAndInputs) {
      noteVisionIOAndInputs.io.updateInputs(noteVisionIOAndInputs.inputs);
      Logger.processInputs(
          "NoteVision/" + noteVisionIOAndInputs.inputs.name, noteVisionIOAndInputs.inputs);
    }

    expireNotes();

    for (int i = 0; i < noteVisionIOsAndInputs.length; i++) {
      if (noteVisionIOsAndInputs[i].inputs.timeStampSeconds
          == noteVisionIOsAndInputs[i].lastTimestamp) {
        continue;
      } else {
        noteVisionIOsAndInputs[i].lastTimestamp = noteVisionIOsAndInputs[i].inputs.timeStampSeconds;
      }

      if (noteVisionIOsAndInputs[i].config.onArm()
          && armPositionSupplierRad.getAsDouble() > NoteVisionConstants.MAX_ARM_POS_RAD) {
        noteVisionIOsAndInputs[i].lastTimestamp = noteVisionIOsAndInputs[i].inputs.timeStampSeconds;
        continue;
      }

      var oldNotes = notesInOdometrySpace;
      var newNotes =
          calculateNotesInOdometrySpace(
              noteVisionIOsAndInputs[i].inputs,
              NoteVisionConstants.CAMERA_CONFIGS[i].cameraPose(),
              noVisionPoseLog.getPoseAtTime(noteVisionIOsAndInputs[i].inputs.timeStampSeconds));

      ArrayList<TimestampedNote> oldNotesInCamera = new ArrayList<>();
      ArrayList<TimestampedNote> oldNotesOutOfCamera = new ArrayList<>();

      splitOldNotesInCameraView(
          noVisionPoseLog.getPoseAtTime(noteVisionIOsAndInputs[i].inputs.timeStampSeconds),
          noteVisionIOsAndInputs[i].config,
          oldNotes,
          oldNotesInCamera,
          oldNotesOutOfCamera);

      var currentNotes =
          updateNotes(
              oldNotesInCamera,
              groupNotes(newNotes),
              noteVisionIOsAndInputs[i].inputs.timeStampSeconds);

      currentNotes.addAll(oldNotesOutOfCamera);

      groupNoteRecords(currentNotes);

      notesInOdometrySpace = currentNotes.toArray(TimestampedNote[]::new);

      checkAutoNote(noteVisionIOsAndInputs[i].config);
    }
  }

  private void expireNotes() {
    notesInOdometrySpace =
        Arrays.stream(notesInOdometrySpace)
            .filter(note -> note.timestamp > Logger.getTimestamp() / 1e6 - getCurrentExpiration())
            .toArray(TimestampedNote[]::new);
  }

  private double getCurrentExpiration() {
    return Optional.ofNullable(getCurrentCommand()).isPresent()
        ? NoteVisionConstants.NOTE_EXPIRATION
        : NoteVisionConstants.IDLE_NOTE_EXPIRATION;
  }

  private static void splitOldNotesInCameraView(
      Pose2d robotPose,
      NoteVisionConstants.CameraConfig config,
      TimestampedNote[] oldNotes,
      List<TimestampedNote> inCameraRange,
      List<TimestampedNote> outOfCameraRange) {
    for (var note : oldNotes) {
      if (canCameraSeeNote(
          config,
          deprojectProjectedNoteFromRobotPose(note.pose, robotPose),
          NoteVisionConstants.MAX_CAMERA_DISTANCE)) {
        inCameraRange.add(note);
      } else {
        outOfCameraRange.add(note);
      }
    }
  }

  private static boolean canCameraSeeNote(
      NoteVisionConstants.CameraConfig config, Translation2d note, double maxRange) {
    final var cameraPose =
        new Pose2d(
            config.cameraPose().getTranslation().toTranslation2d(),
            config.cameraPose().getRotation().toRotation2d());
    double distance = note.getDistance(cameraPose.getTranslation());
    if (distance > maxRange || distance < config.minDistance()) {
      return false;
    }
    Rotation2d angle = deprojectProjectedNoteFromRobotPose(note, cameraPose).getAngle();
    return MathUtil.isNear(0, angle.getDegrees(), NoteVisionConstants.LIFECAM_3000_HFOV / 2);
  }

  private static ArrayList<TimestampedNote> updateNotes(
      List<TimestampedNote> oldNotesInCamera,
      List<Translation2d> newNotes,
      double timeStampSeconds) {

    for (var oldNote : oldNotesInCamera) {
      for (int i = 0; i < newNotes.size(); i++) {
        if (areNotesSame(oldNote.pose, newNotes.get(i))) {
          newNotes.set(i, averageNotes(oldNote.pose, newNotes.get(i)));
          break;
        }
      }
    }

    var currentNotes =
        new ArrayList<>(
            newNotes.stream().map(note -> new TimestampedNote(note, timeStampSeconds)).toList());

    var recentNotes =
        oldNotesInCamera.stream()
            .filter(
                note ->
                    note.timestamp > timeStampSeconds - NoteVisionConstants.IN_CAMERA_EXPIRATION)
            .toList();

    currentNotes.addAll(recentNotes);

    return currentNotes;
  }

  private static boolean areNotesSame(Translation2d a, Translation2d b) {
    return a.getDistance(b) < NoteVisionConstants.NOTE_GROUPING_TOLERANCE;
  }

  private static Translation2d averageNotes(Translation2d a, Translation2d b) {
    return a.plus(b).div(2);
  }

  private static TimestampedNote averageNotes(TimestampedNote a, TimestampedNote b) {
    return new TimestampedNote(a.pose.plus(b.pose).div(2), Math.max(a.timestamp, b.timestamp));
  }

  private static void groupNoteRecords(ArrayList<TimestampedNote> notes) {
    outer:
    for (int i = 0; i < notes.size() - 1; ) {
      for (int j = i + 1; j < notes.size(); j++) {
        if (areNotesSame(notes.get(i).pose, notes.get(j).pose)) {
          notes.set(i, averageNotes(notes.get(i), notes.get(j)));
          notes.remove(j);
          continue outer;
        }
      }

      i++;
    }
  }

  private static List<Translation2d> groupNotes(List<Translation2d> noteTranslations) {
    ArrayList<Translation2d> notes = new ArrayList<>(noteTranslations);

    outer:
    for (int i = 0; i < notes.size() - 1; ) {
      for (int j = i + 1; j < notes.size(); j++) {
        if (areNotesSame(notes.get(i), notes.get(j))) {
          notes.set(i, averageNotes(notes.get(i), notes.get(j)));
          notes.remove(j);
          continue outer;
        }
      }

      i++;
    }

    return notes;
  }

  private static List<Translation2d> calculateNotesInOdometrySpace(
      NoteVisionIO.NoteVisionIOInputs inputs, Transform3d cameraPose, Pose2d oldRobotPose) {
    final ArrayList<Translation2d> notes = new ArrayList<>();

    for (int i = 0; i < inputs.notePitches.length; i++) {
      double noteAngle = inputs.notePitches[i] + cameraPose.getRotation().getY();

      if (noteAngle <= 0) {
        continue;
      }

      double distanceFromCamera =
          cameraPose.getZ() / Math.tan(noteAngle) / Math.cos(inputs.noteYaws[i]);

      Translation2d noteToCameraPose =
          new Translation2d(distanceFromCamera, new Rotation2d(inputs.noteYaws[i]));

      var noteRelativeToOldRobot =
          noteToCameraPose
              .rotateBy(cameraPose.getRotation().toRotation2d())
              .plus(cameraPose.getTranslation().toTranslation2d());
      var noteInOdometrySpace =
          projectRelativeNotePoseOntoRobotPose(noteRelativeToOldRobot, oldRobotPose);

      notes.add(noteInOdometrySpace);
    }

    return notes;
  }

  public Translation2d[] getNotesInRelativeSpace() {
    return Arrays.stream(notesInOdometrySpace)
        .map(
            odometrySpaceNote ->
                deprojectProjectedNoteFromRobotPose(
                    odometrySpaceNote.pose, currentRobotPoseNoVisionSupplier.get()))
        .filter(this::isRelativeNoteInsideField)
        .toArray(Translation2d[]::new);
  }

  public boolean isRelativeNoteInsideField(Translation2d note) {
    return isNoteInsideField(
        projectRelativeNotePoseOntoRobotPose(note, currentRobotVisionFieldPoseSupplier.get()));
  }

  public static boolean isNoteInsideField(Translation2d globalNote) {
    final double x = globalNote.getX();
    final double y = globalNote.getY();
    return x > -NoteVisionConstants.INSIDE_FIELD_TOLERANCE
        && x < FieldConstants.fieldLength + NoteVisionConstants.INSIDE_FIELD_TOLERANCE
        && y > -NoteVisionConstants.INSIDE_FIELD_TOLERANCE
        && y < FieldConstants.fieldWidth + NoteVisionConstants.INSIDE_FIELD_TOLERANCE;
  }

  public static Translation2d projectRelativeNotePoseOntoRobotPose(
      Translation2d noteInRelativeSpace, Pose2d robotPose) {
    return noteInRelativeSpace.rotateBy(robotPose.getRotation()).plus(robotPose.getTranslation());
  }

  public static Translation2d deprojectProjectedNoteFromRobotPose(
      Translation2d noteInRobotSpace, Pose2d robotPose) {
    return new Pose2d(noteInRobotSpace, new Rotation2d()).relativeTo(robotPose).getTranslation();
  }

  public static double closenessRating(Translation2d note) {
    return note.getNorm()
        + Math.abs(note.getAngle().getRadians())
            * NoteVisionConstants.ROTATION_CLOSENESS_WEIGHT_RAD_TO_M;
  }

  public static Optional<Translation2d> getTargetNote(Translation2d[] notes) {
    Optional<Translation2d> closest = Optional.empty();

    for (Translation2d note : notes) {
      if (closest.isEmpty() || closenessRating(note) < closenessRating(closest.get())) {
        closest = Optional.of(note);
      }
    }

    return closest;
  }

  @AutoLogOutput
  public Translation2d[] getNotesInGlobalSpace() {
    Translation2d[] notes = getNotesInRelativeSpace();
    for (int i = 0; i < notes.length; i++) {
      notes[i] =
          projectRelativeNotePoseOntoRobotPose(notes[i], currentRobotVisionFieldPoseSupplier.get());
    }

    return notes;
  }

  public Optional<Translation2d> getNoteByPosition(
      Translation2d targetGlobalTranslation, double threshold) {
    final var globalNotes = getNotesInGlobalSpace();
    Optional<Translation2d> closestNote = Optional.empty();
    for (var note : globalNotes) {
      final double currentDistance = note.getDistance(targetGlobalTranslation);
      if (currentDistance < threshold) {
        if (closestNote.isPresent()) {
          if (closestNote.get().getDistance(targetGlobalTranslation) > currentDistance) {
            closestNote = Optional.of(note);
          }
        } else {
          closestNote = Optional.of(note);
        }
      }
    }

    return closestNote;
  }

  /**
   * @param globalNoteFilter a predicate that is fed a note in global space
   * @return A **robot relative** note that has passed the filter
   */
  public Optional<Translation2d> getClosestNoteFiltered(Predicate<Translation2d> globalNoteFilter) {
    return Arrays.stream(getNotesInGlobalSpace())
        .filter(globalNoteFilter)
        .map(
            globalNote ->
                deprojectProjectedNoteFromRobotPose(
                    globalNote, currentRobotVisionFieldPoseSupplier.get()))
        .reduce((noteA, noteB) -> noteA.getNorm() < noteB.getNorm() ? noteA : noteB);
  }

  public Optional<Translation2d> getCurrentNote() {
    return NoteVisionSubsystem.getTargetNote(getNotesInRelativeSpace());
  }

  /**
   * Set the note to simulate for auto. Please require this subsystem in the command that calls
   * this.
   *
   * @param globalNote the field relative position of target note
   * @param distanceToRemove the subsystem will remove this note once the distance between the note
   *     and a camera is less than this param
   */
  public void setVirtualAutoNote(Translation2d globalNote, double distanceToRemove) {
    virtualAutoNote = Optional.of(globalNote);
    virtualNoteDistanceToRemove = distanceToRemove;
    virtualAutoNoteCoveredFrameCount = 0;
  }

  public void clearAutoNote() {
    virtualAutoNote = Optional.empty();
  }

  private void checkAutoNote(NoteVisionConstants.CameraConfig config) {
    if (virtualAutoNote.isEmpty()) {
      Logger.recordOutput("auto/virtual auto note", new Pose2d());
      return;
    }

    Logger.recordOutput(
        "auto/virtual auto note", new Pose2d(virtualAutoNote.get(), new Rotation2d()));

    final var relativeAutoNote =
        deprojectProjectedNoteFromRobotPose(
            virtualAutoNote.get(), currentRobotVisionFieldPoseSupplier.get());

    if (canCameraSeeNote(config, relativeAutoNote, virtualNoteDistanceToRemove)) {
      virtualAutoNoteCoveredFrameCount++;
    }

    if (virtualAutoNoteCoveredFrameCount
            > NoteVisionConstants.VIRTUAL_AUTO_NOTE_FRAMES_TO_CLEAR.get()
        || Arrays.stream(getNotesInGlobalSpace())
            .anyMatch(
                note ->
                    note.getDistance(virtualAutoNote.get())
                        < AutoConstants.AutoNoteOffsetThresholds.WHILE_ATTEMPTING_PICKUP.get())) {
      virtualAutoNote = Optional.empty();
    }
  }
}
