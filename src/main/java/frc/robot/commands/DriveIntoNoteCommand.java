package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveIntoNoteCommand extends Command {
  private final Drive drive;
  private final Supplier<Optional<Translation2d>> relativeNoteSupplier;
  private final BooleanSupplier hasNote;
  private final BooleanSupplier armDown;
  private final double scanRadPerSec;

  public DriveIntoNoteCommand(
      Drive drive,
      Supplier<Optional<Translation2d>> relativeNoteSupplier,
      BooleanSupplier hasNote,
      BooleanSupplier armDown) {
    this(drive, relativeNoteSupplier, hasNote, armDown, 0);
  }

  public DriveIntoNoteCommand(
      Drive drive,
      Supplier<Optional<Translation2d>> relativeNoteSupplier,
      BooleanSupplier hasNote,
      BooleanSupplier armDown,
      double scanRadPerSec) {
    this.drive = drive;
    this.relativeNoteSupplier = relativeNoteSupplier;
    this.hasNote = hasNote;
    this.armDown = armDown;
    this.scanRadPerSec = scanRadPerSec;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.drive);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    var currentNote = relativeNoteSupplier.get();

    if (currentNote.isEmpty()) {
      drive.runVelocity(
          ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, scanRadPerSec, new Rotation2d()));
      return;
    }

    var angle = currentNote.get().getAngle();
    double distanceToNote = currentNote.get().getNorm();

    var omega =
        drive.getThetaController().calculate(0, angle.getRadians())
            * DriveConstants.HeadingControllerConstants.NOTE_PICKUP_MULT.get();
    if (drive.getThetaController().atSetpoint()) {
      omega = 0;
    }

    double speed =
        MathUtil.clamp(
            distanceToNote * DriveConstants.NOTE_PICKUP_DISTANCE_TO_SPEED_MULT.get(),
            DriveConstants.NOTE_PICKUP_MIN_SPEED.get(),
            DriveConstants.NOTE_PICKUP_MAX_SPEED.get());

    final boolean nearNote = distanceToNote < 1.2;
    final boolean pointedAtNote =
        Math.abs(drive.getThetaController().getPositionError())
            < Units.degreesToRadians(
                DriveConstants.HeadingControllerConstants.NOTE_PICKUP_TOLERANCE.get());
    if (nearNote && (!armDown.getAsBoolean() || !pointedAtNote)) {
      speed = 0;
    }

    double speedx = speed * angle.getCos();
    double speedy = speed * angle.getSin();

    var speeds = ChassisSpeeds.fromRobotRelativeSpeeds(speedx, speedy, omega, new Rotation2d());
    Logger.recordOutput("note pickup/omega", omega);
    Logger.recordOutput("note pickup/pid offset", drive.getThetaController().getPositionError());
    Logger.recordOutput("note pickup/speed", speed);

    drive.runVelocity(speeds);
  }

  @Override
  public boolean isFinished() {
    return hasNote.getAsBoolean();
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
