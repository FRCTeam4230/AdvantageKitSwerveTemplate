package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.vision.NoteVisionSubsystem;
import java.util.function.BooleanSupplier;

public class PickUpNoteCommand extends Command {
  private final Drive drive;
  private final Intake intake;
  private final NoteVisionSubsystem noteVisionSubsystem;
  private final BooleanSupplier hasNote;

  public PickUpNoteCommand(
      Drive drive,
      Intake intake,
      NoteVisionSubsystem noteVisionSubsystem,
      BooleanSupplier hasNote) {
    this.drive = drive;
    this.intake = intake;
    this.noteVisionSubsystem = noteVisionSubsystem;
    this.hasNote = hasNote;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.drive, this.intake, this.noteVisionSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    var currentNote = noteVisionSubsystem.getCurrentNote();

    if (currentNote.isEmpty()) {
      drive.stop();
      intake.stop();
      return;
    }

    var angle = currentNote.get().getAngle();
    double distance = currentNote.get().getNorm();

    var omega = drive.getThetaController().calculate(0, angle.getRadians());
    if (drive.getThetaController().atGoal()) {
      omega = 0;
    }
    omega = Math.copySign(Math.min(1, Math.abs(omega)), omega);

    var speed = 1;

    var speedx = speed * angle.getCos();
    var speedy = speed * angle.getSin();

    if (distance < 1 && !drive.getThetaController().atGoal()) {
      speedx = 0;
      speedy = 0;
    }

    var speeds = ChassisSpeeds.fromRobotRelativeSpeeds(speedx, speedy, omega, new Rotation2d());

    drive.runVelocity(speeds);

    if (currentNote.get().getNorm() < 2) {
      intake.setVoltage(IntakeConstants.INTAKE_VOLTAGE.get());
    } else {
      intake.stop();
    }
  }

  @Override
  public boolean isFinished() {
    return hasNote.getAsBoolean();
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
    drive.stop();
  }
}
