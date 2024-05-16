package frc.robot.subsystems.personDetection;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class PersonDetectionSubsystem extends SubsystemBase {

  private final PersonDetectionIO personDetectionIO;
  private final PersonDetectionIOInputsAutoLogged personDetectionIOInputs =
      new PersonDetectionIOInputsAutoLogged();

  public PersonDetectionSubsystem(PersonDetectionIO personDetectionIO) {
    this.personDetectionIO = personDetectionIO;
  }

  @Override
  public void periodic() {
    personDetectionIO.updateInputs(personDetectionIOInputs);
    Logger.processInputs("PersonDetection", personDetectionIOInputs);

    final var currentAngle = getPersonAngle();
    currentAngle.ifPresent(rot -> Logger.recordOutput("PersonRotation", rot));
  }

  public Optional<Rotation2d> getPersonAngle() {
    if (Logger.getTimestamp() / 1e6 < personDetectionIOInputs.timeStampSeconds + 0.5) {
      return Optional.of(Rotation2d.fromRadians(personDetectionIOInputs.personYaw));
    }
    return Optional.empty();
  }
}
