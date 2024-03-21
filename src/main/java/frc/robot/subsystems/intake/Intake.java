package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.beamBreak.BeamBreak;
import frc.robot.util.ErrorChecker;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  @Getter private double voltage = 0;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    ErrorChecker.checkError(inputs);
  }

  public void stop() {
    voltage = 0;
    io.stop();
  }

  @AutoLogOutput
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  public void setVoltage(double volts) {
    voltage = volts;
    io.setVoltage(volts);
  }

  public Command centerNote(BeamBreak beamBreak) {
    return Commands.waitSeconds(IntakeConstants.NOTE_CENTERING_DELAY_SECONDS)
        .andThen(
            runEnd(
                () -> {
                  if (beamBreak.areBothTriggered() || !beamBreak.detectNote()) {
                    stop();
                    return;
                  }

                  if (beamBreak.isLowerTriggered()) {
                    setVoltage(IntakeConstants.NOTE_CENTERING_VOLTS.get());
                    return;
                  }

                  if (beamBreak.isUpperTriggered()) {
                    setVoltage(-IntakeConstants.NOTE_CENTERING_VOLTS.get());
                    return;
                  }

                  System.out.println(
                      "THIS SHOULD NOT HAVE BEEN REACHED, CHECKOUT centerNote IN Intake.java");
                },
                this::stop));
  }
}
