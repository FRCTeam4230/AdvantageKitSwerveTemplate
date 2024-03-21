package frc.robot.subsystems.beamBreak;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class BeamBreak extends SubsystemBase {

  private final BeamBreakIO beamBreakIO;
  private final BeamBreakIOInputsAutoLogged beamBreakInputs = new BeamBreakIOInputsAutoLogged();
  private final String descriptor;

  public BeamBreak(BeamBreakIO beamBreakIO, String descriptor) {
    this.beamBreakIO = beamBreakIO;
    this.descriptor = descriptor;
  }

  @Override
  public void periodic() {
    beamBreakIO.updateInputs(beamBreakInputs);
    Logger.processInputs("BeamBreak/" + descriptor, beamBreakInputs);
  }

  public boolean detectNote() {
    return beamBreakInputs.triggered;
  }
}
