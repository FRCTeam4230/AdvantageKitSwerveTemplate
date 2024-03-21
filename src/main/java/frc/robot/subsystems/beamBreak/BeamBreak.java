package frc.robot.subsystems.beamBreak;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class BeamBreak extends SubsystemBase {

  private final BeamBreakIO lowerBeamBreakIO;
  private final BeamBreakIO upperBeamBreakIO;
  private final BeamBreakIOInputsAutoLogged lowerBeamBreakInputs =
      new BeamBreakIOInputsAutoLogged();
  private final BeamBreakIOInputsAutoLogged upperBeamBreakInputs =
      new BeamBreakIOInputsAutoLogged();

  public BeamBreak(BeamBreakIO lowerBeamBreakIO, BeamBreakIO upperBeamBreakIO) {
    this.lowerBeamBreakIO = lowerBeamBreakIO;
    this.upperBeamBreakIO = upperBeamBreakIO;
  }

  @Override
  public void periodic() {
    lowerBeamBreakIO.updateInputs(lowerBeamBreakInputs);
    Logger.processInputs("BeamBreak/lower", lowerBeamBreakInputs);

    upperBeamBreakIO.updateInputs(upperBeamBreakInputs);
    Logger.processInputs("BeamBreak/upper", upperBeamBreakInputs);
  }

  public boolean isLowerTriggered() {
    return lowerBeamBreakInputs.triggered;
  }

  public boolean isUpperTriggered() {
    return upperBeamBreakInputs.triggered;
  }

  public boolean detectNote() {
    return isLowerTriggered() || isUpperTriggered();
  }

  public boolean areBothTriggered() {
    return isLowerTriggered() && isUpperTriggered();
  }
}
