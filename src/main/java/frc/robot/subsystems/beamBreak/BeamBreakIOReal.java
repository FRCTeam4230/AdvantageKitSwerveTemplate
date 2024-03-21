package frc.robot.subsystems.beamBreak;

import edu.wpi.first.wpilibj.DigitalInput;

public class BeamBreakIOReal implements BeamBreakIO {
  private final DigitalInput beamBreakSensor;

  public BeamBreakIOReal(int port) {
    beamBreakSensor = new DigitalInput(port);
  }

  @Override
  public void updateInputs(BeamBreakIOInputs inputs) {
    inputs.triggered = !beamBreakSensor.get();
  }
}
