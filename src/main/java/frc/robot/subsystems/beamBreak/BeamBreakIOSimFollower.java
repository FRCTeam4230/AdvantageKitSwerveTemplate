package frc.robot.subsystems.beamBreak;

import java.util.function.BooleanSupplier;

public class BeamBreakIOSimFollower implements BeamBreakIO {
  private final BooleanSupplier isTriggeredSupplier;

  public BeamBreakIOSimFollower(BooleanSupplier isTriggeredSupplier) {
    this.isTriggeredSupplier = isTriggeredSupplier;
  }

  @Override
  public void updateInputs(BeamBreakIOInputs inputs) {
    inputs.triggered = isTriggeredSupplier.getAsBoolean();
  }
}
