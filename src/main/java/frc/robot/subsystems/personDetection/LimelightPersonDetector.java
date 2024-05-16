package frc.robot.subsystems.personDetection;

import edu.wpi.first.math.util.Units;
import frc.robot.util.LimelightHelpers;
import org.littletonrobotics.junction.Logger;

public class LimelightPersonDetector implements PersonDetectionIO {
  public LimelightPersonDetector() {}

  @Override
  public void updateInputs(PersonDetectionIOInputs inputs) {
    final boolean tv = LimelightHelpers.getTV("limelight");
    final double tx = LimelightHelpers.getTX("limelight");
    final double timestamp = Logger.getRealTimestamp() / 1e6 - 0.1;

    if (tv) {
      inputs.personYaw = Units.degreesToRadians(-tx);
      inputs.timeStampSeconds = timestamp;
    }
  }

  @Override
  public void enable() {
    LimelightHelpers.setPipelineIndex("limelight", 2);
  }

  @Override
  public void disable() {
    LimelightHelpers.setPipelineIndex("limelight", 1);
  }
}
