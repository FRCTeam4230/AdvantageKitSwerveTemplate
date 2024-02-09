package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ArmIOSparkMax implements ArmIO {
  private final CANSparkMax leader =
      new CANSparkMax(ArmConstants.LEFT_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax follower =
      new CANSparkMax(ArmConstants.RIGHT_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
  private final DutyCycleEncoder encoder =
      new DutyCycleEncoder(ArmConstants.DUTY_CYCLE_ENCODER_PORT);

  private final DigitalInput upperLimitSwitch =
      new DigitalInput(ArmConstants.UPPER_LIMIT_SWITCH_PORT);
  private final DigitalInput lowerLimitSwitch =
      new DigitalInput(ArmConstants.LOWER_LIMIT_SWITCH_PORT);

  public ArmIOSparkMax() {
    // The motors are mirrored, so invert
    follower.follow(leader, true);
    encoder.reset();
    encoder.setDistancePerRotation(2 * Math.PI);
    // todo tune encoder
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    // todo measure max and min angles and add them to the limit switches
    inputs.upperLimit = upperLimitSwitch.get();
    inputs.lowerLimit = lowerLimitSwitch.get();
    inputs.positionRad = encoder.getDistance();
    inputs.appliedVolts = leader.getAppliedOutput() * leader.getBusVoltage();
    inputs.currentAmps = new double[] {leader.getOutputCurrent(), follower.getOutputCurrent()};
  }

  @Override
  public void setVoltage(double volts) {
    volts = MathUtil.clamp(volts, -11.9, 11.9);
    leader.setVoltage(volts);
  }
}
