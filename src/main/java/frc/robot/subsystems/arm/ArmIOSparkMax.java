package frc.robot.subsystems.arm;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class ArmIOSparkMax implements ArmIO {
  private final SparkMax leader =
      new SparkMax(ArmConstants.LEFT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
  private final SparkMax follower =
      new SparkMax(ArmConstants.RIGHT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
  private final DutyCycleEncoder encoder =
      new DutyCycleEncoder(ArmConstants.DUTY_CYCLE_ENCODER_PORT);

  private final RelativeEncoder velocityEncoder = leader.getEncoder();
  private final DigitalInput upperLimitSwitch =
      new DigitalInput(ArmConstants.UPPER_LIMIT_SWITCH_PORT);

  public ArmIOSparkMax() {

    EncoderConfig leaderEncoderConfig= new EncoderConfig().velocityConversionFactor(Math.PI * 2 / 60 / ArmConstants.MOTOR_TO_ARM_RATIO);
    EncoderConfig relativeEncoderConfig = new EncoderConfig().positionConversionFactor(2 * Math.PI);
    leader.configure(new SparkMaxConfig()
        .inverted(true)
        .idleMode(SparkBaseConfig.IdleMode.kBrake)
            .apply(leaderEncoderConfig),
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    // The motors are mirrored, so invert



    follower.configure(new SparkMaxConfig().follow(leader,true).idleMode(SparkBaseConfig.IdleMode.kBrake),
    SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.positionRad =
        (encoder.get() * Math.PI * 2) - ArmConstants.ARM_ENCODER_OFFSET_RAD;
    inputs.velocityRadPerSec = velocityEncoder.getVelocity();
    inputs.upperLimit = (inputs.positionRad > ArmConstants.MAX_RAD) || (!upperLimitSwitch.get());
    inputs.lowerLimit = (inputs.positionRad < ArmConstants.MIN_RAD);
    inputs.appliedVolts = leader.getAppliedOutput() * leader.getBusVoltage();
    inputs.currentAmps = new double[] {leader.getOutputCurrent(), follower.getOutputCurrent()};
    inputs.leftMotorTemperatureCelsius = leader.getMotorTemperature();
    inputs.rightMotorTemperatureCelsius = follower.getMotorTemperature();
    inputs.leftMotorSensorFault = leader.getFaults().sensor;
    inputs.leftMotorBrownOut = leader.getFaults().other;
    inputs.leftMotorCANID = leader.getDeviceId();
    inputs.rightMotorSensorFault = follower.getFaults().sensor;
    inputs.rightMotorBrownOut = follower.getFaults().other;
    inputs.rightMotorCANID = follower.getDeviceId();
  }

  @Override
  public void setVoltage(double volts) {
    volts = MathUtil.clamp(volts, -11.9, 11.9);
    leader.setVoltage(volts);
  }
}
