package frc.robot.subsystems.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;

public class ClimberIOSparkMax implements ClimberIO {
  private final SparkMax motor;

  private final DigitalInput limitSwitch;
  private final RelativeEncoder encoder;

  public ClimberIOSparkMax(int sparkMaxCanID, int limitSwitchDIOPort) {
    motor = new SparkMax(sparkMaxCanID, SparkLowLevel.MotorType.kBrushless);
    motor.configure(new SparkMaxConfig()
        .inverted(false)
        .idleMode(SparkBaseConfig.IdleMode.kBrake),
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters);

    limitSwitch = new DigitalInput(limitSwitchDIOPort);
    encoder = motor.getEncoder();
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.atBottom = !limitSwitch.get();
    inputs.positionRotations = encoder.getPosition();
    inputs.motorTemperatureCelsius = motor.getMotorTemperature();
    inputs.motorSensorFault = motor.getFaults().sensor;
    inputs.motorBrownOut = motor.getFaults().other;
    inputs.motorCANID = motor.getDeviceId();
  }

  @Override
  public void resetEncoder() {
    encoder.setPosition(0);
  }

  @Override
  public void toggleMotorInversion() {
    motor.setInverted(!motor.getInverted());
  }

  @Override
  public void setVoltage(double volts) {
    volts = MathUtil.clamp(volts, -11, 11);
    motor.setVoltage(volts);
  }
}
