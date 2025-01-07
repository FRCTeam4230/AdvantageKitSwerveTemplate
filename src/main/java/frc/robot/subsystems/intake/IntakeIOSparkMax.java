package frc.robot.subsystems.intake;

import com.revrobotics.*;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;

public class IntakeIOSparkMax implements IntakeIO {
  private static final double GEAR_RATIO = IntakeConstants.GEAR_RATIO;

  private final SparkMax motor =
      new SparkMax(IntakeConstants.MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();
  private final SparkClosedLoopController pid = motor.getClosedLoopController();

  public IntakeIOSparkMax() {

    motor.configure(
        new SparkMaxConfig()
            .idleMode(SparkBaseConfig.IdleMode.kCoast)
            .voltageCompensation(12.0)
            .smartCurrentLimit(30)
            .closedLoopRampRate(IntakeConstants.CLOSED_LOOP_RAMP_RATE)
            .openLoopRampRate(IntakeConstants.OPEN_LOOP_RAMP_RATE),
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    motor.setCANTimeout(250);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition() / GEAR_RATIO);
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / GEAR_RATIO);
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = new double[] {motor.getOutputCurrent()};
    inputs.motorTemperatureCelsius = motor.getMotorTemperature();
    inputs.motorSensorFault = motor.getFaults().sensor; //might be wrong
    inputs.motorBrownOut = motor.getFaults().other; //might be wrong
    inputs.motorCANID = motor.getDeviceId();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }
}
