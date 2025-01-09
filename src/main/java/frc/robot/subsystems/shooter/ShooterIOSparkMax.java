package frc.robot.subsystems.shooter;

import com.revrobotics.*;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;

public class ShooterIOSparkMax implements ShooterIO {
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController pidController;

  public ShooterIOSparkMax(ShooterConstants.ShooterWheels topOrBottom) {
    SparkBaseConfig config =
        new SparkMaxConfig().voltageCompensation(12.0).smartCurrentLimit(30).inverted(false);

    switch (topOrBottom) {
      case TOP:
        motor = new SparkMax(ShooterConstants.TOP_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        encoder = motor.getEncoder();
        config
            .encoder
            .positionConversionFactor(ShooterConstants.TOP_GEAR_RATIO)
            .velocityConversionFactor(ShooterConstants.TOP_GEAR_RATIO);
        motor.configure(
            config,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters);
        break;
      case BOTTOM:
        motor = new SparkMax(ShooterConstants.BOTTOM_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        encoder = motor.getEncoder();
        config
            .encoder
            .positionConversionFactor(ShooterConstants.BOTTOM_GEAR_RATIO)
            .velocityConversionFactor(ShooterConstants.BOTTOM_GEAR_RATIO);
        motor.configure(
            config,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters);
        break;
      default:
        System.out.println("Shooter top/bottom not valid");
        motor = null;
        encoder = null;
        break;
    }

    motor.setCANTimeout(250);

    // TODO what are these values in the new config??
    // motor.setClosedLoopRampRate(ShooterConstants.CLOSED_LOOP_RAMP_RATE);
    // motor.setOpenLoopRampRate(ShooterConstants.OPEN_LOOP_RAMP_RATE);
    // motor.burnFlash();

    pidController = motor.getClosedLoopController();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition());
    inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = new double[] {motor.getOutputCurrent()};
    inputs.motorTemperatureCelsius = motor.getMotorTemperature();
    inputs.motorSensorFault = motor.getFaults().sensor;
    inputs.motorBrownOut = motor.getFaults().other;
    inputs.motorCANID = motor.getDeviceId();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double feedForwardVolts) {
    pidController.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec),
        SparkBase.ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        feedForwardVolts,
        SparkClosedLoopController.ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    SparkBaseConfig config = new SparkMaxConfig();
    config.closedLoop.pid(kP, kI, kD).velocityFF(0);
    motor.configure(
        config,
        SparkBase.ResetMode.kNoResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
  }
}
