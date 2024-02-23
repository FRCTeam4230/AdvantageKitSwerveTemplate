package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
    public double motorTemperatureCelsius = 0.0;
    public boolean motorSensorFault = false;
    public boolean motorBrownOut = false;
    public boolean motorCANRXError = false;
    public boolean motorCANTXError = false;
    public int motorCANID = -1;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Run open loop at the specified voltage/ */
  public default void setVoltage(double volts) {}

  /** Stop in open loop. */
  public default void stop() {}
}
