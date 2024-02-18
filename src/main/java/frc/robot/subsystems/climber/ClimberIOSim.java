package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

public class ClimberIOSim implements ClimberIO {
    private final CANSparkMax motor = new DCMotor();


    public ClimberIOSim() {
      motor.setInverted(false);
      limitSwitch = new DigitalInput(limitSwitchDIOPort);
      encoder = motor.getEncoder();
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
      inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
      inputs.currentAmps = motor.getOutputCurrent();
      inputs.atBottom = !limitSwitch.get();
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
