// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.flywheel;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.*;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "SparkMax" with
 * "CANSparkFlex".
 */
public class FlywheelIOSparkMax implements FlywheelIO {
  private static final double GEAR_RATIO = 1.5;

  private final SparkMax leader = new SparkMax(0, MotorType.kBrushless);
  private final SparkMax follower = new SparkMax(1, MotorType.kBrushless);
  private final RelativeEncoder encoder = leader.getEncoder();
  private final SparkClosedLoopController pid = leader.getClosedLoopController();


  private void configureLeader(ClosedLoopConfig pidConfig){
    SparkBaseConfig config = new SparkMaxConfig()
        .inverted(false)
        .voltageCompensation(12.0)
        .smartCurrentLimit(30);
    config.apply(pidConfig);

    leader.configure(config,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
  }

  public FlywheelIOSparkMax() {
    leader.setCANTimeout(250);
    follower.configure(new SparkMaxConfig()
        .follow(leader,false),
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    follower.setCANTimeout(250);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition() / GEAR_RATIO);
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / GEAR_RATIO);
    inputs.appliedVolts = leader.getAppliedOutput() * leader.getBusVoltage();
    inputs.currentAmps = new double[] {leader.getOutputCurrent(), follower.getOutputCurrent()};
  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    pid.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * GEAR_RATIO,
        SparkBase.ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ffVolts,
        SparkClosedLoopController.ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    configureLeader(new ClosedLoopConfig().pid(kP,kI,kD));
  }
}
