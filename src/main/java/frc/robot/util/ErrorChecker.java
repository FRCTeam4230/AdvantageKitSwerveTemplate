package frc.robot.util;

import frc.robot.subsystems.arm.ArmIOInputsAutoLogged;
import frc.robot.subsystems.climber.ClimberIOInputsAutoLogged;
import frc.robot.subsystems.drive.ModuleIOInputsAutoLogged;
import frc.robot.subsystems.intake.IntakeIOInputsAutoLogged;
import frc.robot.subsystems.shooter.ShooterIOInputsAutoLogged;

public class ErrorChecker {
  private ErrorChecker() {};

  public static void checkError(ShooterIOInputsAutoLogged inputs) {
    //Print error messages
    if(inputs.motorBrownOut) {
      System.err.println("MOTOR CAN ID " + inputs.motorCANID + " BROWNOUT");
    }
    if(inputs.motorSensorFault) {
      System.err.println("MOTOR CAN ID " + inputs.motorCANID + " SENSOR FAULT");
    }
    if(inputs.motorCANRXError) {
      System.err.println("MOTOR CAN ID " + inputs.motorCANID + " CAN RX ERROR");
    }
    if(inputs.motorCANTXError) {
      System.err.println("MOTOR CAN ID " + inputs.motorCANID + " CAN TX ERROR");
    }
  }

  public static void checkError(ModuleIOInputsAutoLogged inputs) {
    //Print error messages
    if(inputs.driveMotorBrownOut) {
      System.err.println("MOTOR CAN ID " + inputs.driveMotorCANID + " BROWNOUT");
    }
    if(inputs.driveMotorSensorFault) {
      System.err.println("MOTOR CAN ID " + inputs.driveMotorCANID + " SENSOR FAULT");
    }
    if(inputs.driveMotorCANRXError) {
      System.err.println("MOTOR CAN ID " + inputs.driveMotorCANID + " CAN RX ERROR");
    }
    if(inputs.driveMotorCANTXError) {
      System.err.println("MOTOR CAN ID " + inputs.driveMotorCANID + " CAN TX ERROR");
    }
    if(inputs.turnMotorBrownOut) {
      System.err.println("MOTOR CAN ID " + inputs.turnMotorCANID + " BROWNOUT");
    }
    if(inputs.turnMotorSensorFault) {
      System.err.println("MOTOR CAN ID " + inputs.turnMotorCANID + " SENSOR FAULT");
    }
    if(inputs.turnMotorCANRXError) {
      System.err.println("MOTOR CAN ID " + inputs.turnMotorCANID + " CAN RX ERROR");
    }
    if(inputs.turnMotorCANTXError) {
      System.err.println("MOTOR CAN ID " + inputs.turnMotorCANID + " CAN TX ERROR");
    }
  }

  public static void checkError(ArmIOInputsAutoLogged inputs) {
    //Print error messages
    if(inputs.leftMotorBrownOut) {
      System.err.println("MOTOR CAN ID " + inputs.leftMotorCANID + " BROWNOUT");
    }
    if(inputs.leftMotorSensorFault) {
      System.err.println("MOTOR CAN ID " + inputs.leftMotorCANID + " SENSOR FAULT");
    }
    if(inputs.leftMotorCANRXError) {
      System.err.println("MOTOR CAN ID " + inputs.leftMotorCANID + " CAN RX ERROR");
    }
    if(inputs.leftMotorCANTXError) {
      System.err.println("MOTOR CAN ID " + inputs.leftMotorCANID + " CAN TX ERROR");
    }
    if(inputs.rightMotorBrownOut) {
      System.err.println("MOTOR CAN ID " + inputs.rightMotorCANID + " BROWNOUT");
    }
    if(inputs.rightMotorSensorFault) {
      System.err.println("MOTOR CAN ID " + inputs.rightMotorCANID + " SENSOR FAULT");
    }
    if(inputs.rightMotorCANRXError) {
      System.err.println("MOTOR CAN ID " + inputs.rightMotorCANID + " CAN RX ERROR");
    }
    if(inputs.rightMotorCANTXError) {
      System.err.println("MOTOR CAN ID " + inputs.rightMotorCANID + " CAN TX ERROR");
    }
  }

  public static void checkError(IntakeIOInputsAutoLogged inputs) {
    //Print error messages
    if(inputs.motorBrownOut) {
      System.err.println("MOTOR CAN ID " + inputs.motorCANID + " BROWNOUT");
    }
    if(inputs.motorSensorFault) {
      System.err.println("MOTOR CAN ID " + inputs.motorCANID + " SENSOR FAULT");
    }
    if(inputs.motorCANRXError) {
      System.err.println("MOTOR CAN ID " + inputs.motorCANID + " CAN RX ERROR");
    }
    if(inputs.motorCANTXError) {
      System.err.println("MOTOR CAN ID " + inputs.motorCANID + " CAN TX ERROR");
    }
  }

  public static void checkError(ClimberIOInputsAutoLogged inputs) {
    //Print error messages
    if(inputs.motorBrownOut) {
      System.err.println("MOTOR CAN ID " + inputs.motorCANID + " BROWNOUT");
    }
    if(inputs.motorSensorFault) {
      System.err.println("MOTOR CAN ID " + inputs.motorCANID + " SENSOR FAULT");
    }
    if(inputs.motorCANRXError) {
      System.err.println("MOTOR CAN ID " + inputs.motorCANID + " CAN RX ERROR");
    }
    if(inputs.motorCANTXError) {
      System.err.println("MOTOR CAN ID " + inputs.motorCANID + " CAN TX ERROR");
    }
  }
}
