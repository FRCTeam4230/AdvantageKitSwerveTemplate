package frc.robot.subsystems.arm;

import static frc.robot.util.interpolation.InterpolationMaps.angleToHoldVolts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ErrorChecker;
import frc.robot.util.interpolation.InterpolationMaps;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {
  private final ArmIO armIO;
  private final ArmIOInputsAutoLogged armIOInputs = new ArmIOInputsAutoLogged();
  public final PIDController pidController =
      new PIDController(ArmConstants.kP.get(), ArmConstants.kI.get(), ArmConstants.kD.get());

  @Getter private boolean positionControlActive = false;

  private static final Translation3d ROTATION_POINT = new Translation3d(-.26, 0.0, .273);
  private static final String LOGGING_PREFIX = "ArmSubsystem";

  @AutoLogOutput
  private Pose3d getArmPose() {
    Rotation3d rotation = new Rotation3d(0, -armIOInputs.positionRad, 0);
    Translation3d transformedPoint = ROTATION_POINT.minus(ROTATION_POINT.rotateBy(rotation));
    return new Pose3d(transformedPoint, rotation);
  }

  public ArmSubsystem(ArmIO armIO) {
    this.armIO = armIO;
    SmartDashboard.putData(this);
    double shooterAngle = 70;
  }

  private void updateControlConstants() {
    pidController.setP(ArmConstants.kP.get());
    pidController.setI(ArmConstants.kI.get());
    pidController.setD(ArmConstants.kD.get());
    pidController.setTolerance(ArmConstants.setpointToleranceRad.get());
  }

  @Override
  public void periodic() {
    armIO.updateInputs(armIOInputs);
    Logger.processInputs("arm", armIOInputs);
    ErrorChecker.checkError(armIOInputs);

    if (positionControlActive) {
      if (pidController.getSetpoint() < 0.05
          && armIOInputs.positionRad < 0.05
          && Math.abs(armIOInputs.velocityRadPerSec) < 0.05) {
        stop();
        return;
      }

      double pidVolts = pidController.calculate(armIOInputs.positionRad);
      double holdVolts = angleToHoldVolts.get(armIOInputs.positionRad);
      double frictionVolts =
          InterpolationMaps.angleToKS.get(armIOInputs.positionRad) * Math.signum(pidVolts);

      if (pidController.atSetpoint()) {
        frictionVolts = 0;
      }

      double volts = pidVolts + frictionVolts;

      volts =
          MathUtil.clamp(
              volts, -ArmConstants.MAX_ARM_PID_VOLTS.get(), ArmConstants.MAX_ARM_PID_VOLTS.get());

      volts += holdVolts;

      Logger.recordOutput(LOGGING_PREFIX + "Arm/hold volts", holdVolts);
      Logger.recordOutput(LOGGING_PREFIX + "Arm/pid volts", pidVolts);
      Logger.recordOutput(LOGGING_PREFIX + "Arm/friction volts", frictionVolts);

      limitAndSetVolts(volts);
    }

    updateControlConstants();
  }

  public void setPositionRad(double target) {
    positionControlActive = true;
    target = MathUtil.clamp(target, ArmConstants.MIN_RAD, ArmConstants.MAX_RAD);
    pidController.setSetpoint(target);
  }

  @AutoLogOutput
  public double getSetpointRad() {
    return pidController.getSetpoint();
  }

  public void stop() {
    positionControlActive = false;
    armIO.stop();
  }

  private void limitAndSetVolts(double volts) {
    if (volts < 0 && armIOInputs.lowerLimit) {
      volts = 0;
    }
    if (volts > -.1 && armIOInputs.upperLimit) {
      volts = 0;
      //      volts = -0.5;
    }
    volts = MathUtil.clamp(volts, -ArmConstants.MAX_ARM_VOLTS, ArmConstants.MAX_ARM_VOLTS);
    Logger.recordOutput(LOGGING_PREFIX + "attemptedVolts", volts);
    armIO.setVoltage(volts);
  }

  @AutoLogOutput
  public boolean atSetpoint() {
    return pidController.atSetpoint();
  }

  public void setManualVoltage(double volts) {
    positionControlActive = false;
    limitAndSetVolts(volts);
    armIO.setVoltage(volts);
  }

  public boolean atTop() {
    return armIOInputs.upperLimit;
  }

  public boolean atBottom() {
    return armIOInputs.lowerLimit;
  }

  @AutoLogOutput
  public double getPositionRad() {
    return armIOInputs.positionRad;
  }

  @AutoLogOutput
  public double getOffset() {
    return pidController.getPositionError();
  }
}
