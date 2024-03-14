package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.intake.IntakeConstants;

public class ControllerLogic {
  private final CommandXboxController driverController;
  private final CommandXboxController secondController;
  private final double CLIMBER_DEADBAND = 0.3;
  private final double RESTORE_MANUAL_DRIVE_CONTROL_THRESHOLD = 0.3;

  public ControllerLogic(
      CommandXboxController driverController, CommandXboxController secondController) {
    this.driverController = driverController;
    this.secondController = secondController;
  }

  public double getIntakeSpeed() {
    return MathUtil.clamp(
        driverController.getLeftTriggerAxis()
            - driverController.getRightTriggerAxis()
            + secondController.getLeftTriggerAxis()
            - secondController.getRightTriggerAxis(),
        -1,
        1);
  }

  public Trigger extakeTrigger() {
    return new Trigger(() -> getIntakeSpeed() < -IntakeConstants.INTAKE_SPEED_THRESHOLD.get());
  }

  public Trigger intakeTrigger() {
    return new Trigger(() -> getIntakeSpeed() > IntakeConstants.INTAKE_SPEED_THRESHOLD.get());
  }

  public Trigger armDown() {
    return driverController.povDown().or(secondController.povDown());
  }

  public Trigger armAmpPos() {
    return secondController.povUp();
  }

  public Trigger armSpeakerPos() {
    return secondController.povLeft();
  }

  public Trigger armPodiumPos() {
    return secondController.povRight();
  }

  public double getLeftClimberSpeed() {
    return -secondController.getLeftX();
  }

  public double getRightClimberSpeed() {
    return -secondController.getRightX();
  }

  public Trigger leftClimberActive() {
    return new Trigger(() -> Math.abs(getLeftClimberSpeed()) > CLIMBER_DEADBAND);
  }

  public Trigger rightClimberActive() {
    return new Trigger(() -> Math.abs(getRightClimberSpeed()) > CLIMBER_DEADBAND);
  }

  public Trigger runShooter() {
    return driverController.rightBumper().or(secondController.rightBumper());
  }

  public Trigger forceIntake() {
    return secondController.leftBumper();
  }

  public Trigger toggleVision() {
    return secondController.back();
  }

  public Trigger setPoseInFrontOfSpeaker() {
    return secondController.start();
  }

  public Trigger driveToAmp() {
    return driverController.a();
  }

  public Trigger pointAtSpeaker() {
    return driverController.povLeft();
  }

  public Trigger climbingAlign() {
    return driverController.y();
  }

  // +x means forward for the robot
  public double getDriveSpeedX() {
    return -driverController.getRightY();
  }

  // +y means left
  public double getDriveSpeedY() {
    return -driverController.getRightX();
  }

  // + means counterclockwise
  public double getDriveRotationSpeed() {
    return -driverController.getLeftX();
  }

  public Trigger disableHeadingControl() {
    return new Trigger(
        () -> Math.abs(getDriveRotationSpeed()) > RESTORE_MANUAL_DRIVE_CONTROL_THRESHOLD);
  }
}
