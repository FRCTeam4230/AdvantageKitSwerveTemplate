package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ControllerLogic {
  private final CommandXboxController driverController;
  private final CommandXboxController secondController;
  private final double CLIMBER_DEADBAND = 0.3;
  private final double INTAKE_DEADBAND = 0.2;
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
    return new Trigger(() -> getIntakeSpeed() < -INTAKE_DEADBAND);
  }

  public Trigger intakeTrigger() {
    return new Trigger(() -> getIntakeSpeed() > INTAKE_DEADBAND);
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
    return secondController.rightBumper();
  }

  public Trigger forceIntake() {
    return secondController.leftBumper();
  }

  public Trigger toggleVision() {
    return secondController.back();
  }

  public Trigger resetRobotPoseAngle() {
    return secondController.start().or(driverController.start());
  }

  public Trigger pointAtSpeaker() {
    return driverController.povLeft();
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

  public Trigger ampPathFind() {
    return driverController.rightStick();
  }

  public Trigger visionIntake() {
    return driverController.leftStick();
  }

  public Trigger leftSpeakerPathFind() {
    return driverController.leftBumper().and(driverController.rightBumper().negate());
  }

  public Trigger rightSpeakerPathFind() {
    return driverController.rightBumper().and(driverController.leftBumper().negate());
  }

  public Trigger centerSpeakerPathFind() {
    return driverController.leftBumper().and(driverController.rightBumper());
  }

  public Trigger lobbing() {
    return driverController.povUp();
  }

  public Trigger climbAlign() {
    return driverController.povLeft();
  }
}
