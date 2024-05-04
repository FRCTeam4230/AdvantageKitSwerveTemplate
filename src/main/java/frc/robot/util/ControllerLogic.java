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

  public Trigger armSourcePos() {
    return secondController.a().or(driverController.rightBumper());
  }

  public Trigger shooterOn() {
    return secondController.b();
  }

  public Trigger armPodiumPos() {
    return new Trigger(() -> false);
  }

  public double getLeftClimberSpeed() {
    return -secondController.getLeftY();
  }

  public double getRightClimberSpeed() {
    return -secondController.getRightY();
  }

  public Trigger leftClimberActive() {
    return new Trigger(() -> Math.abs(getLeftClimberSpeed()) > CLIMBER_DEADBAND);
  }

  public Trigger rightClimberActive() {
    return new Trigger(() -> Math.abs(getRightClimberSpeed()) > CLIMBER_DEADBAND);
  }

  public Trigger runShooter() {
    return new Trigger(() -> false);
  }

  public Trigger runShooterForLobbing() {
    return secondController.povRight();
  }

  public Trigger forceIntake() {
    return secondController.leftBumper().or(driverController.leftBumper());
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

  public Trigger pointAtSource() {
    return driverController.a();
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
    return driverController.y();
  }

  public Trigger visionIntake() {
    return driverController.x();
  }

  public Trigger lobbingAlign() {
    return driverController.povUp();
  }

  public Trigger climbAlign() {
    return driverController.povRight();
  }

  public Trigger multiDistanceShot() {
    return secondController.rightBumper().or(driverController.b());
  }
}
