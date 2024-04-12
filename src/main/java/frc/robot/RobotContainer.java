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

package frc.robot;

import static frc.robot.subsystems.drive.DriveConstants.moduleConfigs;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.commands.auto.AutoCommandBuilder;
import frc.robot.commands.auto.AutoConstants;
import frc.robot.commands.climber.ManualClimberCommand;
import frc.robot.commands.climber.ResetClimberBasic;
import frc.robot.subsystems.RumbleSubsystem;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.beamBreak.*;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSparkMax;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.drive.DriveController;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.vision.*;
import frc.robot.util.*;
import java.util.Arrays;
import java.util.List;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardString;
import org.photonvision.simulation.VisionSystemSim;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final AprilTagVision aprilTagVision;
  private final NoteVisionSubsystem noteVision;
  private static final DriveController driveMode = new DriveController();
  private final ShooterSubsystem shooter;

  private final Intake intake;
  private final ArmSubsystem arm;
  private final ClimberSubsystem leftClimber;
  private final ClimberSubsystem rightClimber;
  private final BeamBreak beamBreak;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController secondController = new CommandXboxController(1);
  private final RumbleSubsystem rumbleSubsystem =
      new RumbleSubsystem(driverController.getHID(), secondController.getHID());

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private final Command resetClimbersCommand;
  private final ShooterStateHelpers shooterStateHelpers;
  private final AutoCommandBuilder autoCommandBuilder;

  //   private final LoggedTunableNumber flywheelSpeedInput =
  //       new LoggedTunableNumber("Flywheel Speed", 1500.0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.getMode()) {
      case REAL -> {
        // Real robot, instantiate hardware IO implementations
        drive =
            /*         new ModuleIOTalonFX(moduleConfigs[0]),
            new ModuleIOTalonFX(moduleConfigs[1]),
            new ModuleIOTalonFX(moduleConfigs[2]),
            new ModuleIOTalonFX(moduleConfigs[3]));
             */
            new Drive(
                new GyroIONavX2(),
                new ModuleIOSparkMax(moduleConfigs[0]),
                new ModuleIOSparkMax(moduleConfigs[1]),
                new ModuleIOSparkMax(moduleConfigs[2]),
                new ModuleIOSparkMax(moduleConfigs[3]));
        aprilTagVision =
            new AprilTagVision(
                new AprilTagVisionIOLimelight("limelight"),
                new AprilTagVisionIOLimelight("limelight-front"));
        beamBreak =
            new BeamBreak(
                new BeamBreakIOReal(BeamBreakConstants.LOWER_BEAM_BREAK_SENSOR_PORT),
                new BeamBreakIOReal(BeamBreakConstants.UPPER_BEAM_BREAK_SENSOR_PORT));
        shooter =
            new ShooterSubsystem(
                new ShooterIOSparkMax(ShooterConstants.ShooterWheels.TOP),
                new ShooterIOSparkMax(ShooterConstants.ShooterWheels.BOTTOM));
        intake = new Intake(new IntakeIOSparkMax());
        arm = new ArmSubsystem(new ArmIOSparkMax());
        leftClimber =
            new ClimberSubsystem(
                new ClimberIOSparkMax(
                    ClimberConstants.LEFT_MOTOR_ID, ClimberConstants.LEFT_LIMIT_SWITCH_DIO_PORT),
                "left");
        rightClimber =
            new ClimberSubsystem(
                new ClimberIOSparkMax(
                    ClimberConstants.RIGHT_MOTOR_ID, ClimberConstants.RIGHT_LIMIT_SWITCH_DIO_PORT),
                "right");
        noteVision =
            new NoteVisionSubsystem(
                Arrays.stream(NoteVisionConstants.CAMERA_CONFIGS)
                    .map(NoteVisionConstants.CameraConfig::name)
                    .map(NoteVisionIOPython::new)
                    .toArray(NoteVisionIO[]::new),
                drive.getPoseLogForNoteDetection(),
                drive::getDrive,
                drive::getPose,
                arm::getPositionRad);
      }
      case SIM -> {
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        // flywheel = new Flywheel(new FlywheelIOSim());
        var simApriltagVisionIO =
            new AprilTagVisionIOPhotonVisionSIM(
                "photonCamera1",
                new Transform3d(
                    new Translation3d(-0.5, 0.0, 0.5),
                    new Rotation3d(0, 0, Units.degreesToRadians(180))),
                drive::getDrive);

        VisionSystemSim noteVisionSimSystem = new VisionSystemSim("notes");
        Commands.run(
                () -> {
                  noteVisionSimSystem.update(drive.getDrive());
                })
            .ignoringDisable(true)
            .schedule();

        aprilTagVision = new AprilTagVision(simApriltagVisionIO);
        final NoteVisionIOSim[] noteVisionIOs =
            Arrays.stream(NoteVisionConstants.CAMERA_CONFIGS)
                .map(config -> new NoteVisionIOSim(noteVisionSimSystem, config))
                .toArray(NoteVisionIOSim[]::new);
        shooter = new ShooterSubsystem(new ShooterIOSim(), new ShooterIOSim());
        intake = new Intake(new IntakeIO() {});
        arm = new ArmSubsystem(new ArmIOSim());
        leftClimber = new ClimberSubsystem(new ClimberIO() {}, "left");
        rightClimber = new ClimberSubsystem(new ClimberIO() {}, "right");
        final var beamBreakIOMain =
            new BeamBreakIOSim(
                drive::getDrive,
                noteVisionIOs[0]::getNoteLocations,
                intake::getVoltage,
                shooter::getTargetVelocityRadPerSec,
                noteVisionIOs[0]::removeNote);
        beamBreak =
            new BeamBreak(
                beamBreakIOMain, new BeamBreakIOSimFollower(beamBreakIOMain::isTriggered));
        noteVision =
            new NoteVisionSubsystem(
                noteVisionIOs,
                drive.getPoseLogForNoteDetection(),
                drive::getDrive,
                drive::getPose,
                arm::getPositionRad);

        new Trigger(DriverStation::isAutonomousEnabled)
            .onTrue(Commands.runOnce(noteVisionIOs[0]::resetNotePoses));
      }
      default -> {
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        // flywheel = new Flywheel(new FlywheelIO() {});
        aprilTagVision = new AprilTagVision(new AprilTagVisionIO() {});
        shooter = new ShooterSubsystem(new ShooterIO() {}, new ShooterIO() {});
        intake = new Intake(new IntakeIO() {});
        arm = new ArmSubsystem(new ArmIO() {});
        leftClimber = new ClimberSubsystem(new ClimberIO() {}, "left");
        rightClimber = new ClimberSubsystem(new ClimberIO() {}, "right");
        beamBreak = new BeamBreak(new BeamBreakIO() {}, new BeamBreakIO() {});
        noteVision =
            new NoteVisionSubsystem(
                Arrays.stream(NoteVisionConstants.CAMERA_CONFIGS)
                    .map(config -> new NoteVisionIO() {})
                    .toArray(NoteVisionIO[]::new),
                drive.getPoseLogForNoteDetection(),
                drive::getDrive,
                drive::getPose,
                arm::getPositionRad);
      }
    }

    shooterStateHelpers = new ShooterStateHelpers(shooter, arm, beamBreak);

    resetClimbersCommand =
        ResetClimberBasic.on(leftClimber).alongWith(ResetClimberBasic.on(rightClimber));

    // Configure the button bindings
    aprilTagVision.setDataInterfaces(drive::addVisionData);
    driveMode.setPoseSupplier(drive::getPose);
    driveMode.disableHeadingControl();

    setupLimelightFlashing();

    drive.setPose(new Pose2d(1, 1, new Rotation2d(1, 1)));
    autoCommandBuilder =
        new AutoCommandBuilder(
            drive, noteVision, shooter, intake, arm, beamBreak, shooterStateHelpers);

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    configureAutoChooser();
    configureButtonBindings();
    configureNamedCommands();
    configureRumble();
    configureTeleopCommands();

    intake.setDefaultCommand(IntakeCommands.keepNoteInCenter(intake, beamBreak));

    Dashboard.logField(drive::getPose, noteVision::getNotesInGlobalSpace).schedule();
    Pathfinding.setDynamicObstacles(
        AutoConstants.createDynamicObstaclesList(
            List.of(AutoConstants.AvoidanceZones.STAGE, AutoConstants.AvoidanceZones.CLOSE_NOTES)),
        drive.getPose().getTranslation());
  }

  private void setupLimelightFlashing() {
    new Trigger(beamBreak::detectNote).whileTrue(LimelightControl.lightsOn("limelight"));

    new Trigger(
            () ->
                MathUtil.isNear(
                    0,
                    AllianceFlipUtil.apply(drive.getRotation().plus(Rotation2d.fromDegrees(30)))
                        .getDegrees(),
                    60))
        .whileTrue(LimelightControl.lightsFlashing("limelight-front", 0.2));
  }

  private void configureTeleopCommands() {
    new Trigger(DriverStation::isTeleopEnabled)
        .onTrue(resetClimbersCommand)
        .onTrue(Commands.runOnce(autoCommandBuilder::clearObstacles));
  }

  private void configureRumble() {
    rumbleSubsystem.setRumbleTimes(30, 10);

    rumbleSubsystem.setDefaultCommand(
        rumbleSubsystem.noteMonitoring(
            beamBreak::detectNote,
            () ->
                noteVision.getCurrentNote().stream()
                    .anyMatch(note -> note.getNorm() < NoteVisionConstants.DISTANCE_TO_RUMBLE)));
  }

  private void configureNamedCommands() {
    NamedCommands.registerCommand(
        "Intake until note", IntakeCommands.untilNote(intake, beamBreak::detectNote));

    NamedCommands.registerCommand(
        "ready shooter",
        ArmCommands.autoArmToPosition(arm, ArmConstants.Positions.SPEAKER_POS_RAD::get)
            .andThen(
                Commands.runOnce(
                    () -> shooter.runVelocity(ShooterConstants.SPEAKER_VELOCITY_RAD_PER_SEC.get()),
                    shooter)));

    NamedCommands.registerCommand("pickup note", autoCommandBuilder.pickupVisibleNote());

    NamedCommands.registerCommand("shoot auto", autoCommandBuilder.autoShoot());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link Joystick} or {@link
   * XboxController}), and then passing it to a {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    final ControllerLogic controllerLogic = new ControllerLogic(driverController, secondController);

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
                drive,
                driveMode,
                controllerLogic::getDriveSpeedX,
                controllerLogic::getDriveSpeedY,
                controllerLogic::getDriveRotationSpeed)
            .finallyDo(driveMode::disableHeadingControl));

    controllerLogic
        .ampPathFind()
        .whileTrue(
            DriveToPointBuilder.driveToAndAlign(
                drive, FieldConstants.ampScoringPose, 0.05, Units.degreesToRadians(3), true));
    controllerLogic.pointAtSpeaker().onTrue(Commands.runOnce(driveMode::enableSpeakerHeading));
    controllerLogic.pointAtSource().onTrue(Commands.runOnce(driveMode::enableSourceHeading));
    controllerLogic.climbAlign().onTrue(Commands.runOnce(driveMode::enableStageHeading));
    controllerLogic.lobbingAlign().onTrue(Commands.runOnce(driveMode::enableAmpLobbingHeading));
    controllerLogic
        .disableHeadingControl()
        .onTrue(Commands.runOnce(driveMode::disableHeadingControl));

    controllerLogic
        .visionIntake()
        .and(new Trigger(() -> noteVision.getCurrentNote().isPresent()))
        .whileTrue(autoCommandBuilder.driveIntoVisibleNote());

    controllerLogic
        .extakeTrigger()
        .whileTrue(IntakeCommands.manualIntakeCommand(intake, controllerLogic::getIntakeSpeed));

    controllerLogic
        .intakeTrigger()
        .whileTrue(
            new ConditionalCommand(
                Commands.waitUntil(shooterStateHelpers::canShoot)
                    .andThen(
                        IntakeCommands.manualIntakeCommand(
                            intake, controllerLogic::getIntakeSpeed)),
                IntakeCommands.manualIntakeCommand(intake, controllerLogic::getIntakeSpeed)
                    .until(beamBreak::detectNote),
                beamBreak::detectNote));

    // backup in case arm or shooter can't reach setpoint
    controllerLogic
        .forceIntake()
        .whileTrue(
            Commands.startEnd(
                () -> intake.setVoltage(IntakeConstants.INTAKE_VOLTAGE.get()),
                intake::stop,
                intake));

    controllerLogic
        .resetRobotPoseAngle()
        .onTrue(
            AdjustPositionCommands.setRotation(
                drive, () -> AllianceFlipUtil.apply(new Rotation2d(0))));
    controllerLogic
        .toggleVision()
        .toggleOnTrue(
            Commands.startEnd(
                    () -> aprilTagVision.setEnableVisionUpdates(false),
                    () -> aprilTagVision.setEnableVisionUpdates(true))
                .ignoringDisable(true));

    controllerLogic
        .leftClimberActive()
        .whileTrue(new ManualClimberCommand(leftClimber, controllerLogic::getLeftClimberSpeed));
    controllerLogic
        .rightClimberActive()
        .whileTrue(new ManualClimberCommand(rightClimber, controllerLogic::getRightClimberSpeed));

    //    LoggedDashboardNumber armVolts = new LoggedDashboardNumber("arm volts", 0);
    //    arm.setDefaultCommand(arm.run(() -> arm.setManualVoltage(armVolts.get())));

    controllerLogic
        .armDown()
        .onTrue(ArmCommands.autoArmToPosition(arm, ArmConstants.Positions.INTAKE_POS_RAD::get))
        .onTrue(Commands.runOnce(shooter::stop, shooter));
    controllerLogic
        .armSpeakerPos()
        .onTrue(ArmCommands.autoArmToPosition(arm, ArmConstants.Positions.SPEAKER_POS_RAD::get))
        .onTrue(
            ShooterCommands.runSpeed(shooter, ShooterConstants.SPEAKER_VELOCITY_RAD_PER_SEC::get));
    controllerLogic
        .armSourcePos()
        .onTrue(ArmCommands.autoArmToPosition(arm, ArmConstants.Positions.SOURCE_POS_RAD::get));
    controllerLogic
        .armAmpPos()
        .onTrue(ArmCommands.autoArmToPosition(arm, ArmConstants.Positions.AMP_POS_RAD::get))
        .onTrue(ShooterCommands.runSpeed(shooter, ShooterConstants.AMP_VELOCITY_RAD_PER_SEC::get));
    controllerLogic
        .runShooterForLobbing()
        .onTrue(ArmCommands.autoArmToPosition(arm, ArmConstants.Positions.LOBBING_POS_RAD::get))
        .onTrue(
            ShooterCommands.runSpeed(shooter, ShooterConstants.AMP_LOB_VELOCITY_RAD_PER_SEC::get));
    controllerLogic
        .armPodiumPos()
        .onTrue(
            ArmCommands.autoArmToPosition(
                arm, ArmConstants.Positions.SPEAKER_FROM_PODIUM_POS_RAD::get))
        .onTrue(
            ShooterCommands.runSpeed(shooter, ShooterConstants.PODIUM_VELOCITY_RAD_PER_SEC::get));
    controllerLogic.shooterOff().onTrue(Commands.runOnce(shooter::stop, shooter));

    final Trigger multiDistance = controllerLogic.multiDistanceShot();
    final Trigger inAllianceWing =
        new Trigger(() -> AllianceFlipUtil.apply(drive.getPose()).getX() < FieldConstants.wingX);

    multiDistance
        .and(inAllianceWing)
        .whileTrue(MultiDistanceShot.forSpeaker(drive::getPose, shooter, arm));
    multiDistance
        .and(inAllianceWing.negate())
        .whileTrue(MultiDistanceShot.forLobbing(drive::getPose, shooter, arm));

    controllerLogic
        .runShooter()
        .whileTrue(
            ShooterCommands.runSpeed(
                shooter,
                () -> {
                  if (arm.getSetpointRad() == ArmConstants.Positions.AMP_POS_RAD.get()) {
                    return ShooterConstants.AMP_VELOCITY_RAD_PER_SEC.get();
                  } else if (arm.getSetpointRad()
                      == ArmConstants.Positions.SPEAKER_FROM_PODIUM_POS_RAD.get()) {
                    return ShooterConstants.PODIUM_VELOCITY_RAD_PER_SEC.get();
                  }
                  return ShooterConstants.SPEAKER_VELOCITY_RAD_PER_SEC.get();
                }));
  }

  private void configureAutoChooser() {
    final var configString = new LoggedDashboardString("auto config string", ".102b");
    autoChooser.addDefaultOption(
        "configurable auto", autoCommandBuilder.autoFromConfigString(configString::get));

    Dashboard.showAutoPlan(configString::get).schedule();

    // -999 is an indicator that it is unchanged
    final var angle =
        new LoggedTunableNumber(
            "starting angle deg (0 is intake away from ds, and + is counterclockwise for blue and clockwise for red)",
            -999);
    final var startingPoseId = new LoggedTunableNumber("starting pose id", -999);

    new Trigger(() -> angle.hasChanged(0))
        .onTrue(
            AdjustPositionCommands.setRotation(
                drive, () -> AllianceFlipUtil.apply(Rotation2d.fromDegrees(angle.get()))));
    new Trigger(() -> startingPoseId.hasChanged(0))
        .onTrue(
            Commands.runOnce(
                    () -> {
                      final var pose =
                          AutoConstants.convertPoseIdToPose((int) startingPoseId.get());
                      pose.ifPresent(drive::setAutoStartPose);
                    })
                .ignoringDisable(true));

    autoChooser.addOption("test note pickup", autoCommandBuilder.pickupVisibleNote());

    final SysIdBuilder sysIdBuilder = new SysIdBuilder(autoChooser);

    sysIdBuilder.createSysId(drive, drive::runCharacterizationVolts);
    sysIdBuilder.createSysId(intake, intake::setVoltage);
    sysIdBuilder.createSysId(arm, arm::setManualVoltage);
    sysIdBuilder.createSysId(shooter, shooter::runVolts);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return resetClimbersCommand.asProxy().alongWith(autoChooser.get().asProxy());
  }
}
