package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class MoveClimberToTopCommand extends Command {
  private final ClimberSubsystem leftClimberSubsystem;
  private final ClimberSubsystem rightClimberSubsystem;

  public MoveClimberToTopCommand(
      ClimberSubsystem leftClimberSubsystem, ClimberSubsystem rightClimberSubsystem) {
    this.leftClimberSubsystem = leftClimberSubsystem;
    this.rightClimberSubsystem = rightClimberSubsystem;
    addRequirements(leftClimberSubsystem, rightClimberSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (!leftClimberSubsystem.climberAtTop()) {
      leftClimberSubsystem.setVoltage(ClimberConstants.CLIMBER_RESET_VOLTS.get());
    }
    if (!rightClimberSubsystem.climberAtTop()) {
      rightClimberSubsystem.setVoltage(ClimberConstants.CLIMBER_RESET_VOLTS.get());
    }
  }

  @Override
  public boolean isFinished() {
    return leftClimberSubsystem.climberAtTop() && rightClimberSubsystem.climberAtTop();
  }

  @Override
  public void end(boolean interrupted) {
    leftClimberSubsystem.stop();
    rightClimberSubsystem.stop();
  }
}
