package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.climber.MoveClimberToBottom;
import frc.robot.commands.climber.MoveClimberToTopCommand;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class AutoClimbCommandGroup extends SequentialCommandGroup {
  public AutoClimbCommandGroup(ClimberSubsystem leftClimber, ClimberSubsystem rightClimber) {
    super(
        new ParallelCommandGroup(
            new MoveClimberToTopCommand(leftClimber, rightClimber),
            new PathFinderAndFollow(PathPlannerPath.fromPathFile("LineUpAmpSideChain"))),
        new ParallelCommandGroup(
            new MoveClimberToBottom(leftClimber), new MoveClimberToBottom(rightClimber)));
  }
}
