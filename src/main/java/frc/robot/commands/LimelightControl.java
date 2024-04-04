package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.LimelightHelpers;
import org.littletonrobotics.junction.Logger;

public class LimelightControl {
  private LimelightControl() {}

  private static void on(String name) {
    LimelightHelpers.setLEDMode_ForceOn(name);
  }

  private static void off(String name) {
    LimelightHelpers.setLEDMode_ForceOff(name);
  }

  public static Command lightsOn(String name) {
    return Commands.startEnd(() -> on(name), () -> off(name)).ignoringDisable(true);
  }

  public static Command lightsFlashing(String name, double time) {
    return lightsFlashing(name, time, time);
  }

  public static Command lightsFlashing(String name, double onTime, double offTime) {
    return Commands.repeatingSequence(
            lightsOn(name).withTimeout(onTime), Commands.waitSeconds(offTime))
        .finallyDo(() -> off(name));
  }
}
