package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TimeTrigger;
import java.util.function.BooleanSupplier;

public class RumbleSubsystem extends SubsystemBase {
  private record RumblePulse(double durationSeconds, double strength) {}

  private static final RumblePulse TIME = new RumblePulse(1, 1);
  private static final RumblePulse BEAM_BREAM = new RumblePulse(2, 1);
  private static final double NOTE_VISION_STRENGTH = 0.03;

  private final XboxController[] controllers;

  public RumbleSubsystem(XboxController... controllers) {
    this.controllers = controllers;
  }

  public void set(double strength) {
    set(RumbleType.kBothRumble, strength);
  }

  public void set(RumbleType rumbleType, double strength) {
    for (var controller : controllers) {
      controller.setRumble(rumbleType, strength);
    }
  }

  public void stop() {
    set(0);
  }

  public Command rumbleForTime(double seconds, double strength) {
    return startEnd(() -> set(strength), this::stop).withTimeout(seconds);
  }

  public void rumbleAtTimeLeft(double secondsLeft) {
    new TimeTrigger(secondsLeft).onTrue(rumbleForTime(TIME.durationSeconds, TIME.strength));
  }

  public void setRumbleTimes(double... times) {
    for (double time : times) {
      rumbleAtTimeLeft(time);
    }
  }

  public Command noteMonitoring(BooleanSupplier hasNote, BooleanSupplier visionHasNote) {
    return Commands.parallel(
            this.run(
                () -> {
                  set(
                      RumbleType.kLeftRumble,
                      visionHasNote.getAsBoolean() && !hasNote.getAsBoolean()
                          ? NOTE_VISION_STRENGTH
                          : 0);
                }),
            Commands.repeatingSequence(
                Commands.waitUntil(() -> !hasNote.getAsBoolean()),
                Commands.waitUntil(hasNote),
                Commands.runOnce(() -> set(RumbleType.kRightRumble, BEAM_BREAM.strength)),
                Commands.waitSeconds(BEAM_BREAM.durationSeconds),
                Commands.runOnce(() -> set(RumbleType.kRightRumble, 0))))
        .finallyDo(this::stop);
  }
}
