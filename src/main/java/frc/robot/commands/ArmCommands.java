package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import java.util.function.DoubleSupplier;

public class ArmCommands {

  private ArmCommands() {}
  ;

  public static Command manualArmCommand(ArmSubsystem arm, DoubleSupplier supplier) {
    return Commands.run(
        () -> {
          double volts = supplier.getAsDouble() * ArmConstants.MANUAL_ARM_MAX_VOLTS;
          if ((volts > 0) && arm.atTop()) {
            return;
          }

          if ((volts < 0) && arm.atBottom()) {
            return;
          }
          arm.setManualVoltage(volts);
        },
        arm);
  }

  public static Command manualArmPos(ArmSubsystem arm, DoubleSupplier radianSupplier) {
    return Commands.runEnd(
        () -> {
          arm.setPositionRad(radianSupplier.getAsDouble());
        },
        arm::stop,
        arm);
  }

  public static Command autoArmToPosition(ArmSubsystem arm, DoubleSupplier targetRadianSupplier) {
    return Commands.runOnce(
        () -> {
          arm.setPositionRad(targetRadianSupplier.getAsDouble());
        });
  }
}
