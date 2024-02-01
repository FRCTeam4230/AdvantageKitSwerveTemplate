package frc.robot.subsystems.drive;

import static java.util.Optional.ofNullable;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;

public class DriveController {
  @Setter private Supplier<Rotation2d> headingSupplier;
  @Getter private DriveModeType driveModeType = DriveModeType.STANDARD;

  public boolean isHeadingControlled() {
    return ofNullable(this.headingSupplier).isPresent();
  }

  public boolean isSpeakerControlled() {
    return this.driveModeType == DriveModeType.SPEAKER
        && ofNullable(this.headingSupplier).isPresent();
  }

  public boolean isAmpControlled() {
    return this.driveModeType == DriveModeType.AMP && ofNullable(this.headingSupplier).isPresent();
  }

  public void disableHeadingSupplier() {
    this.headingSupplier = null;
  }

  public Rotation2d getHeadingAngle() {
    return ofNullable(headingSupplier).map(Supplier::get).orElse(null);
  }

  public enum DriveModeType {
    STANDARD,
    AMP,
    SPEAKER,
    SHOOT_WHILE_MOVING,
    SPIN_TO_WIN,
    GAME_PICKUP,
  }

  public void setDriveMode(DriveModeType driveModeType) {
    this.driveModeType = driveModeType;
  }
}
