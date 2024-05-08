package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Optional;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;

/**
 * The DriveController class represents a controller for the robot's drive system. It provides
 * methods to control the heading and drive mode of the robot.
 */
public class DriveController {
  @Getter private Optional<Supplier<Rotation2d>> headingSupplier = Optional.empty();
  @Setter private Supplier<Pose2d> poseSupplier = Pose2d::new;

  /**
   * Sets the heading supplier that provides the desired heading for the robot.
   *
   * @param headingSupplier The supplier that provides the desired heading.
   */
  public void setHeadingSupplier(Supplier<Rotation2d> headingSupplier) {
    this.headingSupplier = Optional.of(headingSupplier);
  }

  /** Disables heading control (heading control is disabled). */
  public void disableHeadingControl() {
    this.headingSupplier = Optional.empty();
  }
}
