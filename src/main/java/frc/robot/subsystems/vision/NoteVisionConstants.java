package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.TunableNumberWrapper;
import java.lang.invoke.MethodHandles;

public class NoteVisionConstants {
  private static final TunableNumberWrapper tunableTable =
      new TunableNumberWrapper(MethodHandles.lookup().lookupClass());

  public record CameraConfig(
      Transform3d cameraPose, double minDistance, boolean onArm, String name) {
    public NoteVisionIOPhotonVision makePhotonVision() {
      return new NoteVisionIOPhotonVision(name);
    }
  }

  public static final CameraConfig[] CAMERA_CONFIGS = {
    // center
    new CameraConfig(
        new Transform3d(
            new Translation3d(0.4, 0, 0.45), new Rotation3d(0, Units.degreesToRadians(23), 0)),
        0.7,
        true,
        "center"),
    new CameraConfig(
        // left
        new Transform3d(
            new Translation3d(0.15, 0.35, 0.33),
            new Rotation3d(0, Units.degreesToRadians(23), Units.degreesToRadians(60))),
        0.7,
        false,
        "left"),
    new CameraConfig(
        // right
        new Transform3d(
            new Translation3d(0.15, -0.35, 0.33),
            new Rotation3d(0, Units.degreesToRadians(23), Units.degreesToRadians(-60))),
        0.7,
        false,
        "right"),
  };
  public static final double LIFECAM_3000_HFOV = 55;
  public static final double LIFECAM_3000_VFOV = 35;

  public static final double NOTE_GROUPING_TOLERANCE = 0.5;

  public static final double MAX_CAMERA_DISTANCE = 5;

  public static final double IDLE_NOTE_EXPIRATION = 0.5;
  public static final double NOTE_EXPIRATION = 1;
  public static final double IN_CAMERA_EXPIRATION = 0.2;
  public static final double MAX_ARM_POS_RAD = Units.degreesToRadians(3);
  public static final double DISTANCE_TO_RUMBLE = 4;

  public static final double ROTATION_CLOSENESS_WEIGHT_RAD_TO_M = 0.5;

  public static final double INSIDE_FIELD_TOLERANCE = 0.3;
  public static final LoggedTunableNumber VIRTUAL_AUTO_NOTE_FRAMES_TO_CLEAR =
      tunableTable.makeField("virtual auto note frames to clear", 5);
}
