package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import frc.robot.commands.auto.AutoConstants;
import frc.robot.util.AllianceFlipUtil;
import java.util.ArrayList;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

public class NoteVisionIOSim implements NoteVisionIO {
  private final VisionSystemSim visionSim;
  private final TargetModel targetModel = new TargetModel(0.2, 0.2, 0.05);
  private final PhotonCamera camera;
  private Pose3d[] notePoses = new Pose3d[8];
  private final NoteVisionConstants.CameraConfig config;

  public NoteVisionIOSim(VisionSystemSim visionSim, NoteVisionConstants.CameraConfig config) {
    this.visionSim = visionSim;
    this.config = config;
    resetNotePoses();

    camera = new PhotonCamera(config.name());

    final var cameraProps = new SimCameraProperties();
    cameraProps.setAvgLatencyMs(10);
    cameraProps.setFPS(30);
    cameraProps.setCalibration(1200, 960, Rotation2d.fromDegrees(75));
    visionSim.addCamera(new PhotonCameraSim(camera, cameraProps), config.cameraPose());
  }

  public void resetNotePoses() {
    notePoses =
        Arrays.stream(AutoConstants.AUTO_NOTES)
            .map(AllianceFlipUtil::apply)
            .map(
                translation ->
                    new Pose3d(translation.getX(), translation.getY(), 0, new Rotation3d()))
            .toArray(Pose3d[]::new);
    updateNoteTargets();
  }

  private void updateNoteTargets() {
    visionSim.clearVisionTargets();

    visionSim.addVisionTargets(
        Arrays.stream(notePoses)
            .map(notePose -> new VisionTargetSim(notePose, targetModel))
            .toArray(VisionTargetSim[]::new));

    Logger.recordOutput("note vision sim locations", notePoses);
  }

  @Override
  public void updateInputs(NoteVisionIOInputs inputs) {
    inputs.name = camera.getName();

    var result = camera.getLatestResult();

    var targets = result.getTargets();

    inputs.notePitches = new double[targets.size()];
    inputs.noteYaws = new double[targets.size()];
    inputs.timeStampSeconds = result.getTimestampSeconds();

    for (int i = 0; i < targets.size(); i++) {
      inputs.noteYaws[i] =
          -Units.degreesToRadians(targets.get(i).getYaw() + 0.5 * (2 * Math.random() - 1));
      inputs.notePitches[i] =
          -Units.degreesToRadians(targets.get(i).getPitch() + 0.5 * (2 * Math.random() - 1));
    }
  }

  public Translation2d[] getNoteLocations() {
    return Arrays.stream(notePoses)
        .map(Pose3d::toPose2d)
        .map(Pose2d::getTranslation)
        .toArray(Translation2d[]::new);
  }

  public void removeNote(int index) {
    var noteArrayList = new ArrayList<>(Arrays.asList(notePoses));
    noteArrayList.remove(index);
    notePoses = noteArrayList.toArray(new Pose3d[0]);
    updateNoteTargets();
  }
}
