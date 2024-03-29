package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NoteVisionIOPython implements NoteVisionIO {
  private final NetworkTableEntry cameraDataEntry;
  private final String name;

  public NoteVisionIOPython(String cameraName) {
    cameraDataEntry =
        NetworkTableInstance.getDefault().getTable("note-vision").getEntry(cameraName);
    name = cameraName;
  }

  /*
   Python code for data
   # format for n detections, [n pitches..., n yaws..., latency]
   output_entry.setDoubleArray(
       [a[1] for a in angles] +
       [a[0] for a in angles] +
       [time.time() - timestamp_seconds]
   )
  */

  @Override
  public void updateInputs(NoteVisionIOInputs inputs) {
    inputs.name = name;

    final var result = cameraDataEntry.getValue();

    if (!result.isValid()) {
      // camera likely not connected yet
      return;
    }

    double[] data = result.getDoubleArray();

    if (data.length % 2 == 0) {
      // we shouldn't have an even number of entries
      System.out.println("MALFORMED DATA FROM PYTHON CAMERA: " + cameraDataEntry.getName());
    }

    final int detectionCount = data.length / 2;

    inputs.notePitches = new double[detectionCount];
    inputs.noteYaws = new double[detectionCount];

    // format for n detections, [n pitches..., n yaws..., latency]
    final double latency = data[data.length - 1];
    inputs.timeStampSeconds = result.getTime() / 1e6 - latency;

    for (int i = 0; i < detectionCount; i++) {
      inputs.notePitches[i] = data[i];
      inputs.noteYaws[i] = data[detectionCount + i];
    }
  }
}
