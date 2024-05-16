// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.personDetection;

import org.littletonrobotics.junction.AutoLog;

public interface PersonDetectionIO {
  @AutoLog
  public static class PersonDetectionIOInputs {
    public double personYaw = 0;
    public double timeStampSeconds = 0;
  }

  default void updateInputs(PersonDetectionIOInputs inputs) {}

  default void enable() {}

  default void disable() {}
}
