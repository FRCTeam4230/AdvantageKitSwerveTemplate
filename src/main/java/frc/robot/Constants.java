// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.Alert;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final int loopPeriodMs = 20;
  private static RobotType robotType = RobotType.SIMBOT;
  public static final boolean tuningMode = true;
  public static final boolean characterizationMode = false;

  public static RobotType getRobot() {
    if (RobotBase.isReal() && robotType == RobotType.SIMBOT) {
      new Alert("Invalid Robot Selected, using COMPBOT as default", Alert.AlertType.ERROR)
          .set(true);
      robotType = RobotType.COMPBOT;
    }
    return robotType;
  }

  public static Mode getMode() {
    return switch (getRobot()) {
      case COMPBOT -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
      case SIMBOT -> Mode.SIM;
    };
  }

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public enum RobotType {
    SIMBOT,
    COMPBOT
  }

  /** Checks whether the robot the correct robot is selected when deploying. */
  public static void main(String... args) {
    if (robotType == RobotType.SIMBOT) {
      System.err.println("Cannot deploy, invalid robot selected: " + robotType.toString());
      System.exit(1);
    }
  }

  public static final class ShooterConstants {
    public static final double ROT_TO_RAD = 0;
    public static final double RAD_PER_SEC = ROT_TO_RAD / 60.0;

    public static final class TopWheelsConstants {
      public static final int MOTOR_ID = 0;

      public static final class FeedForwardConstants {
        public static final double kV = 0;
        public static final double kS = 0;
      }

      public static final class PIDControllerConstants {
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
      }
    }

    public static final class BottomWheelsConstants {
      public static final int MOTOR_ID = 0;

      public static final class FeedForwardConstants {
        public static final double kV = 0;
        public static final double kS = 0;
      }

      public static final class PIDControllerConstants {
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
      }
    }
  }
}
