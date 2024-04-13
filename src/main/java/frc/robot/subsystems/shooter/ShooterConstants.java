package frc.robot.subsystems.shooter;

import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.TunableNumberWrapper;
import java.lang.invoke.MethodHandles;

public final class ShooterConstants {
  private static final TunableNumberWrapper tunableTable =
      new TunableNumberWrapper(MethodHandles.lookup().lookupClass());
  public static final int TOP_MOTOR_ID = 14;
  public static final int BOTTOM_MOTOR_ID = 15;
  public static final LoggedTunableNumber RUN_VOLTS = tunableTable.makeField("run volts", 6);
  public static final LoggedTunableNumber IDLE_VOLTS = tunableTable.makeField("idle volts", 1);
  public static final LoggedTunableNumber AMP_VELOCITY_RAD_PER_SEC =
      tunableTable.makeField("amp rad per sec", 150);
  public static final LoggedTunableNumber AUTO_FIRST_SHOT_RAD_PER_SEC =
      tunableTable.makeField("speaker rad per sec", 250);
  public static final LoggedTunableNumber SPEAKER_VELOCITY_RAD_PER_SEC =
      tunableTable.makeField("speaker rad per sec", 280);
  public static final LoggedTunableNumber PODIUM_VELOCITY_RAD_PER_SEC =
      tunableTable.makeField("podium rad per sec", 300);
  public static final LoggedTunableNumber AMP_LOB_VELOCITY_RAD_PER_SEC =
      tunableTable.makeField("amp lob rad per sec", 350);
  public static final double CLOSED_LOOP_RAMP_RATE = 0.01;
  public static final double OPEN_LOOP_RAMP_RATE = 0.01;
  public static final LoggedTunableNumber VELOCITY_TOLERANCE =
      tunableTable.makeField("velocity tolerance rad per s", 10);

  public static final LoggedTunableNumber AUTO_SHOOTER_TIMEOUT =
      tunableTable.makeField("auto shooter timeout", 1);

  public static final double TOP_GEAR_RATIO = 1;
  public static final double BOTTOM_GEAR_RATIO = 1;

  public enum ShooterWheels {
    TOP,
    BOTTOM
  }

  public record FlywheelConstants(double ks, double kv, double kp) {}

  public record ShooterTune(FlywheelConstants top, FlywheelConstants bottom) {}

  public static final ShooterTune CURRENT_TUNE =
      new ShooterTune(
          new FlywheelConstants(0.21553, 0.021, 1E-4), new FlywheelConstants(0.13955, 0.021, 1E-4));
  public static final ShooterTune OLD_TUNE =
      new ShooterTune(
          new FlywheelConstants(0.0508, 0.0196, 5E-6), new FlywheelConstants(0.1828, 0.0207, 5E-6));

  public static final class FlywheelModelConstants {
    public static final class Top {
      public static final LoggedTunableNumber kP =
          tunableTable.makeField("top/kP", CURRENT_TUNE.top.kp);
    }

    public static final class Bottom {
      public static final LoggedTunableNumber kP =
          tunableTable.makeField("bottom/kP", CURRENT_TUNE.bottom.kp);
    }
  }
}
