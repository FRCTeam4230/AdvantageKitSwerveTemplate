package frc.robot.util.interpolation;

import java.util.Arrays;

public class InterpolationData {

  protected static double[][] armHoldDataRaw = {
    /* rad position, upper hold volts, lower hold volts */
    {-1, 2, 0.39},
    {-0.1, 1.77, 0.47},
    {0.1, 1.77, 0.55},
    {0.2, 1.65, 0.51},
    {0.3, 1.6, 0.49},
    {0.4, 1.47, 0.44},
    {0.5, 1.4, 0.40},
    {0.6, 1.25, 0.35},
    {0.7, 1.1, 0.27},
    {0.8, 0.9, 0.21},
    {0.9, 0.75, 0.16},
    {1.0, 0.6, 0.11},
    {1.1, 0.55, 0.08},
    {1.2, 0.55, -0.18},
    {1.3, 0.4, -0.28},
    {1.4, 0.2, -0.38},
    {1.5, 0.2, -0.4},
    {3, -2, -2},
  };

  protected static double[][] armHoldData =
      Arrays.stream(armHoldDataRaw)
          .map(
              row -> {
                double upper = row[1];
                double lower = row[2];

                double middle = (upper + lower) / 2;
                double kS = (upper - lower) / 2;

                return new double[] {row[0], middle, kS};
              })
          .toArray(double[][]::new);

  /** distance to speaker m, angle rad, velocity rad/s */
  protected static final double[][] shooterDistanceData = {
    {0, .28, 500},
    {1.37, .28, 500},
    {1.572, .40, 500},
    {2.14, .48, 500},
    {3.255, 0.66, 500},
    {4.103, 0.70, 550},
    {5, 0.70, 550},
    {5, 0.3, 550},
    {1000, 0.3, 400}
  };

  public static final double[][] lobbingDistanceData = {
    /* distance m to lobbing target, angle rad, velocity rad/s */
    {0, 0, 350}, {4, 0, 350}, {8, 0.25, 350}, {100, 0.25, 350},
  };
}
