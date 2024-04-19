package frc.robot.util.interpolation;

import java.util.Arrays;

public class InterpolationData {

  protected static double[][] armHoldDataRaw = {
    /* rad position, upper hold volts, lower hold volts */
    {-1, 1.35, 0.39},
    {-0.1, 1.29, 0.47},
    {0.1, 1.37, 0.55},
    {0.2, 1.27, 0.51},
    {0.3, 1.23, 0.49},
    {0.4, 1.11, 0.44},
    {0.5, 1.03, 0.40},
    {0.6, 0.89, 0.35},
    {0.7, 0.78, 0.27},
    {0.8, 0.63, 0.21},
    {0.9, 0.555, 0.16},
    {1.0, 0.41, 0.11},
    {1.1, 0.29, 0.08},
    {1.2, 0.19, -0.18},
    {1.3, 0.1, -0.28},
    {1.4, -0.10, -0.38},
    {1.5, -0.013, -0.4},
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
    {0, .28, 350},
    {1.37, .28, 370},
    {1.572, .40, 370},
    {2.14, .47, 400},
    {3.255, 0.64, 500},
    {4.103, 0.69, 550},
    {5, 0.70, 550},
    {5, 0.3, 550},
    {1000, 0.3, 400}
  };

  public static final double[][] lobbingDistanceData = {
    /* distance m to lobbing target, angle rad, velocity rad/s */
    {0, 0, 350}, {4, 0, 350}, {8, 0.25, 350}, {100, 0.25, 350},
  };
}
