package frc.robot.util.interpolation;

public class InterpolationData {

  protected static double[][] armHoldData = {
    /* rad position, hold volts */
    {-1., 1.},
    {0.067897516, 0.897637814},
    {0.2207049, 0.874015778},
    {0.269767311, 0.66141732},
    {0.2816152, 0.566929132},
    {0.3615125, 0.47},
    {0.41, 0.44},
    {0.487758347, 0.354330719},
    {0.618824339, 0.377952754},
    {0.7323615, 0.25984253},
    {0.800188802, 0.118110236},
    {0.876895658, 0.188976377},
    {0.944489381, 0.118110236},
    {0.971653791, 0.118110236},
    {1.15881465, 0.},
    {1.26878736, 0.},
    {1.3, 0.},
    {Math.PI, -1.},
  };

  protected static final double[][] shooterDistanceData = {
    /* distance m, angle rad, velocity rad/s */
          {0,0.2,280},
          {1.143,0.2,280},
          {2.3622,0.426759297,280},
          {2.9718,0.502168204,300},
          {3.5814,0.466016954,250},
          {4.191,0.625507763,350},
          {4.192,0,0},
          {100,0,0}
  };
}
