package frc.robot.subsystems.drive;

import java.util.*;

public abstract class OdometryThread extends Thread {

  public enum OdometryType {
    PHOENIX,
    SPARKMAX
  }

  protected final List<Queue<Double>> queues = new ArrayList<>();
  protected final List<Queue<Double>> timestampQueues = new ArrayList<>();

  private static final Map<OdometryType, OdometryThread> instanceMap =
      new EnumMap<>(OdometryType.class);

  public static OdometryThread getInstance(OdometryType type) {

    if (!instanceMap.containsKey(type)) {
      switch (type) {
        case PHOENIX:
          instanceMap.put(OdometryType.PHOENIX, new PhoenixOdometryThread());
          break;
        case SPARKMAX:
          instanceMap.put(OdometryType.SPARKMAX, new SparkMaxOdometryThread());
          break;
      }
    }
    return instanceMap.get(type);
  }

  @Override
  public synchronized void start() {
    if (!timestampQueues.isEmpty()) {
      super.start();
    }
  }

  public Queue<Double> makeTimestampQueue() {
    Queue<Double> queue = new ArrayDeque<>(100);
    Drive.odometryLock.lock();
    try {
      timestampQueues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
    }
    return queue;
  }
}
