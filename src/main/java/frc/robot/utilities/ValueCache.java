package frc.robot.utilities;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.RobotController;

public class ValueCache<T> {
  private Supplier<T> m_supplier;
  private final long m_ttlMicroseconds;
  private long m_timestampMicroseconds;
  private T m_value;

  public ValueCache(Supplier<T> supplier, long ttlMicroseconds) {
    m_supplier = supplier;
    m_ttlMicroseconds = ttlMicroseconds;
    update(getTimestampMicroseconds());
  }

  private long getTimestampMicroseconds() {
    return RobotController.getFPGATime();
  }

  private void update(long currentTimestampMicroseconds) {
    m_timestampMicroseconds = currentTimestampMicroseconds;
    m_value = m_supplier.get();
  }

  public T get() {
    long currentTimestampMicroseconds = getTimestampMicroseconds();
    if (m_timestampMicroseconds + m_ttlMicroseconds < currentTimestampMicroseconds) {
      update(currentTimestampMicroseconds);
    }
    return m_value;
  }

  public void flush() {
    // Set the timestamp far enough in the past to invalidate the cache.
    long currentTimestampMicroseconds = getTimestampMicroseconds();
    m_timestampMicroseconds = currentTimestampMicroseconds - m_ttlMicroseconds - 1;
  }
}