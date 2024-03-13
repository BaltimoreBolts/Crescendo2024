package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;
import org.growingstems.measurements.Angle;

public class HeadingControl {

  private Angle m_pastHeading = Angle.ZERO;
  private double m_pastTimestamp_s = 0.0;
  private Angle m_goalHeading = Angle.ZERO;

  private boolean m_snapEnabled = false;

  public void update(double commandedPower, Angle heading) {

    double headingVelocity = (m_pastHeading.asRadians() - heading.asRadians())
        / (Timer.getFPGATimestamp() - m_pastTimestamp_s);

    m_pastHeading = heading;
    m_pastTimestamp_s = Timer.getFPGATimestamp();

    boolean previousSnapState = m_snapEnabled;
    m_snapEnabled = !(Math.abs(headingVelocity) < 0.03 && !previousSnapState)
        && Math.abs(commandedPower) < 0.05;

    if (m_snapEnabled && !previousSnapState) {
      reset(heading);
    }
  }

  public Angle getSnapHeading() {
    return m_goalHeading;
  }

  public boolean getIfSnapping() {
    return m_snapEnabled;
  }

  public void reset(Angle newSnapHeading) {
    m_goalHeading = newSnapHeading;
  }
}
