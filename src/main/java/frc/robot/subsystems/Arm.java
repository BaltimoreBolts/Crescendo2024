package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.growingstems.measurements.Angle;

public class Arm extends SubsystemBase {

  private final CANSparkMax m_LeftArmMotor = new CANSparkMax(12, MotorType.kBrushless);
  private final CANSparkMax m_RightArmMotor = new CANSparkMax(13, MotorType.kBrushless);

  private final DutyCycleEncoder m_AbsEncoder = new DutyCycleEncoder(4);

  private final RelativeEncoder m_RelativeEncoder;

  private static final Angle k_reverseHardStop = Angle.degrees(0.790).neg();
  private static final Angle k_fwdHardStop = Angle.degrees(0.222).neg();

  private final Angle m_absOffset;

  private static final double k_sensorRatio = 16.0 / 32.0;

  //   private final RelativeEncoder m_LeftArmEncoder;
  //   private final RelativeEncoder m_RightArmEncoder;

  public Arm() {
    m_LeftArmMotor.restoreFactoryDefaults();
    m_RightArmMotor.restoreFactoryDefaults();

    m_LeftArmMotor.setInverted(true);
    m_RightArmMotor.setInverted(false);

    // m_RightArmEncoder = m_RightArmMotor.getEncoder();
    // m_LeftArmEncoder = m_LeftArmMotor.getEncoder();

    m_RelativeEncoder = m_LeftArmMotor.getAlternateEncoder(8192);
    m_RelativeEncoder.setInverted(true);

    m_AbsEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);

    m_absOffset = getRawAbsPos().sub(k_reverseHardStop);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("getRawAbsPos", getRawAbsPos().asDegrees());
    SmartDashboard.putNumber("getAbsPos", getAbsPos().asDegrees());
    SmartDashboard.putNumber("getRawRelPos", getRawRelPos().asDegrees());
  }

  private Angle getRawAbsPos() {
    return Angle.rotations(-m_AbsEncoder.getAbsolutePosition());
  }

  private Angle getAbsPos() {
    return getRawAbsPos().sub(k_reverseHardStop).mul(k_sensorRatio);
  }

  private Angle getRawRelPos() {
    return Angle.rotations(m_RelativeEncoder.getPosition());
  }
}
