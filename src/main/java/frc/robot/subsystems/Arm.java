package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

  private final CANSparkMax m_LeftArmMotor = new CANSparkMax(12, MotorType.kBrushless);
  private final CANSparkMax m_RightArmMotor = new CANSparkMax(13, MotorType.kBrushless);

  private final DutyCycleEncoder m_AbsEncoder = new DutyCycleEncoder(4);

  private final RelativeEncoder m_RelativeEncoder;

  private static final Rotation2d k_reverseHardStop =
      Rotation2d.fromRotations(0.790).times(-1.0);
  private static final Rotation2d k_fwdHardStop =
      Rotation2d.fromRotations(0.222).times(-1.0);

  private final Rotation2d m_absOffset;

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

    m_absOffset = getRawAbsPos().minus(k_reverseHardStop);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("getRawAbsPos", getRawAbsPos().getDegrees());
    SmartDashboard.putNumber("getAbsPos", getAbsPos().getDegrees());
    SmartDashboard.putNumber("getRawRelPos", getRawRelPos().getDegrees());
  }

  private Rotation2d getRawAbsPos() {
    return Rotation2d.fromRotations(-m_AbsEncoder.getAbsolutePosition());
  }

  private Rotation2d getAbsPos() {
    return getRawAbsPos().minus(k_reverseHardStop).times(k_sensorRatio);
  }

  private Rotation2d getRawRelPos() {
    return Rotation2d.fromRotations(m_RelativeEncoder.getPosition());
  }
}
