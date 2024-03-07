package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.util.function.Supplier;

import org.growingstems.measurements.Angle;
import org.growingstems.measurements.Measurements.Voltage;

public class Arm extends SubsystemBase {

  private final CANSparkMax m_LeftArmMotor = new CANSparkMax(12, MotorType.kBrushless);
  private final CANSparkMax m_RightArmMotor = new CANSparkMax(13, MotorType.kBrushless);

  private final DutyCycleEncoder m_AbsEncoder = new DutyCycleEncoder(4);

  private final RelativeEncoder m_RelativeEncoder;

  private static final Angle k_reverseHardStop = Angle.degrees(-142.0);
  private static final Angle k_fwdHardStop = Angle.degrees(-43.8);

  private static final Angle m_absOffset = Angle.degrees(-2.0);
  private Angle m_relOffset = Angle.ZERO;

  private static final double k_sensorRatio = 16.0 / 32.0;

  //   private final RelativeEncoder m_LeftArmEncoder;
  //   private final RelativeEncoder m_RightArmEncoder;

  public Arm() {
    m_LeftArmMotor.restoreFactoryDefaults();
    m_RightArmMotor.restoreFactoryDefaults();

    m_LeftArmMotor.follow(m_RightArmMotor, true);

    m_LeftArmMotor.setIdleMode(IdleMode.kBrake);
    m_RightArmMotor.setIdleMode(IdleMode.kBrake);

    // m_RightArmEncoder = m_RightArmMotor.getEncoder();
    // m_LeftArmEncoder = m_LeftArmMotor.getEncoder();

    m_RelativeEncoder = m_LeftArmMotor.getAlternateEncoder(8192);
    m_RelativeEncoder.setInverted(true);

    m_AbsEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);

    new WaitCommand(0.5).andThen(this::findRelativeOffset).ignoringDisable(true).schedule();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("getRawAbsPos", getRawAbsPos().asDegrees());
    SmartDashboard.putNumber("getAbsPos", getAbsPos().asDegrees());
    SmartDashboard.putNumber("getRawRelPos", getRawRelPos().asDegrees());
    SmartDashboard.putNumber("getRelPos()", getRelPos().asDegrees());
    SmartDashboard.putNumber("m_relOffset()", m_relOffset.asDegrees());
  }

  private void findRelativeOffset() {
    m_relOffset = getAbsPos().sub(getRawRelPos());
  }

  private Angle getRawAbsPos() {
    return Angle.rotations(-m_AbsEncoder.getAbsolutePosition()).mul(k_sensorRatio);
  }

  private Angle getAbsPos() {
    return getRawAbsPos().sub(k_reverseHardStop).add(m_absOffset);
  }

  private Angle getRawRelPos() {
    return Angle.rotations(m_RelativeEncoder.getPosition()).mul(k_sensorRatio);
  }

  private Angle getRelPos() {
    return getRawRelPos().add(m_relOffset);
  }

  private void setPower(Voltage voltage) {
      m_RightArmMotor.setVoltage(voltage.asVolts());
  }

  public Command setPowerCommand(Supplier<Voltage> voltage) {
    return new RunCommand(
        () -> {
          setPower(voltage.get());
        },
        this);
  }

  public Command emergencyStopCommand() {
    return new RunCommand(
        () -> {
          m_RightArmMotor.stopMotor();
        },
        this);
  }
}
