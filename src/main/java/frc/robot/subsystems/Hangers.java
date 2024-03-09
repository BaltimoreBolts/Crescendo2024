package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

public class Hangers extends SubsystemBase {

  private final CANSparkMax m_hangerRightmotor = new CANSparkMax(11, MotorType.kBrushless);
  private final CANSparkMax m_hangerLeftmotor = new CANSparkMax(10, MotorType.kBrushless);

  private final RelativeEncoder m_rightHangerEncoder;
  private final RelativeEncoder m_leftHangerEncoder;

  private final DigitalInput m_Lefthall = new DigitalInput(2);
  private final DigitalInput m_Righthall = new DigitalInput(3);

  public Hangers() {
    m_hangerRightmotor.restoreFactoryDefaults();
    m_hangerLeftmotor.restoreFactoryDefaults();

    m_hangerRightmotor.setInverted(true);
    m_hangerLeftmotor.setInverted(false);

    m_rightHangerEncoder = m_hangerRightmotor.getEncoder();
    m_leftHangerEncoder = m_hangerLeftmotor.getEncoder();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Left Hall", getLeftLimit());
    SmartDashboard.putBoolean("Right Hall", getRightLimit());

    SmartDashboard.putNumber("Left Enc", m_leftHangerEncoder.getPosition());
    SmartDashboard.putNumber("Right Enc", m_rightHangerEncoder.getPosition());
  }

  //   private void spinLeftUp(){
  //     m_hangerLeftmotor.set(.1);
  //   }
  //   private void spinLeftDown(){
  //     m_hangerLeftmotor.set(-.1);
  //   }
  //   private void spinRightUp(){
  //     m_hangerLeftmotor.set(.1);
  //   }
  //   private void spinRightDown(){
  //     m_hangerLeftmotor.set(-.1);
  //   }

  private boolean getLeftLimit() {
    return !m_Lefthall.get();
  }

  private boolean getRightLimit() {
    return !m_Righthall.get();
  }

  public void resetEncs() {
    m_leftHangerEncoder.setPosition(0); //79
    m_rightHangerEncoder.setPosition(0); //74
  }

  public Command hangToTop() {
    return setPowerCommand(() -> 5.0);
  }

  public Command hangToTopStop(){
    return hangToTop().until(() -> m_leftHangerEncoder.getPosition() > 75)
    .andThen(setPowerCommand(() -> 0.0));
  }

  private void setPower(double leftVoltage, double rightVoltage) {
    if (leftVoltage > 0 || !getLeftLimit()) {
      m_hangerLeftmotor.setVoltage(leftVoltage);
    } else {
      m_hangerLeftmotor.stopMotor();
    }

    if (rightVoltage > 0 || !getRightLimit()) {
      m_hangerRightmotor.setVoltage(rightVoltage);
    } else {
      m_hangerRightmotor.stopMotor();
    }
  }

  public Command setPowerCommand(Supplier<Double> voltage) {
    return new RunCommand(
        () -> {
          setPower(voltage.get(), voltage.get());
        },
        this);
  }
}
