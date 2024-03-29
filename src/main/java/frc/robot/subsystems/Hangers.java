package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
  public void periodic(){
    SmartDashboard.putBoolean("Left Hall", getLeftLimit());
    SmartDashboard.putBoolean("Right Hall", getRightLimit());
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

    private void setPower(double leftVoltage, double rightVoltage) {
        if(leftVoltage > 0 || !getLeftLimit()) {
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
    return new RunCommand(() -> {
        setPower(voltage.get(), voltage.get());
    }, this);
  }

}
