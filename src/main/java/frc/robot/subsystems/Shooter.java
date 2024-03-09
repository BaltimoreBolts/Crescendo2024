package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Shooter extends PIDSubsystem {
  private final CANSparkMax m_shooterMotor = new CANSparkMax(15, MotorType.kBrushless);
  private final CANSparkMax m_shooterMotor2 = new CANSparkMax(16, MotorType.kBrushless);

  //private final CANSparkMax m_feederMotor = new CANSparkMax(14, MotorType.kBrushless);

  private final RelativeEncoder m_shooterEncoder;


  private final SimpleMotorFeedforward m_shooterFeedforward =
      new SimpleMotorFeedforward(
          0.05, 12/5000, 0);

  /** The shooter subsystem for the robot. */
  public Shooter() {

    super(new PIDController(0.01, 0, 0));

    m_shooterMotor2.follow(m_shooterMotor, true);

    m_shooterEncoder = m_shooterMotor.getEncoder();
    getController().setTolerance(100);
    setSetpoint(2000);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    m_shooterMotor.setVoltage(output + m_shooterFeedforward.calculate(setpoint));
  }


  @Override
  public double getMeasurement() {
    return m_shooterEncoder.getVelocity();
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }

  // public void runFeeder() {
  //   m_feederMotor.set(0.4);
  // }

  // public void stopFeeder() {
  //   m_feederMotor.set(0);
  // }
}