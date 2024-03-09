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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Shooter extends SubsystemBase {
  // The motor on the shooter wheel .
  private final CANSparkMax m_shooterMotor = new CANSparkMax(15, MotorType.kBrushless);
  private final CANSparkMax m_shooterMotor2 = new CANSparkMax(16, MotorType.kBrushless);


  // The motor on the feeder wheels.
  private final CANSparkMax m_feederMotor = new CANSparkMax(14, MotorType.kBrushless);

  // The shooter wheel encoder

  private RelativeEncoder m_shooterEncoder;

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Angle> m_angle = mutable(Rotations.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));

  // Create a new SysId routine for characterizing the shooter.
  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
      // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
          // Tell SysId how to plumb the driving voltage to the motor(s).
          (Measure<Voltage> volts) -> {
            m_shooterMotor.setVoltage(volts.in(Volts));
          },
          // Tell SysId how to record a frame of data for each motor on the mechanism being
          // characterized.
          log -> {
            // Record a frame for the shooter motor.
            log.motor("shooter-wheel")
                .voltage(m_appliedVoltage.mut_replace(
                    m_shooterMotor.get() * RobotController.getBatteryVoltage(), Volts))
                .angularPosition(m_angle.mut_replace(m_shooterEncoder.getPosition(), Rotations))
                .angularVelocity(
                    m_velocity.mut_replace(m_shooterEncoder.getVelocity(), RotationsPerSecond));
          },
          // Tell SysId to make generated commands require this subsystem, suffix test state in
          // WPILog with this subsystem's name ("shooter")
          this));
  // PID controller to run the shooter wheel in closed-loop, set the constants equal to those
  // calculated by SysId
  private final PIDController m_shooterFeedback = new PIDController(1, 0, 0);
  // Feedforward controller to run the shooter wheel in closed-loop, set the constants equal to
  // those calculated by SysId
  private final SimpleMotorFeedforward m_shooterFeedforward =
      new SimpleMotorFeedforward(0.05, 12 / 5000, 0);

  /** Creates a new Shooter subsystem. */
  public Shooter() {

    SmartDashboard.putNumber("shooter/set Shooter Speed", 0.0);

    m_shooterMotor.restoreFactoryDefaults();
    m_shooterMotor2.restoreFactoryDefaults();

    m_shooterMotor.setInverted(false);
    m_shooterMotor2.follow(m_shooterMotor, true);

    m_shooterMotor.setIdleMode(IdleMode.kCoast);
    m_shooterMotor2.setIdleMode(IdleMode.kCoast);

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("shooter/Shooter RPM", m_shooterEncoder.getVelocity());
  }
  /**
   * Returns a command that runs the shooter at a specifc velocity.
   *
   * @param shooterSpeed The commanded shooter wheel speed in rotations per second
   */
  public Command runShooter(DoubleSupplier shooterSpeed) {
    // Run shooter wheel at the desired speed using a PID controller and feedforward.
    return run(() -> {
          m_shooterMotor.setVoltage(
              m_shooterFeedback.calculate(m_shooterEncoder.getVelocity(), shooterSpeed.getAsDouble())
                  + m_shooterFeedforward.calculate(shooterSpeed.getAsDouble()));
          m_shooterMotor2.setVoltage(
              m_shooterFeedback.calculate(m_shooterEncoder.getVelocity(), shooterSpeed.getAsDouble())
                  + m_shooterFeedforward.calculate(shooterSpeed.getAsDouble()));
          m_feederMotor.set(0.7);
        })
        .finallyDo(() -> {
          m_shooterMotor.stopMotor();
          m_shooterMotor2.stopMotor();
          m_feederMotor.stopMotor();
        })
        .withName("runShooter");
  }


  /**
   * Returns a command that will execute a quasistatic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}
