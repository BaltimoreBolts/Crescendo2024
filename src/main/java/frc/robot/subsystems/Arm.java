package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.library.WpiTimeSource;
import java.util.function.Supplier;
import org.growingstems.logic.LogicCore.Edge;
import org.growingstems.measurements.Angle;
import org.growingstems.measurements.Measurements.AngularAcceleration;
import org.growingstems.measurements.Measurements.AngularVelocity;
import org.growingstems.measurements.Measurements.Voltage;
import org.growingstems.measurements.Measurements.VoltagePerFrequency;
import org.growingstems.signals.EdgeDetector;
import org.growingstems.util.timer.Timer;

public class Arm extends SubsystemBase {
  private final CANSparkMax m_LeftArmMasterMotor = new CANSparkMax(12, MotorType.kBrushless);
  private final DutyCycleEncoder m_AbsEncoder = new DutyCycleEncoder(4);
  private final RelativeEncoder m_RelativeEncoder;
  private final SparkPIDController m_positionController;

  private static final Voltage k_gravityCompensation = Voltage.volts(0.5);
  /** Voltage per Frequency (Voltage per AngularVelocity) */
  private static final VoltagePerFrequency k_velocityCompensation =
      Voltage.volts(0.0).div(AngularVelocity.ZERO);

  private static final Angle k_reverseRawAbsoluteHardStop_SU = Angle.degrees(-265.2);
  private static final Angle k_forwardRawAbsoluteHardStop_SU = Angle.degrees(-21.9 * 4.0);

  private static final Angle k_reverseAbsoluteHardStop = Angle.degrees(-2.0);

  private Angle m_relOffset = Angle.ZERO;

  private static final double k_anglePerSensorUnit = 16.0 / 32.0;

  private static final AngularVelocity k_cruiseVelocity = AngularVelocity.degreesPerSecond(25.0);
  private static final AngularAcceleration k_acceleration =
      AngularAcceleration.degreesPerSecondSquared(10);

  private final Timer m_trajectoryTimer = new WpiTimeSource().createTimer();
  private Angle m_angleGoal = Angle.ZERO;
  private State m_trajectorySetpoint = new State();
  private boolean m_runPositionControl = false;
  private EdgeDetector m_startPositionControl = new EdgeDetector(Edge.RISING);

  private final TrapezoidProfile m_trajectoryContoller = new TrapezoidProfile(new Constraints(
      k_cruiseVelocity.asRadiansPerSecond(), k_acceleration.asRadiansPerSecondSquared()));

  public Arm() {
    @SuppressWarnings("resource")
    var rightArmFollowerMotor = new CANSparkMax(13, MotorType.kBrushless);

    m_LeftArmMasterMotor.restoreFactoryDefaults();
    rightArmFollowerMotor.restoreFactoryDefaults();

    m_LeftArmMasterMotor.setInverted(true);
    rightArmFollowerMotor.follow(m_LeftArmMasterMotor, true);

    m_LeftArmMasterMotor.setIdleMode(IdleMode.kBrake);
    rightArmFollowerMotor.setIdleMode(IdleMode.kBrake);

    m_RelativeEncoder = m_LeftArmMasterMotor.getAlternateEncoder(8192);
    m_RelativeEncoder.setInverted(true);

    m_AbsEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);

    m_positionController = m_LeftArmMasterMotor.getPIDController();
    m_positionController.setFeedbackDevice(m_RelativeEncoder);
    m_positionController.setP(2.0); //gain

    SmartDashboard.putNumber("arm/set arm Pos", 0.0);

    new WaitCommand(0.5)
        .andThen(this::calculateRelativeOffset)
        .ignoringDisable(true)
        .schedule();

    m_trajectoryTimer.start();
  }

  @Override
  public void periodic() {
    var startedPositionControl = m_startPositionControl.update(m_runPositionControl);
    if (m_runPositionControl) {
      var currentState = getCurrentState(startedPositionControl);
      m_trajectorySetpoint = m_trajectoryContoller.calculate(
          m_trajectoryTimer.reset().asSeconds(), currentState, getGoalState());

      var trajectoryPos = Angle.radians(m_trajectorySetpoint.position);
      var trajectoryVel = AngularVelocity.radiansPerSecond(m_trajectorySetpoint.velocity);

      setPosition(trajectoryPos, trajectoryVel);

      SmartDashboard.putNumber("arm/trajectory position", trajectoryPos.asDegrees());
      SmartDashboard.putNumber("arm/trajectory velocity", trajectoryVel.asDegreesPerSecond());
    }

    SmartDashboard.putBoolean("use trajectory", m_runPositionControl);

    SmartDashboard.putNumber("arm/getRawAbsPos", getRawAbsolutePosition_SU().asDegrees());
    SmartDashboard.putNumber("arm/getAbsPos", getAbsolutePosition().asDegrees());
    SmartDashboard.putNumber("arm/getRawRelPos", getRawRelativePosition_SU().asDegrees());
    SmartDashboard.putNumber("arm/getRelPos()", getRelativePosition().asDegrees());
    SmartDashboard.putNumber("arm/m_relOffset()", m_relOffset.asDegrees());

    var rawPos = angleToSensorUnits(Angle.degrees(SmartDashboard.getNumber("set arm Pos", 0.0)));
    SmartDashboard.putNumber("arm/set raw position", rawPos.asDegrees());
    SmartDashboard.putNumber("arm/goal position", m_angleGoal.asDegrees());

    SmartDashboard.putNumber("arm/output power", m_LeftArmMasterMotor.getAppliedOutput());

    SmartDashboard.putNumber(
        "arm/angle Setpoint", Angle.radians(m_trajectorySetpoint.position).asDegrees());
    SmartDashboard.putBoolean("Nick's Test", startedPositionControl);
  }

  private State getCurrentState(boolean reset) {
    if (reset) {
      m_trajectorySetpoint = new State(
          getRelativePosition().asRadians(),
          AngularVelocity.revolutionsPerMinute(m_RelativeEncoder.getVelocity())
              .asRadiansPerSecond());
    }

    return m_trajectorySetpoint;
  }

  private State getGoalState() {
    return new State(m_angleGoal.asRadians(), AngularVelocity.ZERO.asRadiansPerSecond());
  }

  private void calculateRelativeOffset() {
    m_relOffset = getAbsolutePosition().sub(sensorUnitToAngle(getRawRelativePosition_SU()));
  }

  private Angle getRawAbsolutePosition_SU() {
    return Angle.rotations(-m_AbsEncoder.getAbsolutePosition());
  }

  private Angle getAbsolutePosition() {
    return sensorUnitToAngle(getRawAbsolutePosition_SU().sub(k_reverseRawAbsoluteHardStop_SU))
        .add(k_reverseAbsoluteHardStop);
  }

  private Angle getRawRelativePosition_SU() {
    return Angle.rotations(m_RelativeEncoder.getPosition());
  }

  private Angle getRelativePosition() {
    return getRawRelativePosition_SU().mul(k_anglePerSensorUnit).add(m_relOffset);
  }

  private Angle sensorUnitToAngle(Angle sensorAngle_SU) {
    return sensorAngle_SU.mul(k_anglePerSensorUnit);
  }

  private Angle angleToSensorUnits(Angle armAngle) {
    return armAngle.sub(m_relOffset).div(k_anglePerSensorUnit);
  }

  private Voltage getCurrentGravityCompensation() {
    return k_gravityCompensation.mul(Math.cos(getRelativePosition().asRadians()));
  }

  private void setPower(Voltage voltage) {
    m_LeftArmMasterMotor.setVoltage(voltage.add(getCurrentGravityCompensation()).asVolts());
  }

  private void setPosition(Angle position, AngularVelocity velocity) {
    m_positionController.setReference(
        angleToSensorUnits(position).asRotations(),
        ControlType.kPosition,
        0,
        getCurrentGravityCompensation()
            .add(k_velocityCompensation.mul(velocity))
            .asVolts());
  }

  public Command setPositionCommand(Supplier<Angle> position) {
    return new InstantCommand(() -> {
          m_runPositionControl = true;
        })
        .andThen(new RunCommand(
            () -> {
              if (position.get().asDegrees() < -2) {
                m_angleGoal = Angle.degrees(-2.0);
              } else if (position.get().asDegrees() > 90) {
                m_angleGoal = Angle.degrees(90);
              } else {
                m_angleGoal = position.get();
              }
            },
            this))
        .finallyDo(() -> m_runPositionControl = false);
  }

  public Command setPowerCommand(Supplier<Voltage> voltage) {
    return new InstantCommand(() -> m_runPositionControl = false)
        .andThen(new RunCommand(
            () -> {
              setPower(voltage.get());
            },
            this));
  }

  public Command emergencyStopCommand() {
    return new InstantCommand(() -> m_runPositionControl = false)
        .andThen(new RunCommand(
            () -> {
              m_LeftArmMasterMotor.stopMotor();
            },
            this));
  }
}
