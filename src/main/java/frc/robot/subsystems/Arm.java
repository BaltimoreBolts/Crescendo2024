package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.library.WpiTimeSource;
import java.util.function.Supplier;
import org.growingstems.logic.LogicCore.Edge;
import org.growingstems.math.RangeU;
import org.growingstems.measurements.Angle;
import org.growingstems.measurements.Measurements.AngularAcceleration;
import org.growingstems.measurements.Measurements.AngularVelocity;
import org.growingstems.measurements.Measurements.Voltage;
import org.growingstems.measurements.Measurements.VoltagePerFrequency;
import org.growingstems.signals.EdgeDetector;
import org.growingstems.util.timer.Timer;

public class Arm extends SubsystemBase {
  private enum SpeedMode {
    NORMAL,
    SLOW
  }

  private final CANSparkMax m_LeftArmMasterMotor = new CANSparkMax(12, MotorType.kBrushless);
  private final DutyCycleEncoder m_AbsEncoder = new DutyCycleEncoder(4);
  private final RelativeEncoder m_RelativeEncoder;

  private static final Angle k_slowThreshold = Angle.degrees(70.0);
  private static final Angle k_allowableError = Angle.degrees(2.0);

  private static final AngularVelocity k_cruiseVelocity = AngularVelocity.degreesPerSecond(140.0);
  private static final AngularAcceleration k_acceleration =
      AngularAcceleration.degreesPerSecondSquared(40); //100 

  private static final Constraints k_profiledConstraints = new Constraints(
      k_cruiseVelocity.asRadiansPerSecond(), k_acceleration.asRadiansPerSecondSquared());

  private static final AngularVelocity k_cruiseVelocitySlow =
      AngularVelocity.degreesPerSecond(20.0);
  private static final AngularAcceleration k_accelerationSlow =
      AngularAcceleration.degreesPerSecondSquared(10.0);

  private static final Constraints k_profiledConstraintsSlow = new Constraints(
      k_cruiseVelocitySlow.asRadiansPerSecond(), k_accelerationSlow.asRadiansPerSecondSquared());

  private final ProfiledPIDController m_positionPid =
      new ProfiledPIDController(6.0, 0.0, 0.0, k_profiledConstraints);

  private static final Voltage k_gravityCompensation = Voltage.volts(0.2);

  /** Voltage per Frequency (Voltage per AngularVelocity) */
  private static final VoltagePerFrequency k_velocityCompensation =
      Voltage.volts(2.25).div(new AngularVelocity(1.0)); // 2.25, 1.0

  private static final Angle k_reverseRawAbsoluteHardStop_SU = Angle.degrees(-306.0);

  private static final Angle k_reverseAbsoluteHardStop = Angle.degrees(-2.0);

  private static final RangeU<Angle> k_safeRange =
      new RangeU<>(k_reverseAbsoluteHardStop, Angle.degrees(100.0));

  private static final Voltage k_maxVoltage = Voltage.volts(12.0);
  private static final RangeU<Voltage> k_voltageRange =
      new RangeU<>(k_maxVoltage.neg(), k_maxVoltage);

  private Angle m_relOffset = Angle.ZERO;

  private static final double k_anglePerSensorUnit = 24.0 / 48.0;

  private final Timer m_trajectoryTimer = new WpiTimeSource().createTimer();
  private Angle m_requestAngleGoal = Angle.ZERO;
  private SpeedMode m_speedMode = SpeedMode.NORMAL;
  private Angle m_angleGoal = Angle.ZERO;
  private State m_trajectorySetpoint = new State();
  private boolean m_runPositionControl = false;
  private EdgeDetector m_startPositionControl = new EdgeDetector(Edge.RISING);

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

    SmartDashboard.putNumber("arm/set arm Pos", 0.0);

    SmartDashboard.putNumber("arm/trajectory position", 0);
    SmartDashboard.putNumber("arm/trajectory velocity", 0);

    new WaitCommand(0.5)
        .andThen(this::calculateRelativeOffset)
        .ignoringDisable(true)
        .schedule();

    m_trajectoryTimer.start();
  }

  @Override
  public void periodic() {
    // Check if we just switched to position control
    var positionControlStarted = m_startPositionControl.update(m_runPositionControl);

    if (m_runPositionControl) {
      m_angleGoal = m_requestAngleGoal;

      setPosition(m_angleGoal, m_speedMode);
    }

    SmartDashboard.putBoolean("arm/in set pos", m_runPositionControl);

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
    SmartDashboard.putBoolean("Nick's Test", positionControlStarted);
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
    m_LeftArmMasterMotor.setVoltage(
        k_voltageRange.coerceValue(voltage.add(getCurrentGravityCompensation())).asVolts());
  }

  private void setPosition(Angle goal, SpeedMode speedMode) {
    var constraints =
        switch (speedMode) {
          case NORMAL -> k_profiledConstraints;
          case SLOW -> k_profiledConstraintsSlow;
        };

    m_positionPid.setConstraints(constraints);

    var power =
        Voltage.volts(m_positionPid.calculate(getRelativePosition().asRadians(), goal.asRadians()));
    var position = Angle.radians(m_positionPid.getSetpoint().position);
    var velocity = AngularVelocity.radiansPerSecond(m_positionPid.getSetpoint().velocity);

    // var gravPower = getCurrentGravityCompensation();
    var velPower = k_velocityCompensation.mul(velocity);
    var compensatedPower = power.add(velPower);

    SmartDashboard.putNumber("arm/trajectory position", position.asDegrees());
    SmartDashboard.putNumber("arm/trajectory velocity", velocity.asDegreesPerSecond());
    SmartDashboard.putNumber("arm/trajectory power", power.asVolts());
    // SmartDashboard.putNumber("arm/trajectory grav power", gravPower.asVolts());
    SmartDashboard.putNumber("arm/trajectory vel power", velPower.asVolts());

    setPower(compensatedPower);
  }

  private boolean isAtPosition(Angle goal) {
    return goal.sub(getRelativePosition()).abs().lt(k_allowableError);
  }

  public boolean isAmpPos() {
    return getRelativePosition().gt(k_slowThreshold);
  }

  private Command setPositionCommand(Angle position, SpeedMode speedMode) {
    return new InstantCommand(
            () -> {
              m_runPositionControl = true;
              m_speedMode = speedMode;
              m_requestAngleGoal = k_safeRange.coerceValue(position);
            },
            this)
        .andThen(new WaitUntilCommand(() -> isAtPosition(position)));
  }

  public Command setPositionCommand(Angle goal) {
    var goToSafePos = setPositionCommand(k_slowThreshold, SpeedMode.SLOW)
        .andThen(new WaitCommand(0.1))
        .onlyIf(() -> goal.lt(k_slowThreshold) && getRelativePosition().gt(k_slowThreshold));
    var goToGoal = setPositionCommand(goal, SpeedMode.NORMAL);

    return goToSafePos.andThen(goToGoal);
  }

  private Command setPowerCommand(Supplier<Voltage> voltage) {
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
