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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import java.util.function.Supplier;
import org.growingstems.measurements.Angle;
import org.growingstems.measurements.Measurements.AngularAcceleration;
import org.growingstems.measurements.Measurements.AngularVelocity;
import org.growingstems.measurements.Measurements.Voltage;

public class Arm extends SubsystemBase {

  private final CANSparkMax m_LeftArmMasterMotor = new CANSparkMax(12, MotorType.kBrushless);

  private final DutyCycleEncoder m_AbsEncoder = new DutyCycleEncoder(4);

  private final RelativeEncoder m_RelativeEncoder;

  private final SparkPIDController m_positionController;

  private static final double k_gravityCompDutyCycle = 0.04;

  private static final Angle k_reverseAbsoluteHardStop_SU = Angle.degrees(-290.3);
  private static final Angle k_forwardAbsoluteHardStop_SU = Angle.degrees(-21.9 * 4.0);

  private static final Angle m_absOffset = Angle.degrees(-2.0);
  private Angle m_relOffset = Angle.ZERO;

  private static final double k_anglePerSensorUnit = 16.0 / 32.0;

  private final TrapezoidProfile m_tragectoryContoller = new TrapezoidProfile(new Constraints(
      AngularVelocity.degreesPerSecond(10.0).asRadiansPerSecond(),
      AngularAcceleration.degreesPerSecondSquared(10).asRadiansPerSecondSquared()));

  private Angle m_angleGoal = Angle.ZERO;

  //   private final RelativeEncoder m_LeftArmEncoder;
  //   private final RelativeEncoder m_RightArmEncoder;

  public Arm() {
    var rightArmFollowerMotor = new CANSparkMax(13, MotorType.kBrushless);

    m_LeftArmMasterMotor.restoreFactoryDefaults();
    rightArmFollowerMotor.restoreFactoryDefaults();

    m_LeftArmMasterMotor.setInverted(true);
    rightArmFollowerMotor.follow(m_LeftArmMasterMotor, true);

    m_LeftArmMasterMotor.setIdleMode(IdleMode.kBrake);
    rightArmFollowerMotor.setIdleMode(IdleMode.kBrake);

    // m_RightArmEncoder = m_RightArmMotor.getEncoder();
    // m_LeftArmEncoder = m_LeftArmMotor.getEncoder();

    m_RelativeEncoder = m_LeftArmMasterMotor.getAlternateEncoder(8192);
    m_RelativeEncoder.setInverted(true);

    m_AbsEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);

    m_positionController = m_LeftArmMasterMotor.getPIDController();
    m_positionController.setFeedbackDevice(m_RelativeEncoder);
    m_positionController.setP(5.0);

    SmartDashboard.putNumber("set arm Pos", 0.0);

    new WaitCommand(0.5).andThen(this::findRelativeOffset).ignoringDisable(true).schedule();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("getRawAbsPos", getRawAbsPos_SU().asDegrees());
    SmartDashboard.putNumber("getAbsPos", getAbsPos().asDegrees());
    SmartDashboard.putNumber("getRawRelPos", getRawRelPos_SU().asDegrees());
    SmartDashboard.putNumber("getRelPos()", getRelPos().asDegrees());
    SmartDashboard.putNumber("m_relOffset()", m_relOffset.asDegrees());

    var rawPos = angleToSensorUnits(Angle.degrees(SmartDashboard.getNumber("set arm Pos", 0.0)));
    SmartDashboard.putNumber("set raw position", rawPos.asDegrees());

    SmartDashboard.putNumber("output power", m_LeftArmMasterMotor.getAppliedOutput());
  }

  private State getCurrentState() {
    return new State(
        getRelPos().asRadians(),
        AngularVelocity.revolutionsPerMinute(m_RelativeEncoder.getVelocity()).asRadiansPerSecond());
  }

  private State getGoalState() {
    return new State(m_angleGoal.asRadians(), AngularVelocity.ZERO.asRadiansPerSecond());
  }

  private void findRelativeOffset() {
    m_relOffset = getAbsPos().sub(sensorUnitToAngle(getRawRelPos_SU()));
  }

  private Angle getRawAbsPos_SU() {
    return Angle.rotations(-m_AbsEncoder.getAbsolutePosition());
  }

  private Angle getAbsPos() {
    return sensorUnitToAngle(getRawAbsPos_SU().sub(k_reverseAbsoluteHardStop_SU))
        .add(m_absOffset);
  }

  private Angle getRawRelPos_SU() {
    return Angle.rotations(m_RelativeEncoder.getPosition());
  }

  private Angle getRelPos() {
    return getRawRelPos_SU().mul(k_anglePerSensorUnit).add(m_relOffset);
  }

  private Angle sensorUnitToAngle(Angle sensorAngle_SU) {
    return sensorAngle_SU.mul(k_anglePerSensorUnit);
  }

  private Angle angleToSensorUnits(Angle armAngle) {
    return armAngle.sub(m_relOffset).div(k_anglePerSensorUnit);
  }

  private void setPower(Voltage voltage) {
    m_LeftArmMasterMotor.setVoltage(voltage.asVolts());
  }

  private void setPosition(Angle position) {
    m_positionController.setReference(angleToSensorUnits(position).asRotations(), ControlType.kPosition, 0, Math.cos(getRelPos().asRadians()) * k_gravityCompDutyCycle);
  }

  public Command setPositionCommand(Supplier<Angle> position) {
    return new RunCommand(
        () -> {
          setPosition(position.get());
        },
        this);
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
          m_LeftArmMasterMotor.stopMotor();
        },
        this);
  }
}
