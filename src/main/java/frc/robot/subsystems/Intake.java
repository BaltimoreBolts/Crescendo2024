package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

// import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.IntakeCommands;

import java.util.function.BooleanSupplier;

public class Intake extends SubsystemBase {

  public double power = 0;

  private CANSparkMax intakeMotor;

  private RelativeEncoder intakeMotorEncoder;

  private DigitalInput noteAtIntake;
  private DigitalInput noteAtShooter;

  public Intake() {

    this.intakeMotor = new CANSparkMax(14, MotorType.kBrushless);
    this.intakeMotor.setIdleMode(IdleMode.kBrake);
    this.intakeMotor.setSmartCurrentLimit(40);
    this.intakeMotor.burnFlash();
    intakeMotorEncoder = intakeMotor.getEncoder();

    noteAtIntake = new DigitalInput(0);

    noteAtShooter = new DigitalInput(1);
  }

  final double intakeFastSpeed = 0.5;
  final double intakeMediumSpeed = 0.4;

  final double intakeSlowSpeed = 0.05;

  final double outakeSpeed = 0.08;
  final double outakeFastSpeed = 0.5;

  @Override
  public void periodic() {
    this.intakeMotor.set(power);
    SmartDashboard.putBoolean("At Shooter", noteAtShooter.get());
    SmartDashboard.putBoolean("At Intake", noteAtIntake.get());
  }

  public Command intakeOffCommand() {
    SmartDashboard.putNumber("Spot", 3);
    return new InstantCommand(() -> intakeOff());
  }

  public Command outtakeCommand() {
    return new InstantCommand(() -> outtake());
  }

  public Command outakeFastCommand() {
    return new InstantCommand(() -> outakeFast());
  }

  public Command intakeFastCommand() {
    return new InstantCommand(() -> intakeFast());
  }

  public Command intakeMediumCommand() {
    return new InstantCommand(() -> intakeMedium());
  }

  public Command intakeSlowCommand() {
    return new InstantCommand(() -> intakeSlow());
  }

  // public Command intakeOutCount() {
  //   intakeMotorEncoder.setPosition(0.0);
  //   return new InstantCommand(() -> if(intakeMotorEncoder < 10) {intakeSlow()} else {intakeOff()});
  // }

  public BooleanSupplier seeShooterSupplier() {
    return noteAtShooter::get;
  }

  public BooleanSupplier seeIntakeSupplier() {
    return noteAtIntake::get;
  }

  public boolean getNotIntake(){
    return !noteAtIntake.get();
  }

  public BooleanSupplier invertedSeeIntakeSupplier() {
    // Define the inverted supplier inline
    return () -> !noteAtIntake.get();
}

  public void intakeOff() {
    this.power = 0;
  }

  private void outtake() {
    this.power = outakeSpeed;
  }

  private void outakeFast() {
    this.power = outakeFastSpeed;
  }

  public void intakeFast() {
    this.power = -intakeFastSpeed;
  }

  public void intakeMedium() {
    this.power = -intakeMediumSpeed;
  }

  public void intakeSlow() {
    this.power = -intakeSlowSpeed;
  }
}
