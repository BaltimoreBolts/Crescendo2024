package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.Arm;

public class Shooter extends SubsystemBase {

  public double power = 0;

  private CANSparkMax shooterMotor1;
  private CANSparkMax shooterMotor2;

  public Shooter() {

    this.shooterMotor1 = new CANSparkMax(15, MotorType.kBrushless);
    this.shooterMotor1.restoreFactoryDefaults();
    this.shooterMotor1.setIdleMode(IdleMode.kCoast);
    this.shooterMotor1.setSmartCurrentLimit(40);

    this.shooterMotor2 = new CANSparkMax(16, MotorType.kBrushless);
    this.shooterMotor2.restoreFactoryDefaults();
    this.shooterMotor2.setIdleMode(IdleMode.kCoast);
    this.shooterMotor2.setSmartCurrentLimit(40);

    this.shooterMotor2.follow(this.shooterMotor1, false);

    // this.shooterMotor1.setOpenLoopRampRate(0.5);
    // this.shooterMotor2.setOpenLoopRampRate(0.5);

    this.shooterMotor1.burnFlash();
    this.shooterMotor2.burnFlash();

    SmartDashboard.putNumber("shooter/Speed", 0.0);
  }

  @Override
  public void periodic() {
    this.shooterMotor1.set(power);
    // this.shooterMotor2.set(-power);
  }

  public Command shooterOffCommand() {
    return new InstantCommand(() -> shooterOff());
  }

  public Command shooterSpinSpeaker() {
    return new InstantCommand(() -> shooterAtSpeed());
  }

  public Command shooterSpinAmp() {
    return new InstantCommand(() -> shooterAmpSpeed());
  }


  public void shooterOff() {
    this.power = 0;
  }

  public void shooterAtSpeed() {
    this.power = 0.6;
  }

  public void shooterAmpSpeed() {
    this.power = 0.3;
  }
}
