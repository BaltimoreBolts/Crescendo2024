package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;
import frc.robot.utils.LEDlights;

public class IntakeCommands extends SequentialCommandGroup {

  public Command intakeNote(Intake intake) {
    return intake.intakeFastCommand();
  }

  public Command intakeNoteIntake(Intake intake) {
    SmartDashboard.putNumber("Spot", 1);
    return intake.intakeFastCommand().until(intake.seeIntakeSupplier());
  }

  public Command intakeNoteSlow(Intake intake) {
    SmartDashboard.putNumber("Spot", 2);
    return intake.intakeSlowCommand().until(intake.seeShooterSupplier());
  }

  public Command intakeNoteTime(Intake intake) {
    return (intakeNote(intake).andThen(new WaitCommand(6))).andThen(intake.intakeOffCommand());
  }

  public Command intakeNoteToBottom(Intake intake) {
    return intakeNoteTime(intake).until(intake.seeIntakeSupplier());
  }

  public Command outakeNoteTime(Intake intake) {
    return (intake.outakeFastCommand().andThen(new WaitCommand(3)))
        .andThen(intake.intakeOffCommand());
  }

  public Command intakeNoteTimeSlow(Intake intake) {
    return (intakeNoteSlow(intake).andThen(new WaitCommand(6))).andThen(intake.intakeOffCommand());
  }

  // stops note in front of shooter - gets too close due to notes momentum/robot reaction time
  public Command intakeNoteStop(Intake intake) {
    return (intakeNoteTimeSlow(intake)
        .until(intake.seeShooterSupplier())
        .andThen(intake.intakeOffCommand()));
  }

  public Command amazingIntaking(Intake intake) {
    return (intakeNoteTime(intake)
            .until(intake.seeShooterSupplier())
            .andThen(intake.outtakeCommand())
            .until(intake.seeIntakeSupplier())
            .andThen(intake.intakeOffCommand()))
        .withTimeout(10);
  }

  public Command amazingIntaking3(Intake intake) {
    return (intakeNoteToBottom(intake)
            .andThen(intake.outtakeCommand())
            .andThen(() -> LEDlights.intakeColor()))
        .withTimeout(10);
  }
}
