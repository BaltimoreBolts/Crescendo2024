package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;

public class IntakeCommands extends SequentialCommandGroup{

  public Command intakeNote(Intake intake){
    return intake.intakeFastCommand().until(intake.seeShooterSupplier());
  }
  public Command intakeNoteSlow(Intake intake){
    return intake.intakeSlowCommand().until(intake.seeShooterSupplier());
  }

  public Command intakeNoteTime(Intake intake){
    return (intakeNote(intake).andThen(new WaitCommand(6))).andThen(intake.intakeOffCommand());
  }
  public Command outakeNoteTime(Intake intake){
    return (intake.outakeFastCommand().andThen(new WaitCommand(3))).andThen(intake.intakeOffCommand());
  }
  public Command intakeNoteTimeSlow(Intake intake){
    return (intakeNoteSlow(intake).andThen(new WaitCommand(6))).andThen(intake.intakeOffCommand());
  }

  // stops note in front of shooter - gets too close due to notes momentum/robot reaction time
  public Command intakeNoteStop(Intake intake){
    return (intakeNoteTimeSlow(intake).until(intake.seeShooterSupplier()).andThen(intake.intakeOffCommand()));
  }

  // new idea created 2/29/2024 at 11:15 PM
  // timeout at end is just a percausion 
  // in theory the command should pull the note in quickly and then push it outwards until it gets
  // to the bottom sensor (when it sees the bottom sensor it will still probably be too high but)
  public Command amazingIntaking(Intake intake){
    return (intakeNoteTime(intake).until(intake.seeShooterSupplier()).andThen(intake.outtakeCommand())
    .until(intake.seeIntakeSupplier()).andThen(intake.intakeOffCommand())).withTimeout(10);
  }

  // may be good assuming the note fully goes past top sensor on way in
  // outaking fast to get the note further down while robot is reacting
  public Command amazingIntaking2(Intake intake){
    return (intakeNoteTime(intake).until(intake.seeShooterSupplier()).andThen(intake.outakeFastCommand())
    .until(intake.seeShooterSupplier()).andThen(intake.intakeOffCommand())).withTimeout(10);
  }


}
