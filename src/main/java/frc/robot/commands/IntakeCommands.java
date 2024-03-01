package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class IntakeCommands extends SequentialCommandGroup{

  public Command intakeNote(Intake intake){
    return intake.intakeFastCommand().until(intake.seeShooterSupplier());
  }

  public Command intakeNoteTime(Intake intake){
    return (intakeNote(intake).andThen(new WaitCommand(6))).andThen(intake.intakeOffCommand());
  }

  public Command intakeNoteStop(Intake intake){
    return (intakeNoteTime(intake).until(intake.seeShooterSupplier()).andThen(intake.intakeOffCommand()));
  }


}
