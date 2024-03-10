package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.IntakeCommands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Hangers;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public final CommandXboxController driver =
      new CommandXboxController(Constants.kControls.DRIVE_JOYSTICK_ID);

  public final CommandXboxController driver2 =
      new CommandXboxController(Constants.kControls.DRIVE_JOYSTICK_ID2);

  public final Swerve swerve;

  public final Intake intake;

  public final IntakeCommands intakeCommands;

  public final Hangers m_hangers = new Hangers();

  public final Shooter m_shooter = new Shooter();

  public final Arm m_arm = new Arm();

  // public final AutoCommands auto;

  public RobotContainer() {
    // driver = new CommandXboxController(Constants.kControls.DRIVE_JOYSTICK_ID);

    // driver2 = new CommandXboxController(Constants.kControls.DRIVE_JOYSTICK_ID2);

    swerve = new Swerve();
    intake = new Intake();
    intakeCommands = new IntakeCommands();

    SmartDashboard.putNumber("drive/speed", 0.0);
    SmartDashboard.putNumber("drive/velocity(RPM)", 0.0);

    SmartDashboard.putBoolean("At Shooter", false);
    SmartDashboard.putBoolean("At Intake", false);
    SmartDashboard.putNumber("Spot", 0);
    // auto = new AutoCommands(swerve);

    // Configure button bindings
    configureButtonBindings();
  }

  public void update() {
    var joystickPos = -driver.getRawAxis(Constants.kControls.TRANSLATION_Y_AXIS);
    SmartDashboard.putNumber("Joystick Pos", joystickPos);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // swerve.setDefaultCommand(swerve.drive(
    //   () ->
    // -Constants.kControls.X_DRIVE_LIMITER.calculate(driver.getRawAxis(Constants.kControls.TRANSLATION_Y_AXIS)),
    //   () ->
    // -Constants.kControls.Y_DRIVE_LIMITER.calculate(driver.getRawAxis(Constants.kControls.TRANSLATION_X_AXIS)),
    //   () ->
    // -Constants.kControls.THETA_DRIVE_LIMITER.calculate(driver.getRawAxis(Constants.kControls.ROTATION_AXIS)),
    //   true,
    //   true
    // ));

    driver.y().onTrue(new InstantCommand(() -> swerve.resetOdometry(new Pose2d())));

    // HANGER
    driver2
        .a()
        .onTrue(m_hangers.setPowerCommand(
            () -> -driver2.getRawAxis(Constants.kControls.TRANSLATION_Y_AXIS) * 4.0,
            () -> -driver2.getRawAxis(Constants.kControls.TRANSLATION_Y_AXIS) * 4.0));
    driver2.a().onFalse(m_hangers.setPowerCommand(() -> 0.0, () -> 0.0));

    driver2
        .y()
        .onTrue(m_hangers.setPowerCommand(
            () -> -driver2.getRawAxis(Constants.kControls.TRANSLATION_Y_AXIS) * 10.0,
            () -> -driver2.getRawAxis(Constants.kControls.TRANSLATION_Y_AXIS) * 10.0));
    driver2.y().onFalse(m_hangers.setPowerCommand(() -> 0.0, () -> 0.0));

    driver2.b().onTrue(new InstantCommand(() -> m_hangers.resetEncs()));

    driver2.x().onTrue(m_hangers.hangToTopStop());

    // ARM
    // driver.b().onTrue(m_arm.setPowerCommand(() -> Voltage.volts(2.0)
    //     .mul(-driver.getRawAxis(Constants.kControls.TRANSLATION_Y_AXIS))));
    // driver.b().onFalse(m_arm.emergencyStopCommand());

    // driver
    //     .rightBumper()
    //     .onTrue(m_arm.setPositionCommand(
    //         () -> Angle.degrees(SmartDashboard.getNumber("arm/set arm Pos", 0.0))));
    // driver.rightBumper().onFalse(m_arm.emergencyStopCommand());

    // test this

    // INTAKE AND SHOOT
    // driver.a().onTrue(intake.intakeOffCommand());
    // driver.rightBumper().onTrue(intakeCommands.amazingIntaking3(intake).andThen(new
    // WaitCommand(0.5)).andThen(intake.intakeOffCommand()));
    // driver.leftBumper().onTrue(intakeCommands.outakeNoteTime(intake));
    // driver.x().onTrue(m_shooter.shooterSpin().andThen(new WaitCommand(1.5))
    // .andThen(intake.intakeFastCommand()).andThen(new WaitCommand(2))
    // .andThen(m_shooter.shooterOffCommand()).andThen(intake.intakeOffCommand()));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    String autoName = "Auto1";
    Command resetOdometry = new InstantCommand(
        () -> swerve.resetOdometry(PathPlannerAuto.getStaringPoseFromAutoFile(autoName)));
    return resetOdometry.andThen(new PathPlannerAuto(autoName));
  }
}
