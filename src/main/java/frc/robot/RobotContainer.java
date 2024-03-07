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
import frc.robot.subsystems.Swerve;

import org.growingstems.measurements.Angle;
import org.growingstems.measurements.Measurements.Voltage;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public final CommandXboxController driver;

  public final Swerve swerve;

  public final Intake intake;

  public final IntakeCommands intakeCommands;

  public final Hangers m_hangers = new Hangers();

  public final Arm m_arm = new Arm();

  // public final AutoCommands auto;

  public RobotContainer() {
    driver = new CommandXboxController(Constants.kControls.DRIVE_JOYSTICK_ID);

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

    // driver.x().onTrue(intake.intakeSlowCommand());
    // driver.b().onTrue(intake.intakeMediumCommand());
    // driver.a().onTrue(intake.intakeFastCommand());

    driver.x().onTrue(intakeCommands.intakeNoteTime(intake));
    driver.leftBumper().onTrue(intakeCommands.outakeNoteTime(intake));

    driver
        .a()
        .onTrue(m_hangers.setPowerCommand(
            () -> -driver.getRawAxis(Constants.kControls.TRANSLATION_Y_AXIS) * 12));
    driver.a().onFalse(m_hangers.setPowerCommand(() -> 0.0));

    driver.b().onTrue(m_arm.setPowerCommand(() -> Voltage.volts(2.0)
        .mul(-driver.getRawAxis(Constants.kControls.TRANSLATION_Y_AXIS))));
    driver.b().onFalse(m_arm.emergencyStopCommand());

    driver.rightBumper().onTrue(m_arm.setPositionCommand(() -> Angle.degrees(SmartDashboard.getNumber("set arm Pos", 0.0))));
    driver.rightBumper().onFalse(m_arm.emergencyStopCommand());

    // test this

    // driver.b().onTrue(intakeCommands.amazingIntaking(intake));
    // and this
    // driver.rightBumper().onTrue(intakeCommands.amazingIntaking3(intake));
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
