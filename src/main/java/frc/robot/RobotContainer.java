package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.library.LimelightHelpers;
import frc.robot.commands.IntakeCommands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Hangers;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.LEDlights;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.growingstems.measurements.Angle;

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

  private static final Field2d m_field = new Field2d();

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
    SmartDashboard.putData("Field", m_field);

    // auto = new AutoCommands(swerve);

    // Configure button bindings
    configureButtonBindings();
  }

  public void update() {
    var joystickPos = -driver.getRawAxis(Constants.kControls.TRANSLATION_Y_AXIS);
    SmartDashboard.putNumber("Joystick Pos", joystickPos);

    // Vision
    // var alliance = DriverStation.getAlliance();
    // if (alliance.isPresent()) {
    //   PoseEstimate llEstimate =
    //       switch (alliance.get()) {
    //         case Red -> LimelightHelpers.getBotPoseEstimate_wpiRed("");
    //         case Blue -> LimelightHelpers.getBotPoseEstimate_wpiBlue("");
    //       };

    //   swerve.updateVision(llEstimate);
    // }
    
    // LimelightHelpers.setCameraMode_Driver("limelight");
    // //LimelightHelpers.setCameraMode_Processor("limelight");
    // LimelightHelpers.setStreamMode_PiPMain("limelight");

    m_field.setRobotPose(swerve.getWpiPose());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    DoubleSupplier xDriveAxis = () -> -Constants.kControls.X_DRIVE_LIMITER.calculate(
        driver.getRawAxis(Constants.kControls.TRANSLATION_Y_AXIS));
    DoubleSupplier yDriveAxis = () -> -Constants.kControls.Y_DRIVE_LIMITER.calculate(
        driver.getRawAxis(Constants.kControls.TRANSLATION_X_AXIS));
    DoubleSupplier thetaDriveAxis = () -> -Constants.kControls.THETA_DRIVE_LIMITER.calculate(
        driver.getRawAxis(Constants.kControls.ROTATION_AXIS));

    swerve.setDefaultCommand(swerve.drive(xDriveAxis, yDriveAxis, thetaDriveAxis, true, true));

    driver.y().onTrue(new InstantCommand(() -> swerve.resetOdometry(new Pose2d())));

    // HANGER
    // pit controll -- good for reseting position
    driver2
        .a()
        .onTrue(m_hangers.setPowerCommand(
            () -> -driver2.getRawAxis(Constants.kControls.TRANSLATION_Y_AXIS) * 4.0,
            () -> -driver2.getRawAxis(Constants.kControls.TRANSLATION_Y_AXIS) * 4.0));
    driver2.a().onFalse(m_hangers.setPowerCommand(() -> 0.0, () -> 0.0));

    // send hangers to top of travel
    driver2.x().onTrue(m_hangers.hangToTopStop());

    // hangers down with hanging power
    driver2
        .y()
        .onTrue(m_hangers.setPowerCommand(
            () -> -driver2.getRawAxis(Constants.kControls.TRANSLATION_Y_AXIS) * 10.0,
            () -> -driver2.getRawAxis(Constants.kControls.TRANSLATION_Y_AXIS) * 10.0));
    driver2.y().onFalse(m_hangers.setPowerCommand(() -> 0.0, () -> 0.0));

    // easing up sides -- to level hang on non-level chain
    driver2.rightBumper().onTrue(m_hangers.setPowerCommand(() -> 0.0, () -> 1.0));
    driver2.rightBumper().onFalse(m_hangers.setPowerCommand(() -> 0.0, () -> 0.0));
    driver2.leftBumper().onTrue(m_hangers.setPowerCommand(() -> 1.0, () -> 0.0));
    driver2.leftBumper().onFalse(m_hangers.setPowerCommand(() -> 0.0, () -> 0.0));

    // reset hangers encoders
    driver2.b().onTrue(new InstantCommand(() -> m_hangers.resetEncs()));

    // small intake control for co-driver
    driver2.pov(0).onTrue(intake.intakeSlowCommand());
    driver2.pov(0).onFalse(intake.intakeOffCommand());

    driver2.pov(180).onTrue(intake.outtakeCommand());
    driver2.pov(180).onFalse(intake.intakeOffCommand());

    // ARM Manual Control
    // driver.b().onTrue(m_arm.setPowerCommand(() -> Voltage.volts(2.0)
    //     .mul(-driver.getRawAxis(Constants.kControls.TRANSLATION_Y_AXIS))));
    // // driver.b().onFalse(m_arm.emergencyStopCommand());

    // driver.a().onTrue(m_arm.setGravCommand(() -> Voltage.volts(.03)));
    // driver.a().onFalse(m_arm.emergencyStopCommand());

    // Arm Auto Control
    Supplier<Command> shuffleboardPos = () ->
        m_arm.setPositionCommand(Angle.degrees(SmartDashboard.getNumber("arm/set arm Pos", 0.0)));
    driver.a().onTrue(new ProxyCommand(shuffleboardPos));
    driver.b().onTrue(m_arm.setPositionCommand(Angle.degrees(100.0)));
    driver.rightBumper().onTrue(m_arm.setPositionCommand(Angle.degrees(-2.0)));
    driver.leftTrigger(0.5).onTrue(m_arm.setPositionCommand(Angle.degrees(15)));

    // test this

    // INTAKE AND SHOOT
    driver.a().onTrue(intake.intakeOffCommand());
    driver
        .rightBumper()
        .onTrue(intakeCommands
            .amazingIntaking3(intake)
            .andThen(new WaitCommand(0.5))
            .andThen(intake.intakeOffCommand()));
    driver.leftBumper().onTrue(intakeCommands.outakeNoteTime(intake));
    driver
        .x()//.and(() -> !m_arm.isAmpPos())
        .onTrue(m_shooter
            .shooterSpinSpeaker()
            .andThen(() -> LEDlights.shootColor())
            .andThen(new WaitCommand(1.5))
            .andThen(intake.intakeFastCommand())
            .andThen(new WaitCommand(2))
            .andThen(m_shooter.shooterOffCommand())
            .andThen(intake.intakeOffCommand())
            .andThen(() -> LEDlights.normalColor()));
    // driver
    //     .x().and(() -> m_arm.isAmpPos())
    //     .onTrue(m_shooter
    //         .shooterSpinAmp()
    //         .andThen(() -> LEDlights.shootColor())
    //         .andThen(new WaitCommand(1.5))
    //         .andThen(intake.intakeFastCommand())
    //         .andThen(new WaitCommand(2))
    //         .andThen(m_shooter.shooterOffCommand())
    //         .andThen(intake.intakeOffCommand())
    //         .andThen(() -> LEDlights.normalColor()));

    // var blueTarget = new Vector2dU<Length>(Length.ZERO, Length.ZERO);
    // Supplier<Angle> aimAngle =
    //     () -> blueTarget.sub(swerve.getPose().getVector()).getAngle();
    // var aimCommand = swerve.drive(xDriveAxis, yDriveAxis, aimAngle, true, true);

    // driver.leftTrigger().onTrue(aimCommand);
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
