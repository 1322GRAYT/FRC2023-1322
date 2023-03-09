// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;
import frc.robot.commandgroups.*;
import frc.robot.subsystems.*;
//import frc.robot.subsystems.ShooterSubsystem.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SendableChooser<Command> m_chooser = new SendableChooser<Command>();
 
  private final CompressorSub compressorSubsystem = new CompressorSub();
  private final SwerveDrivetrain swerveSubsystem = new SwerveDrivetrain();
  private final LiftElevator liftElevatorSubsystem = new LiftElevator();
  private final LiftClaw liftClawSubsystem = new LiftClaw();
  private final FloorPickup floorSubsystem = new FloorPickup();

  private final Camera cameraSubsystem = new Camera();
  private XboxController driverStick = new XboxController(Constants.DRVR_CNTRLR);
  private XboxController auxStick = new XboxController(Constants.AUX_CNTRLR);

  private Command m_autoCommandSelected;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure Autonomous Selections Available
     m_chooser.setDefaultOption("Default Auto", new CA_DriveDeadrecken(swerveSubsystem, -0.5, 2));

    SmartDashboard.putData("Auto choices: ", m_chooser);

    // Configure the button bindings
    configureButtonBindings();
    // Configure Default Commands
    setDefaultCommands();

  }

  /**
   * Use this method to schedule any subsystem initialization tasks required to be
   * run
   * at the initialization of the Autonomous Period.
   */
  public void autonomousInit() {
    swerveSubsystem.init_periodic();
  }

  /**
   * Use this method to schedule any subsystem initialization tasks required to be
   * run
   * at the initialization of the Tele-Op Period.
   */
  public void teleopInit() {
    swerveSubsystem.init_periodic();
    // need a new subsystem for new lift.
    floorSubsystem.init_periodic();
    // intakeSubsystem.init_periodic();
    // shooterSubsystem.init_periodic();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* BEGIN DRIVER STICK BUTTON ASSIGNMENTS */
    final JoystickButton driverButton_BumpLT = new JoystickButton(driverStick, Constants.BUMPER_LEFT);

    // driverButton_Start.onTrue(new CT_LiftRobot(liftSubsystem, auxStick));
    driverButton_BumpLT.onTrue(new CG_ResetAndZeroEncdrs(swerveSubsystem));

    /* BEGIN AUXILLARY STICK BUTTON ASSIGNMENTS */

    // need to update this
    //final Trigger rightTriggerButton = new Trigger( setArmPosition);
    final JoystickButton auxButton_A = new JoystickButton(auxStick, Constants.BUTTON_A);
    final JoystickButton auxButton_B = new JoystickButton(auxStick, Constants.BUTTON_B);
    final JoystickButton auxButton_X = new JoystickButton(auxStick, Constants.BUTTON_X);
    final JoystickButton auxButton_Y = new JoystickButton(auxStick, Constants.BUTTON_Y);
    final JoystickButton auxButton_BumpLT = new JoystickButton(auxStick, Constants.BUMPER_LEFT);
    final JoystickButton auxButton_BumpRT = new JoystickButton(auxStick, Constants.BUMPER_RIGHT);


  }

  private void setDefaultCommands() {
    // CommandScheduler.getInstance().setDefaultCommand(swerveSubsystem, new
    // ManualDrive(swerveSubsystem, driverStick));
    // Subsystem, Control Joystick, fieldCentric, openLoop
    swerveSubsystem.setDefaultCommand(new CT_SwerveDrive(swerveSubsystem, driverStick, true, true));
    // floorSubsystem.setDefaultCommand(new CT_Floor(floorSubsystem, auxStick));
    liftElevatorSubsystem.setDefaultCommand(new CT_LiftElevator(liftElevatorSubsystem, auxStick));
    liftClawSubsystem.setDefaultCommand(new CT_LiftCLaw(liftClawSubsystem, auxStick, driverStick));
    cameraSubsystem.setDefaultCommand(new CC_CameraTrackTarget(cameraSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    m_autoCommandSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoCommandSelected);
    return (m_autoCommandSelected);
  }
}