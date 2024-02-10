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
import frc.robot.subsystems.swerve.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import edu.wpi.first.wpilibj2.command.button.Trigger;


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
  private final SendableChooser<Command> _auto_chooser = new SendableChooser<Command>();
  private final SendableChooser<Command> _teleop_chooser = new SendableChooser<Command>();


  private final SwerveDrivetrain _swerveSubsystem = new SwerveDrivetrain();
  private final LiftSubsystem _liftSubsystem = new LiftSubsystem(Constants.LIFT_MOTOR);
  private final TiltSubsystem _tiltSubsystem = new TiltSubsystem(Constants.TILT_MOTOR);
  private final IntakeSubsystem _intakeSubsystem = new IntakeSubsystem(Constants.FLOOR_PICKUP);
  private final ShooterSubsystem _shooterSubsystem = new ShooterSubsystem(Constants.SHOOTER_MOTOR_0, Constants.SHOOTER_MOTOR_1, Constants.SHOOTER_PRESHOOT,Constants.SHOOTER_SENSOR0, Constants.SHOOTER_SENSOR1, Constants.SHOOTER_SENSOR2);
  private XboxController _driverStick = new XboxController(Constants.DRVR_CNTRLR);
  private XboxController _auxStick = new XboxController(Constants.AUX_CNTRLR);


  public RobotContainer() {

    _auto_chooser.setDefaultOption("Default Auto", new SequentialCommandGroup());
    SmartDashboard.putData("Auto Choices: ", _auto_chooser);

    _teleop_chooser.setDefaultOption("Teleop - Robot Centric", 
          new CG_Teleop(_swerveSubsystem, 
                        _driverStick,
                        _tiltSubsystem, 
                        _liftSubsystem, 
                        _shooterSubsystem,
                        _intakeSubsystem,
                        _auxStick  ,
                        false, 
                        true));
    //teleop_chooser.setDefaultOption(
    _teleop_chooser.addOption("Teleop - Field Centric", 
          new CG_Teleop(_swerveSubsystem, 
                        _driverStick,
                        _tiltSubsystem, 
                        _liftSubsystem, 
                        _shooterSubsystem,
                        _intakeSubsystem,
                        _auxStick  ,
                        false, 
                        true));
    
    
    SmartDashboard.putData("Teleop Choices", _teleop_chooser);
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
    _swerveSubsystem.init_periodic();
  }

  /**
   * Use this method to schedule any subsystem initialization tasks required to be
   * run
   * at the initialization of the Tele-Op Period.
   */
  public void teleopInit() {
    _swerveSubsystem.setDefaultCommand(getTeleopCommand());
    _swerveSubsystem.init_periodic();
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
    // BEGIN DRIVER STICK BUTTON ASSIGNMENTS
    final JoystickButton driverButton_BumpLT = new JoystickButton(_driverStick, Constants.BUMPER_LEFT);
   
 

  }

  private void setDefaultCommands() {
    _swerveSubsystem.setDefaultCommand(new CT_SwerveDrive(_swerveSubsystem, _driverStick, false, true));

  }


  public Command getTeleopCommand() {
    return _teleop_chooser.getSelected();
   }

  public Command getAutonomousCommand() {
    return _auto_chooser.getSelected();
  }
}