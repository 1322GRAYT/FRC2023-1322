package frc.robot;

import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Path;
import java.util.function.Consumer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;
import frc.robot.commandgroups.*;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TiltSubsystem;
import frc.robot.subsystems.swerve.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic 
 * should actually be handled in the {@link Robot} periodic methods 
 * (other than the scheduler calls). Instead, the structure of the robot 
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final SendableChooser<Command> _auto_chooser = new SendableChooser<Command>();
  private final SendableChooser<Command> _teleop_chooser = new SendableChooser<Command>();

  private final SwerveDrivetrain _swerveSubsystem = new SwerveDrivetrain();
  private final LiftSubsystem _liftSubsystem = new LiftSubsystem(Constants.LIFT_MOTOR);
  private final TiltSubsystem _tiltSubsystem = new TiltSubsystem(Constants.TILT_MOTOR);
  private final IntakeSubsystem _intakeSubsystem = new IntakeSubsystem(Constants.FLOOR_PICKUP_MOTOR);
  private final ShooterSubsystem _shooterSubsystem = new ShooterSubsystem(
          Constants.SHOOTER_MOTOR_0,
          Constants.SHOOTER_MOTOR_1, 
          Constants.SHOOTER_PRESHOOT_MOTOR, 
          Constants.SHOOTER_SENSOR0, 
          Constants.SHOOTER_SENSOR1,
          Constants.SHOOTER_SENSOR2
          );
  private final CameraSubsystem _cameraSubsystem = new CameraSubsystem();

  private XboxController _driverStick = new XboxController(Constants.DRIVER_CONTROLLER);
  private XboxController _auxStick = new XboxController(Constants.AUX_CONTROLLER);

  public RobotContainer() {

    ProfiledPIDController thetaController = Constants.Auton.THETA_CONTROLLER;
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

    _auto_chooser.setDefaultOption("Default Auto", new SequentialCommandGroup());
    _auto_chooser.addOption("Follow Trajectory", 
            new FollowTrajectoryAndStop("Unnamed.wpilib.json", _swerveSubsystem));
    
    /*new SwerveControllerCommand(
      getTrajectory("output/Unnamed.wpilib.json"),
      this::getPose, 
      _swerveSubsystem.getKinematics(),
      Constants.Auton.PX_CONTROLLER,
      Constants.Auton.PY_CONTROLLER,
      thetaController,
      this::setStates,
      new SwerveDrivetrain [] {_swerveSubsystem}
    )); */
    SmartDashboard.putData("Auto Choices: ", _auto_chooser);

    _teleop_chooser.setDefaultOption("Teleop - Robot Centric",
        new TeleopGroupCommand(_swerveSubsystem,
            _driverStick,
            _tiltSubsystem,
            _liftSubsystem,
            _shooterSubsystem,
            _intakeSubsystem,
            _auxStick,
            false,
            true));
    // teleop_chooser.setDefaultOption(
    _teleop_chooser.addOption("Teleop - Field Centric",
        new TeleopGroupCommand(_swerveSubsystem,
            _driverStick,
            _tiltSubsystem,
            _liftSubsystem,
            _shooterSubsystem,
            _intakeSubsystem,
            _auxStick,
            false,
            true));

    SmartDashboard.putData("Teleop Choices", _teleop_chooser);

    SmartDashboard.putData("Reset Robot Sensors to 0",
        new ResetRobotPositions(_swerveSubsystem, _liftSubsystem, _tiltSubsystem));
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

  }

  private void setDefaultCommands() {
    _swerveSubsystem.setDefaultCommand(new TeleopSwerveDrive(_driverStick, _swerveSubsystem, false, true));
  }

  public Command getTeleopCommand() {
    return _teleop_chooser.getSelected();
  }

  public Command getAutonomousCommand() {
    return _auto_chooser.getSelected();
  }
}