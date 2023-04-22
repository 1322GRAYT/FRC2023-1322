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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.Direction;

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
  private final SendableChooser<Command> m_chooser;

  private final Gyro gyroSubsystem;
  private final SwerveDrivetrain swerveSubsystem;
  private final LiftElevator liftElevatorSubsystem;
  private final Lift liftClawSubsystem;
  private final Claw clawSubsystem;

  private final Camera cameraSubsystem;
  private final XboxController driverStick;
  private final XboxController auxStick;

  private Trigger auxPOVup;
  private Trigger auxPOVdown;
  private Trigger auxPOVleft;
  private Trigger auxPOVright;

  private JoystickButton auxAButton;
  private JoystickButton auxYButton;
  // private JoystickButton auxBButton;

  private Command m_autoCommandSelected;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    m_chooser = new SendableChooser<Command>();

    gyroSubsystem = new Gyro(Constants.GYRO_ID);
    swerveSubsystem = new SwerveDrivetrain(gyroSubsystem);
    liftElevatorSubsystem = new LiftElevator();
    liftClawSubsystem = new Lift();
    clawSubsystem = new Claw();

    cameraSubsystem = new Camera();
    driverStick = new XboxController(Constants.DRVR_CNTRLR);
    auxStick = new XboxController(Constants.AUX_CNTRLR);

    // no var needed as it is a singleton instance.
    CompressorSub.getInstance();
    // Configure Autonomous Selections Available
    // m_chooser.setDefaultOption("Default Auto", new
    // CG_PlaceCone(liftClawSubsystem, liftElevatorSubsystem));

    // m_chooser.setDefaultOption("Default Auto", new
    // CA_DriveDeadrecken(swerveSubsystem, -0.5, 2));

    // compressorSubsystem.getCompressorPressure(); // this gets rid of the

    m_chooser.setDefaultOption("PlaceAndBack", new CG_DriveBack(swerveSubsystem, clawSubsystem, liftElevatorSubsystem));
    m_chooser.addOption("Ramp Balance",
        new BalanceOnRamp(swerveSubsystem, gyroSubsystem, clawSubsystem, liftElevatorSubsystem));
    m_chooser.addOption("Yeet Cube, then Ramp",
        new CG_YeetCubeRamp(swerveSubsystem, gyroSubsystem, clawSubsystem, liftElevatorSubsystem));

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
    final JoystickButton driverButton_BumpLT = new JoystickButton(driverStick, Constants.BUMPER_LEFT);
    driverButton_BumpLT.onTrue(new CG_ResetAndZeroEncdrs(swerveSubsystem, gyroSubsystem));

    auxPOVup = new Trigger(() -> auxStick.getPOV() == Direction.UP.getValue());
    auxPOVdown = new Trigger(() -> auxStick.getPOV() == Direction.DOWN.getValue());
    auxPOVleft = new Trigger(() -> auxStick.getPOV() == Direction.LEFT.getValue());
    auxPOVright = new Trigger(() -> auxStick.getPOV() == Direction.RIGHT.getValue());

    auxAButton = new JoystickButton(auxStick, Constants.BUTTON_A);
    auxYButton = new JoystickButton(auxStick, Constants.BUTTON_Y);
    // auxBButton = new JoystickButton(auxStick, Constants.BUTTON_B);

    auxYButton.whileTrue(new RunCommand(() -> {
      clawSubsystem.setIntakeMotorPower(1.0);
    }, clawSubsystem));
    auxAButton.whileTrue(new RunCommand(() -> {
      clawSubsystem.setIntakeMotorPower(-1.0);
    }, clawSubsystem));
  }

  private void setDefaultCommands() {
    swerveSubsystem.setDefaultCommand(new CT_SwerveDrive(swerveSubsystem, driverStick, false, true));
    liftElevatorSubsystem.setDefaultCommand(new CT_LiftElevator(liftElevatorSubsystem,
        () -> auxStick.getLeftY(),
        () -> (auxStick.getLeftTriggerAxis() - auxStick.getRightTriggerAxis())));
    liftClawSubsystem.setDefaultCommand(new CT_LiftCLaw(liftClawSubsystem,
        clawSubsystem,
        () -> auxStick.getRightY(),
        () -> auxStick.getRightX(),
        () -> auxStick.getBButtonPressed()));
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