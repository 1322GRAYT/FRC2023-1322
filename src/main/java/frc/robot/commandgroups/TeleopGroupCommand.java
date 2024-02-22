package frc.robot.commandgroups;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.TeleopLiftTileShooter;
import frc.robot.commands.TeleopSwerveDrive;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TiltSubsystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class TeleopGroupCommand extends ParallelCommandGroup {

public TeleopGroupCommand(
    SwerveDrivetrain swerveDrivetrain, 
    XboxController mainStick, 
    TiltSubsystem tilt,
    LiftSubsystem lift,
    ShooterSubsystem shooter,
    IntakeSubsystem intake,
    XboxController auxStick,
    boolean fieldCentric,
    boolean openLoop
    ) {
    addCommands(
        new TeleopSwerveDrive(mainStick, swerveDrivetrain, fieldCentric, openLoop),
        new TeleopLiftTileShooter(auxStick, tilt, lift,intake, shooter)
    );
}
}