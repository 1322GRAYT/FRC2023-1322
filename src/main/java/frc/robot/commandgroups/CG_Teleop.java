package frc.robot.commandgroups;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.CT_LiftTiltShooter;
import frc.robot.commands.CT_SwerveDrive;
import frc.robot.subsystems.swerve.IntakeSubsystem;
import frc.robot.subsystems.swerve.LiftSubsystem;
import frc.robot.subsystems.swerve.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.swerve.TiltSubsystem;

public class CG_Teleop extends ParallelCommandGroup {

public CG_Teleop(
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
        new CT_SwerveDrive(swerveDrivetrain, mainStick, fieldCentric, openLoop),
        new CT_LiftTiltShooter(auxStick, tilt, lift,intake, shooter)
    );
}
}