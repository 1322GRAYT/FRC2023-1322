package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class CC_ShootRing extends CommandBase {
    ShooterSubsystem _shooterSubsystem;
    public CC_ShootRing(ShooterSubsystem shooterSubsystem) {
        _shooterSubsystem = shooterSubsystem;
        addRequirements(_shooterSubsystem);
        // Use addRequirements() here to declare subsystem dependencies.
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        _shooterSubsystem.shoot();
        System.out.println("C_ShootRing executed.");
    }
    
}
