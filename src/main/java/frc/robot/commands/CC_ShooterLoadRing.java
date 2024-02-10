package frc.robot.commands;

import frc.robot.subsystems.swerve.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CC_ShooterLoadRing extends CommandBase {


    private ShooterSubsystem shooterSubsystem;
    
    public CC_ShooterLoadRing (ShooterSubsystem Shooter) {

        this.shooterSubsystem = Shooter;
        addRequirements(Shooter);
    }

    @Override
    public void execute() {


        // load ring
        shooterSubsystem.loadRing();

        

    }
}
