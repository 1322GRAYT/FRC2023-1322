package frc.robot.commands;

import frc.robot.subsystems.swerve.LiftSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CC_LiftLoadPos extends CommandBase {


    private LiftSubsystem liftSubsystem;

    
    public CC_LiftLoadPos (LiftSubsystem lift) {
        this.liftSubsystem = lift;
        addRequirements(lift);
    }

    @Override
    public void execute() {
        // tilt to intake position
        liftSubsystem.loadPosition();
    }
}
