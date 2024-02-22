package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LiftSubsystem;

public class LiftToLoadPosition extends CommandBase {


    private LiftSubsystem liftSubsystem;

    
    public LiftToLoadPosition (LiftSubsystem lift) {
        this.liftSubsystem = lift;
        addRequirements(lift);
    }

    @Override
    public void execute() {
        // tilt to intake position
        liftSubsystem.loadPosition();
    }
}
