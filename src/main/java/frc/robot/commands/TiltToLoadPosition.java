package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TiltSubsystem;

public class TiltToLoadPosition extends CommandBase {
    private TiltSubsystem tiltSubsystem;

    
    public TiltToLoadPosition (TiltSubsystem tilt) {
        this.tiltSubsystem = tilt;
        addRequirements(tilt);
    }

    @Override
    public void execute() {
        // tilt to intake position
        tiltSubsystem.loadPosition();
    }
}
