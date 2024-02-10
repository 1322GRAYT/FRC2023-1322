package frc.robot.commands;


import frc.robot.subsystems.swerve.TiltSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CC_TiltLoadPos extends CommandBase {
    private TiltSubsystem tiltSubsystem;

    
    public CC_TiltLoadPos (TiltSubsystem tilt) {
        this.tiltSubsystem = tilt;
        addRequirements(tilt);
    }

    @Override
    public void execute() {
        // tilt to intake position
        tiltSubsystem.loadPosition();
    }
}
