package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TiltSubsystem;

public class TiltZeroPosition extends CommandBase{
    private TiltSubsystem tiltSubsystem;

    public TiltZeroPosition(TiltSubsystem tiltSubsystem) {
        this.tiltSubsystem = tiltSubsystem;

        addRequirements(tiltSubsystem);
    }

    @Override
    public void initialize() {
        tiltSubsystem.zeroTiltEncoder();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
