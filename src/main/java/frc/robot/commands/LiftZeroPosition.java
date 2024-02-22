package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LiftSubsystem;

public class LiftZeroPosition extends CommandBase{
    private LiftSubsystem liftSubsystem;

    public LiftZeroPosition(LiftSubsystem liftSubsystem) {
        this.liftSubsystem = liftSubsystem;

        addRequirements(liftSubsystem);
    }

    @Override
    public void initialize() {
        liftSubsystem.zeroLiftEncoder();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
