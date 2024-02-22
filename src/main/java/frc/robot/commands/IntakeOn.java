package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeOn extends CommandBase {


    private IntakeSubsystem intakeSubsystem;
 
    
    public IntakeOn (IntakeSubsystem Intake) {
        this.intakeSubsystem = Intake;
        addRequirements(Intake);
    }

    @Override
    public void execute() {
        intakeSubsystem.intakeOn();
    }
}
