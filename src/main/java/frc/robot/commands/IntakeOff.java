package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeOff extends CommandBase {


    private IntakeSubsystem intakeSubsystem;
 
    
    public IntakeOff (IntakeSubsystem Intake) {
        this.intakeSubsystem = Intake;
        addRequirements(Intake);
    }

    @Override
    public void execute() {
        intakeSubsystem.intakeOff();
    }
}
