package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class CC_IntakeOn extends CommandBase {


    private IntakeSubsystem intakeSubsystem;
 
    
    public CC_IntakeOn (IntakeSubsystem Intake) {
        this.intakeSubsystem = Intake;
        addRequirements(Intake);
    }

    @Override
    public void execute() {
        intakeSubsystem.intakeOn();
    }
}
