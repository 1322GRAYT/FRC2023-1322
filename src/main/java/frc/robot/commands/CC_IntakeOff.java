package frc.robot.commands;

import frc.robot.subsystems.swerve.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CC_IntakeOff extends CommandBase {


    private IntakeSubsystem intakeSubsystem;
 
    
    public CC_IntakeOff (IntakeSubsystem Intake) {
        this.intakeSubsystem = Intake;
        addRequirements(Intake);
    }

    @Override
    public void execute() {
        intakeSubsystem.intakeOff();
    }
}
