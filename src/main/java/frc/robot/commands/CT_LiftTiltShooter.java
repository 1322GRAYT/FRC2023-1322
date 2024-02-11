package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TiltSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CT_LiftTiltShooter extends CommandBase {

    private static final double DEADBAND = 0.15;


    private TiltSubsystem _tiltSubsystem;
    private LiftSubsystem _liftSubsystem;
    private ShooterSubsystem _shooterSubsystem;
    private IntakeSubsystem _intakeSubsystem;

    private XboxController controller;
    
    public CT_LiftTiltShooter (XboxController controller, 
    TiltSubsystem tiltSubsystem, 
    LiftSubsystem liftSubsystem, 
    IntakeSubsystem intakeSubsystem,
    ShooterSubsystem shooterSubsystem) {
        this._tiltSubsystem = tiltSubsystem;
        this._liftSubsystem = liftSubsystem;
        this._shooterSubsystem = shooterSubsystem;
        this._intakeSubsystem = intakeSubsystem;
        

        addRequirements(tiltSubsystem, liftSubsystem, shooterSubsystem,intakeSubsystem);
        
        this.controller = controller;
    }

    @Override
    public void execute() {

        double liftAxis = -controller.getLeftY();
        double tiltAxis = -controller.getRightY();
        double shooterAxisRight = controller.getRightTriggerAxis();
        double shooterAxisLeft = controller.getLeftTriggerAxis();
        

        /* Deadbands */
        //liftAxis = Constants.ApplyDeadBand_Scaled(liftAxis, DEADBAND, 1.0);
        tiltAxis = Constants.ApplyDeadBand_Scaled(tiltAxis, DEADBAND, 1.0);
        
        if (controller.getLeftBumper()) _intakeSubsystem.move(1.0);
        else if (controller.getRightBumper()) _intakeSubsystem.move( -1.0);
        else _intakeSubsystem.move(0.0);
        
        _liftSubsystem.move(liftAxis);
        _tiltSubsystem.move(tiltAxis);
        _shooterSubsystem.move(shooterAxisRight);
        _shooterSubsystem.preShootMove(shooterAxisLeft);
        
    }
}
