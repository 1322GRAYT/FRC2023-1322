package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TiltSubsystem;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopLiftTileShooter extends CommandBase {

    private static final double DEADBAND = 0.15;

    private TiltSubsystem _tiltSubsystem;
    private LiftSubsystem _liftSubsystem;
    private ShooterSubsystem _shooterSubsystem;
    private IntakeSubsystem _intakeSubsystem;

    private XboxController controller;

    public TeleopLiftTileShooter(XboxController controller,
            TiltSubsystem tiltSubsystem,
            LiftSubsystem liftSubsystem,
            IntakeSubsystem intakeSubsystem,
            ShooterSubsystem shooterSubsystem) {
        this._tiltSubsystem = tiltSubsystem;
        this._liftSubsystem = liftSubsystem;
        this._shooterSubsystem = shooterSubsystem;
        this._intakeSubsystem = intakeSubsystem;

        addRequirements(tiltSubsystem, liftSubsystem, shooterSubsystem, intakeSubsystem);

        this.controller = controller;
        this.abutton = new Debouncer(0.3, DebounceType.kBoth);
    }

    Debouncer abutton;

    @Override
    public void execute() {

        double liftAxis = -controller.getLeftY();
        double tiltAxis = -controller.getRightY();
        double shooterAxisRight = controller.getRightTriggerAxis();
        // double shooterAxisLeft = controller.getLeftTriggerAxis();
        controller.getLeftBumper();

        /* Deadbands */
        // liftAxis = Constants.ApplyDeadBand_Scaled(liftAxis, DEADBAND, 1.0);
        tiltAxis = Constants.ApplyDeadBand_Scaled(tiltAxis, DEADBAND, 1.0);

        if (controller.getLeftBumper())
            _intakeSubsystem.move(1.0);
        else if (controller.getRightBumper())
            _intakeSubsystem.move(-1.0);
        else
            _intakeSubsystem.move(0.0);

        _liftSubsystem.move(liftAxis);
        _tiltSubsystem.move(tiltAxis);
        if (shooterAxisRight > 0.15) {
            _shooterSubsystem.shoot();
        }
        if (controller.getRightBumper()) {
            _shooterSubsystem.drop();
        }
        // _shooterSubsystem.preShootMove(-shooterAxisLeft);

        if (_shooterSubsystem.loaded() == false) {
            if (controller.getLeftTriggerAxis() > 0.15) {
                _shooterSubsystem.setLoading();
                _intakeSubsystem.intake(0.7);
                _shooterSubsystem.preShootMove(-.2);
            } else {
                _shooterSubsystem.unSetLoading();
                _intakeSubsystem.intake(0.0);
            }
        }
    }
}
