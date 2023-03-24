package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class CA_Balance extends CommandBase {
    SwerveDrivetrain drive;

    public CA_Balance(SwerveDrivetrain drive) {
        this.drive = drive;
    }

    double pitch = 0, roll = 0;
    double kP = 0.5;

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        pitch = drive.getPitchOptimized();
        roll = drive.getRollOptimized();
        if (-5 < roll && roll < 5) {
            roll = 0;
        }
        if (-5 < pitch && pitch < 5) {
            pitch = 0;
        }

        pitch = (pitch/90)*Constants.SwerveDrivetrain.MAX_SPEED * kP;
        roll =(roll/90)*Constants.SwerveDrivetrain.MAX_SPEED * kP;

        if ( ! (pitch == 0 && roll == 0) ) {
            drive.drive( new Translation2d(pitch,roll), 0, false, true);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drive.drive(new Translation2d(0,0), .5, false, false);
        drive.stopSwerveDriveMotors();
        drive.stopSwerveRotMotors();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}