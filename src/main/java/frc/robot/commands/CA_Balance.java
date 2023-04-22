package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class CA_Balance extends CommandBase {
    private SwerveDrivetrain drive;
    private Gyro gyro;

    public CA_Balance(SwerveDrivetrain drive, Gyro gyro) {
        this.drive = drive;
        this.gyro = gyro;
    }

    double pitch = 0, roll = 0;
    double kP = 1;

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        drive.resetSwerveDriveEncoders();
        drive.resetSwerveRotateEncoders();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        pitch = gyro.getPitchOptimized();
        roll = gyro.getRollOptimized();

        pitch = (pitch/90);
        roll =(roll/90);
        System.out.println("pitch --" +pitch +"    roll --"+roll);



        Translation2d translation = new Translation2d(pitch, roll).times(Constants.SwerveDrivetrain.MAX_SPEED);
        System.out.println("AFTER    pitch --" +translation.getY() +"    roll --"+translation.getX());

            drive.drive( translation, 0, false, true);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //drive.zeroSwerveRotationMotors();
        drive.stopSwerveDriveMotors();
        drive.stopSwerveRotMotors();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}