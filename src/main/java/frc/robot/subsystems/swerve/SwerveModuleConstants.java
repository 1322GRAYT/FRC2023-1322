package frc.robot.subsystems.swerve;

public class SwerveModuleConstants {
    
    public final int driveMotorID;
    public final int angleMotorID;

    public final int cancoderID;
    //public final double angleOffset;

    public SwerveModuleConstants(int driveMotorID, int angleMotorID, int canCoderID) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;

        this.cancoderID = canCoderID;
        //this.angleOffset = angleOffset;
    }

}
