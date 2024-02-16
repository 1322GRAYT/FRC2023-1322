package frc.robot.subsystems.swerve;

public class SwerveModuleConstants {
    
    public final int driveMotorID;
    public final int angleMotorID;

    public final int cancoderID;
    public final double angleOffset;
    public final int zeroSensor;

    public SwerveModuleConstants(int driveMotorID, int angleMotorID, int canCoderID, double angleOffset, int zeroSensor) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;

        this.cancoderID = canCoderID;
        this.angleOffset = angleOffset;
        this.zeroSensor = zeroSensor;
    }
}
