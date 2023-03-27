package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.robot.Constants;
import frc.robot.Robot;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class SwerveModule {
    public int moduleNumber;

    public TalonFX driveMotor;
    private TalonFX angleMotor;
    private AnalogPotentiometer angleEncoder;

    private double lastAngle;

    public SwerveModulePosition geSwerveModulePosition() {
        return new SwerveModulePosition(getDrvEncdrCurrentPstn(), new Rotation2d(getDrvEncdrCurrentPstn()));
    }

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.SwerveDrivetrain.FF_kS,
            Constants.SwerveDrivetrain.FF_kV, Constants.SwerveDrivetrain.FF_kA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;

        /* Angle Encoder Config */
        angleEncoder = new AnalogPotentiometer(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Drive Motor Config */
        driveMotor = new TalonFX(moduleConstants.driveMotorID);
        configDriveMotor();

        /* Angle Motor Config */
        angleMotor = new TalonFX(moduleConstants.angleMotorID);
        configAngleMotor();

        resetDrvEncdrPstn();
        resetRotEncdrPstn();
        lastAngle = getState().angle.getDegrees();
        System.out.println("lastangle == "+lastAngle);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {

        desiredState = CTREModuleState.optimize(desiredState, getState().angle); // Custom optimize command, since
                                                                                 // default WPILib optimize assumes
                                                                                 // continuous controller which CTRE is
                                                                                 // not

        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveDrivetrain.MAX_SPEED;
            driveMotor.set(ControlMode.PercentOutput, percentOutput);
        } else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond,
                    Constants.SwerveDrivetrain.WHEEL_CIRCUMFERENCE, Constants.SwerveDrivetrain.DRIVE_GEAR_RATIO);
            driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward,
                    feedforward.calculate(desiredState.speedMetersPerSecond));
        }

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveDrivetrain.MAX_SPEED * 0.01))
                ? lastAngle
                : desiredState.angle.getDegrees(); // Prevent rotating module if speed is less then 1%. Prevents
                                                   // Jittering.
        angleMotor.set(ControlMode.Position,
                Conversions.degreesToFalcon(angle, Constants.SwerveDrivetrain.ANGLE_GEAR_RATIO));
        lastAngle = angle;

    }

    private void configAngleEncoder() {
        // TODO should this be commented or should this be uncommented? 
        //angleEncoder.configFactoryDefault();
        // System.out.println("HERE");
        //angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCANCoderConfig);
        // System.out.println("HERE 2");
    }

    private void configAngleMotor() {
        angleMotor.configFactoryDefault();
        angleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleTalonFXConfig);
        angleMotor.setInverted(Constants.SwerveDrivetrain.ANGLE_MOTOR_INVERTED);
        angleMotor.setNeutralMode(Constants.SwerveDrivetrain.ANGLE_NEUTRAL_MODE);
    }

    private void configDriveMotor() {
        driveMotor.configFactoryDefault();
        driveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveTalonFXConfig);
        driveMotor.setInverted(Constants.SwerveDrivetrain.DRIVE_MOTOR_INVERTED);
        driveMotor.setNeutralMode(Constants.SwerveDrivetrain.DRIVE_NEUTRAL_MODE);
        driveMotor.setSelectedSensorPosition(0);
    }

    public Rotation2d getCanCoder() {
        // Encoder Range: 0 - 72
        double raw = angleEncoder.get();
        double temp = raw * 360;
        SmartDashboard.putNumber("Encoder for Module " + moduleNumber + " Position: ", temp);
        //SmartDashboard.putNumber("Encoder for Module " + moduleNumber + " Raw Position: ", raw);
        return Rotation2d.fromDegrees(temp);
    }

    public SwerveModuleState getState() {
        double velocity = Conversions.falconToMPS(driveMotor.getSelectedSensorVelocity(),
                Constants.SwerveDrivetrain.WHEEL_CIRCUMFERENCE, Constants.SwerveDrivetrain.DRIVE_GEAR_RATIO);
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(
                angleMotor.getSelectedSensorPosition(), Constants.SwerveDrivetrain.ANGLE_GEAR_RATIO));
        return new SwerveModuleState(velocity, angle);
    }

    public void stopDrvMotor() {
        driveMotor.set(TalonFXControlMode.Disabled, 0);
    }

    public double getDrvEncdrCurrentPstn() {
        return driveMotor.getSelectedSensorPosition();
    }

    public void resetDrvEncdrPstn() {
        driveMotor.setSelectedSensorPosition(0);
    }

    public void stopRotMotor() {
        angleMotor.set(TalonFXControlMode.Disabled, 0);
    }

    public void resetRotEncdrPstn() {
        angleMotor.setSelectedSensorPosition(0);
    }

    public void zeroRotEncdrPstn() {
       /*
        double zeroPstnOffset = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset,
                Constants.SwerveDrivetrain.ANGLE_GEAR_RATIO);
        angleMotor.set(TalonFXControlMode.Position, zeroPstnOffset);
        */
    }

}
