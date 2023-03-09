package frc.robot.utils.swerve;

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
    private double angleOffset;

    private double lastAngle;

    public SwerveModulePosition geSwerveModulePosition() {
        return new SwerveModulePosition(getDrvEncdrCurrentPstn(), new Rotation2d(getDrvEncdrCurrentPstn()));
    }

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.SwerveDrivetrain.FF_kS,
            Constants.SwerveDrivetrain.FF_kV, Constants.SwerveDrivetrain.FF_kA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;

        this.angleOffset = moduleConstants.angleOffset;

        /* Angle Encoder Config */
        this.angleEncoder = new AnalogPotentiometer(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Drive Motor Config */
        this.driveMotor = new TalonFX(moduleConstants.driveMotorID);
        configDriveMotor();

        /* Angle Motor Config */
        this.angleMotor = new TalonFX(moduleConstants.angleMotorID);
        configAngleMotor();

        this.lastAngle = getState().angle.getDegrees();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {

        desiredState = CTREModuleState.optimize(desiredState, getState().angle); // Custom optimize command, since
                                                                                 // default WPILib optimize assumes
                                                                                 // continuous controller which CTRE is
                                                                                 // not

        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveDrivetrain.MAX_SPEED;
            this.driveMotor.set(ControlMode.PercentOutput, percentOutput);
        } else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond,
                    Constants.SwerveDrivetrain.WHEEL_CIRCUMFERENCE, Constants.SwerveDrivetrain.DRIVE_GEAR_RATIO);
            this.driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward,
                    feedforward.calculate(desiredState.speedMetersPerSecond));
        }

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveDrivetrain.MAX_SPEED * 0.01))
                ? lastAngle
                : desiredState.angle.getDegrees(); // Prevent rotating module if speed is less then 1%. Prevents
                                                   // Jittering.
        this.angleMotor.set(ControlMode.Position,
                Conversions.degreesToFalcon(angle, Constants.SwerveDrivetrain.ANGLE_GEAR_RATIO));
        this.lastAngle = angle;

    }

    public void resetToAbsolute() {
        double absolutePosition = Conversions.degreesToFalcon(this.getCanCoder().getDegrees() - angleOffset,
                Constants.SwerveDrivetrain.ANGLE_GEAR_RATIO);
        this.angleMotor.set(TalonFXControlMode.Position, absolutePosition);
    }

    private void configAngleEncoder() {
        // this.angleEncoder.configFactoryDefault();
        // System.out.println("HERE");
        // this.angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCANCoderConfig);
        // System.out.println("HERE 2");
    }

    private void configAngleMotor() {
        this.angleMotor.configFactoryDefault();
        this.angleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleTalonFXConfig);
        this.angleMotor.setInverted(Constants.SwerveDrivetrain.ANGLE_MOTOR_INVERTED);
        this.angleMotor.setNeutralMode(Constants.SwerveDrivetrain.ANGLE_NEUTRAL_MODE);
        resetToAbsolute();
    }

    private void configDriveMotor() {
        this.driveMotor.configFactoryDefault();
        this.driveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveTalonFXConfig);
        this.driveMotor.setInverted(Constants.SwerveDrivetrain.DRIVE_MOTOR_INVERTED);
        this.driveMotor.setNeutralMode(Constants.SwerveDrivetrain.DRIVE_NEUTRAL_MODE);
        this.driveMotor.setSelectedSensorPosition(0);
    }

    public Rotation2d getCanCoder() {
        // Encoder Range: 0 - 72
        double raw = angleEncoder.get();
        double temp = raw * 360;
        SmartDashboard.putNumber("Encoder for Module " + this.moduleNumber + " Position: ", temp);
        SmartDashboard.putNumber("Encoder for Module " + this.moduleNumber + " Raw Position: ", raw);
        return Rotation2d.fromDegrees(temp);
    }

    public SwerveModuleState getState() {
        double velocity = Conversions.falconToMPS(this.driveMotor.getSelectedSensorVelocity(),
                Constants.SwerveDrivetrain.WHEEL_CIRCUMFERENCE, Constants.SwerveDrivetrain.DRIVE_GEAR_RATIO);
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(
                this.angleMotor.getSelectedSensorPosition(), Constants.SwerveDrivetrain.ANGLE_GEAR_RATIO));
        return new SwerveModuleState(velocity, angle);
    }

    public void stopDrvMotor() {
        this.driveMotor.set(TalonFXControlMode.Disabled, 0);
    }

    public double getDrvEncdrCurrentPstn() {
        double encdrCnts = this.driveMotor.getSelectedSensorPosition();
        return encdrCnts;
    }

    public void resetDrvEncdrPstn() {
        this.driveMotor.setSelectedSensorPosition(0);
    }

    public void stopRotMotor() {
        this.angleMotor.set(TalonFXControlMode.Disabled, 0);
    }

    public void resetRotEncdrPstn() {
        this.angleMotor.setSelectedSensorPosition(0);
    }

    public void zeroRotEncdrPstn() {
        double zeroPstnOffset = Conversions.degreesToFalcon(this.getCanCoder().getDegrees() - angleOffset,
                Constants.SwerveDrivetrain.ANGLE_GEAR_RATIO);
        this.angleMotor.set(TalonFXControlMode.Position, zeroPstnOffset);
    }

}
