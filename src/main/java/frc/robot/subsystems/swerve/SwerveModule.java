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
    public   int moduleNumber;

    private  TalonFX driveMotor;
    private TalonFX angleMotor;
    private AnalogPotentiometer angleEncoder;

    private double lastAngle;

    private double _zeroPosition;

    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(getDriveEncoderCurrentPosition(), new Rotation2d(getDriveEncoderCurrentPosition()));
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

        resetDriveEncoderPosition();
        resetRotaeEncoderPosition();
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
                angle * Constants.SwerveDrivetrain.ANGLE_DEGREES_TO_FALCON);
        lastAngle = angle;

    }

    private void configAngleEncoder() {
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
        double degrees = angleEncoder.get() * 360;
        SmartDashboard.putNumber("Encoder for Module " + moduleNumber + " Position: ", degrees);
        return Rotation2d.fromDegrees(degrees);
    }

    public SwerveModuleState getState() {
        double velocity = Conversions.falconToMPS(driveMotor.getSelectedSensorVelocity(),
                Constants.SwerveDrivetrain.WHEEL_CIRCUMFERENCE, Constants.SwerveDrivetrain.DRIVE_GEAR_RATIO);
        Rotation2d angle = Rotation2d.fromDegrees(
                angleMotor.getSelectedSensorPosition() * Constants.SwerveDrivetrain.ANGLE_FALCON_TO_DEGREES);
        return new SwerveModuleState(velocity, angle);
    }

    public void stopDriveMotor() {
        driveMotor.set(TalonFXControlMode.Disabled, 0);
    }

    public double getDriveEncoderCurrentPosition() {
        return driveMotor.getSelectedSensorPosition();
    }

    public void resetDriveEncoderPosition() {
        driveMotor.setSelectedSensorPosition(0);
    }

    public void stopRotateMotor() {
        angleMotor.set(TalonFXControlMode.Disabled, 0);
    }

    public void resetRotaeEncoderPosition() {
        angleMotor.setSelectedSensorPosition(0);
    }

    public void zeroRotateEncoderPosition() {
        double start=0.0,end=0.0;

        angleMotor.set(ControlMode.Position, 0);
        do {
            if (angleMotor.getSensorCollection().isRevLimitSwitchClosed() == 1) {
                start = angleMotor.getSelectedSensorPosition();
                break;
            }
        } while (angleMotor.getSensorCollection().isRevLimitSwitchClosed() == 0);

        do {
            if (angleMotor.getSensorCollection().isRevLimitSwitchClosed() == 1) {
                end = angleMotor.getSelectedSensorPosition();
                break;
            }
        } while (angleMotor.getSensorCollection().isRevLimitSwitchClosed() == 1);

        _zeroPosition = (Math.abs(start - end) / 2);

        /*
        while (! angleMotor.getSensorCollection().isRevLimitSwitchClosed()) {
            angleMotor.set(ControlMode.PercentOutput, 0.1);
        }
        angleMotor.set(ControlMode.PercentOutput, 0);
        double start  = angleMotor.getSelectedSensorPosition();
        while (angleMotor.getSensorCollection().isRevLimitSwitchClosed()) {
            angleMotor.set(ControlMode.PercentOutput, 0.1);
        }
        angleMotor.set(ControlMode.PercentOutput, 0);
        double finish = angleMotor.getSelectedSensorPosition();

        _zero_position = (Math.abs(start - finish) / 2);
        angleMotor.set(ControlMode.RuntoPosition,_zero_position);
        */
        }
    }
