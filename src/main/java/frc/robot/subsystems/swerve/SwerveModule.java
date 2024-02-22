package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

public class SwerveModule {
    public int _moduleNumber;

    private TalonFX _driveMotor;
    private TalonFX _angleMotor;
    private AnalogPotentiometer _angleEncoder;
    private double _angleOffset;

    private DigitalInput _zeroSensor;
   

    private double _lastAngle;

    SimpleMotorFeedforward _feedforward = new SimpleMotorFeedforward(
        Constants.SwerveDrivetrain.FeedForwardStaticGain,
        Constants.SwerveDrivetrain.FeedForwardVelocityGain,
        Constants.SwerveDrivetrain.FeedForwardAccerationGain
        );

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDriveEncoderPosition(), new Rotation2d(getRotateEncoderPosition()));
    }

    public SwerveModule(
        int moduleNumber,
        double angleOffset,
        int cancoderID,
        int driveMotorID,
        int angleMotorID,
        int zeroSensor
        ) 
    {
        this.init(moduleNumber,angleOffset,cancoderID,driveMotorID,angleMotorID, zeroSensor);
    }

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.init(
            moduleNumber,
            moduleConstants.angleOffset,
            moduleConstants.cancoderID,
            moduleConstants.driveMotorID,
            moduleConstants.angleMotorID,
            moduleConstants.zeroSensor
            );
    }


    private void init(
        int moduleNumber,
        double angleOffset,
        int cancoderID,
        int driveMotorID,
        int angleMotorID,
        int zeroSensor
        ) 
    {
        System.out.println("Starting Module init - "+moduleNumber);
        this._moduleNumber = moduleNumber;

        this._angleOffset = angleOffset;

        /* Angle Encoder Config */
        this._angleEncoder = new AnalogPotentiometer(cancoderID);
        //configAngleEncoder();

        /* Drive Motor Config */
        this._driveMotor = new TalonFX(driveMotorID);
        configDriveMotor();

        /* Angle Motor Config */
        this._angleMotor = new TalonFX(angleMotorID);
        configAngleMotor();

        resetDriveEncoderPosition();
        
        this._zeroSensor = new DigitalInput(zeroSensor);

        this._lastAngle = getState().angle.getDegrees();
        System.out.println("lastangle "+moduleNumber+" == "+_lastAngle);
    }


    public  void findZeroRotation() {
        System.out.println("Starting find zero");
        // for all 
        // spin wheels in turn until they find the zero

        boolean start_found = false;
        boolean end_found = false;
        double start=-1, end=-1;

        _angleMotor.set(ControlMode.PercentOutput,Constants.SwerveDrivetrain.SWERVE_ZERO_SPEED );
        while (!end_found) {
            if (!_zeroSensor.get()) {
                start_found=true;
                start = getRotateEncoderPosition();
                continue;
            }
            if (start_found) {
                if (_zeroSensor.get()) {
                    end_found = true;
                    end = getDriveEncoderPosition();
                }
            }
        }
        _angleMotor.set(ControlMode.PercentOutput, 0.0);
        double mid = Math.abs((start-end)/2);
        double pos = (start-end>0)?start+mid:end+mid;
        System.out.println("Module - "+_moduleNumber+" Pos:("+pos+") Start("+start+") End("+end+") Mid("+mid+")");
        _angleMotor.set(ControlMode.Position, pos);
        resetRotateEncoderPosition();
    }
    public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
        double targetSpeed = desiredState.speedMetersPerSecond;
        double delta = targetAngle - currentAngle.getDegrees();
        if (Math.abs(delta) > 90) {
            targetSpeed = -targetSpeed;
            targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
        }
        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }

    /**
     * @param scopeReference Current Angle
     * @param newAngle       Target Angle
     * @return Closest angle within scope
     */
    private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        lowerBound = scopeReference - lowerOffset;
        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }
        while (newAngle < lowerBound) {
            newAngle += 360;
        }
        while (newAngle > upperBound) {
            newAngle -= 360;
        }
        if (newAngle - scopeReference > 180) {
            newAngle -= 360;
        } else if (newAngle - scopeReference < -180) {
            newAngle += 360;
        }
        return newAngle;
    }


    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        // Custom optimize command, since default WPILib optimize assumes continuous controller which CTRE is not
        desiredState = optimize(desiredState, getState().angle); 
        if (isOpenLoop) {
            this._driveMotor.set(
                ControlMode.PercentOutput, 
                desiredState.speedMetersPerSecond / Constants.SwerveDrivetrain.MAX_SPEED
                );
        } else {
            double velocity = Constants.MPSToFalcon(desiredState.speedMetersPerSecond,
                    Constants.SwerveDrivetrain.WHEEL_CIRCUMFERENCE, Constants.SwerveDrivetrain.DRIVE_GEAR_RATIO);
            this._driveMotor.set(
                ControlMode.Velocity,
                velocity,
                DemandType.ArbitraryFeedForward,
                _feedforward.calculate(desiredState.speedMetersPerSecond)
                );
        }

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveDrivetrain.MAX_SPEED * 0.01))
                ? _lastAngle
                : desiredState.angle.getDegrees(); // Prevent rotating module if speed is less then 1%. Prevents
                                                   // Jittering.
        this._lastAngle = angle;
        this._angleMotor.set(
            ControlMode.Position,
            Constants.degreesToFalcon(angle, Constants.SwerveDrivetrain.ANGLE_GEAR_RATIO)
            );
    }

    private void configAngleMotor() {
        TalonFXConfiguration swerveAngleTalonFXConfig = new TalonFXConfiguration();
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.SwerveDrivetrain.ANGLE_ENABLE_CURRENT_LIMIT, 
            Constants.SwerveDrivetrain.ANGLE_CONTINUOUS_CL, 
            Constants.SwerveDrivetrain.ANGLE_PEAK_CL, 
            Constants.SwerveDrivetrain.ANGLE_PEAK_CURRENT_DURATION);

        swerveAngleTalonFXConfig.slot0.kP = Constants.SwerveDrivetrain.ANGLE_kP;
        swerveAngleTalonFXConfig.slot0.kI = Constants.SwerveDrivetrain.ANGLE_kI;
        swerveAngleTalonFXConfig.slot0.kD = Constants.SwerveDrivetrain.ANGLE_kD;
        swerveAngleTalonFXConfig.slot0.kF = Constants.SwerveDrivetrain.ANGLE_kF;
        swerveAngleTalonFXConfig.supplyCurrLimit = angleSupplyLimit;
        swerveAngleTalonFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;

        _angleMotor.configFactoryDefault();
        _angleMotor.configAllSettings(swerveAngleTalonFXConfig);
        _angleMotor.setInverted(Constants.SwerveDrivetrain.ANGLE_MOTOR_INVERTED);
        _angleMotor.setNeutralMode(Constants.SwerveDrivetrain.ANGLE_NEUTRAL_MODE);
        _angleMotor.set(
            TalonFXControlMode.Position, 
            Constants.degreesToFalcon(
                getCanCoder().getDegrees() - _angleOffset,
                Constants.SwerveDrivetrain.ANGLE_GEAR_RATIO
                )
            );
    }

    private void configDriveMotor() {
        TalonFXConfiguration swerveDriveTalonFXConfig = new TalonFXConfiguration();

        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.SwerveDrivetrain.DRIVE_ENABLE_CURRENT_LIMIT, 
            Constants.SwerveDrivetrain.DRIVE_CONTINUOUS_CL, 
            Constants.SwerveDrivetrain.DRIVE_PEAK_CL, 
            Constants.SwerveDrivetrain.DRIVE_PEAK_CURRENT_DURATION);

        swerveDriveTalonFXConfig.slot0.kP = Constants.SwerveDrivetrain.DRIVE_kP;
        swerveDriveTalonFXConfig.slot0.kI = Constants.SwerveDrivetrain.DRIVE_kI;
        swerveDriveTalonFXConfig.slot0.kD = Constants.SwerveDrivetrain.DRIVE_kD;
        swerveDriveTalonFXConfig.slot0.kF = Constants.SwerveDrivetrain.DRIVE_kF;        
        swerveDriveTalonFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveTalonFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        swerveDriveTalonFXConfig.openloopRamp = Constants.SwerveDrivetrain.OPEN_LOOP_RAMP;
        swerveDriveTalonFXConfig.closedloopRamp = Constants.SwerveDrivetrain.CLOSED_LOOP_RAMP;
        
        _driveMotor.configFactoryDefault();
        _driveMotor.configAllSettings(swerveDriveTalonFXConfig);
        _driveMotor.setInverted(Constants.SwerveDrivetrain.DRIVE_MOTOR_INVERTED);
        _driveMotor.setNeutralMode(Constants.SwerveDrivetrain.DRIVE_NEUTRAL_MODE);
        _driveMotor.setSelectedSensorPosition(0);
    }

    public Rotation2d getCanCoder() {
        // Encoder Range: 0 - 72
        double raw = _angleEncoder.get();
        double temp = raw * 360;
        //SmartDashboard.putNumber("Encoder for Module " + this.moduleNumber + " Raw Position: ", raw);
        return Rotation2d.fromDegrees(temp);
    }

    public SwerveModuleState getState() {
        double velocity = Constants.falconToMPS(this._driveMotor.getSelectedSensorVelocity(),
                Constants.SwerveDrivetrain.WHEEL_CIRCUMFERENCE, Constants.SwerveDrivetrain.DRIVE_GEAR_RATIO);
        Rotation2d angle = Rotation2d.fromDegrees(Constants.falconToDegrees(
                this._angleMotor.getSelectedSensorPosition(), Constants.SwerveDrivetrain.ANGLE_GEAR_RATIO));
        return new SwerveModuleState(velocity, angle);
    }

    public void stopDriveMotor() {
        this._driveMotor.set(TalonFXControlMode.Disabled, 0);
    }

    public double getDriveEncoderPosition() {
        return _driveMotor.getSelectedSensorPosition();
    }

    public double getRotateEncoderPosition() {
        return _angleMotor.getSelectedSensorPosition();
    }

    public void resetDriveEncoderPosition() {
        this._driveMotor.setSelectedSensorPosition(0);
    }

    public void stopRotateMotor() {
        this._angleMotor.set(TalonFXControlMode.Disabled, 0);
    }

    public void resetRotateEncoderPosition() {
        this._angleMotor.setSelectedSensorPosition(0);
    }

    public PIDController getAzimuthPIDController() {
        // You might want to tune these constants based on your specific requirements
        return new PIDController(
            Constants.SwerveDrivetrain.DRIVE_kP,
            Constants.SwerveDrivetrain.DRIVE_kI,
            Constants.SwerveDrivetrain.DRIVE_kD
            );
    }

    public PIDController getDrivePIDController() {
        // You might want to tune these constants based on your specific requirements
        return new PIDController(
            Constants.SwerveDrivetrain.ANGLE_kP,
            Constants.SwerveDrivetrain.ANGLE_kI,
            Constants.SwerveDrivetrain.ANGLE_kD
        );
    }
}
