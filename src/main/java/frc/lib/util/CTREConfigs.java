package frc.lib.util;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import frc.robot.Constants;

public final class CTREConfigs {
    
    public TalonFXConfiguration swerveDriveTalonFXConfig;
    public TalonFXConfiguration swerveAngleTalonFXConfig;
    public CANCoderConfiguration swerveCANCoderConfig;

    public CTREConfigs () {
        swerveDriveTalonFXConfig   = new TalonFXConfiguration();
        swerveAngleTalonFXConfig   = new TalonFXConfiguration();
        swerveCANCoderConfig       = new CANCoderConfiguration();

        /* Swerve Angle Motor Configurations */
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

        
        /* Swerve CANCoder Configuration */
        swerveCANCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        //swerveCANCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        swerveCANCoderConfig.sensorDirection = Constants.SwerveDrivetrain.CAN_CODER_INVERTED;
        swerveCANCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCANCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }

}
