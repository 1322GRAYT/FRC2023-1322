// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.subsystems.swerve.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static void configMotor(TalonFX motor) {
        TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();
    
        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.SwerveDrivetrain.DRIVE_ENABLE_CURRENT_LIMIT, 
            Constants.SwerveDrivetrain.DRIVE_CONTINUOUS_CL, 
            Constants.SwerveDrivetrain.DRIVE_PEAK_CL, 
            Constants.SwerveDrivetrain.DRIVE_PEAK_CURRENT_DURATION);
    
        talonFXConfig.slot0.kP = Constants.SwerveDrivetrain.DRIVE_kP;
        talonFXConfig.slot0.kI = Constants.SwerveDrivetrain.DRIVE_kI;
        talonFXConfig.slot0.kD = Constants.SwerveDrivetrain.DRIVE_kD;
        talonFXConfig.slot0.kF = Constants.SwerveDrivetrain.DRIVE_kF;        
        talonFXConfig.supplyCurrLimit = driveSupplyLimit;
        talonFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        talonFXConfig.openloopRamp = Constants.SwerveDrivetrain.OPEN_LOOP_RAMP;
        talonFXConfig.closedloopRamp = Constants.SwerveDrivetrain.CLOSED_LOOP_RAMP;
        
        motor.configFactoryDefault();
        motor.configAllSettings(talonFXConfig);
        //motor.setInverted(Constants.SwerveDrivetrain.DRIVE_MOTOR_INVERTED);
        motor.setNeutralMode(Constants.SwerveDrivetrain.DRIVE_NEUTRAL_MODE);
    }


    // April Tag List 2024
    public static final Map<Integer, String> AprilTags = new HashMap<Integer, String>() {{
        put(1, "Blue Source - Right");
        put(2, "Blue Source - Left");
        put(3, "Red Source - Right");
        put(4, "Red Source - Left");
        put(5, "Red Amp");
        put(6, "Blue Amp");
        put(7, "Blue Speaker - Right");
        put(8, "Blue Speaker - Left");
        put(9, "Red Speaker - Right");
        put(10, "Red Speaker - Left");
        put(11, "Red Stage - Left");
        put(12, "Red Stage - Right");
        put(13, "Red Stage - Center");
        put(14, "Blue Stage - Center");
        put(15, "Blue Stage - Left");
        put(16, "Blue Stage - Right");
    }};

    /* X-BOX CONTROLLER MAPPING */
    // Controller Assignments
    public static final int DRVR_CNTRLR = 0;
    public static final int AUX_CNTRLR = 1;
    // Button Assignments
    public static final String [] BUTTONS = {"None","A","B","X","Y","LeftBumper","RightBumper", "Back", "Start", "LeftStickPress", "RightStickPress"};
    public static final int BUTTON_A = 1;
    public static final int BUTTON_B = 2;
    public static final int BUTTON_X = 3;
    public static final int BUTTON_Y = 4;
    public static final int BUMPER_LEFT = 5;
    public static final int BUMPER_RIGHT = 6;
    public static final int BUTTON_BACK = 7; // LEFT(SELECT)
    public static final int BUTTON_START = 8; // RIGHT
    public static final int STICK_LEFT_PRESS = 9; // JOYSTICK PRESS
    public static final int STICK_RIGHT_PRESS = 10; // JOYSTICK PRESS
    // Analog Assignments
    public static final int STICK_LEFT_XAXIS = 1;
    public static final int STICK_LEFT_YAXIS = 2;
    public static final int TRIGGERS = 3;
    public static final int STICK_RIGHT_XAXIS = 4;
    public static final int STICK_RIGHT_YAXIS = 5;
    public static final int DPAD = 6;


    /*
     * First pickup, then preshoot, then lift, then shooter
     */
    public static final int SHOOTER_MOTOR_0 = 13;
    public static final int SHOOTER_MOTOR_1 = 10;
    public static final int SHOOTER_PRESHOOT = 12;

    public static final int FLOOR_PICKUP = 18;
    public static final int LIFT_MOTOR = 17;
    public static final int TILT_MOTOR = 15;

    public static final double SHOOTER_VELOCITY = 1.0;
    public static final double PRESHOOT_INITIAL_VELOCITY = 1.0;
    public static final double PRESHOOT_TIME = 3.0;
    public static final int SHOOTER_SENSOR0 = 5;
    public static final int SHOOTER_SENSOR1 = 6;
    public static final int SHOOTER_SENSOR2 = 7;

    public static final int TILT_LOAD_POSITION = 100;



    /*
    public static int MOTOR_9 = 9;
    public static int MOTOR_10 = 10;
    public static int MOTOR_11 = 11;
    public static int MOTOR_12 = 12;
    */
    public static String _motorStick [] = {
        "LX",
        "LY",
        "RX",
        "RY"
      };

    public static final class SwerveDrivetrain {

        /* Rotation control PID settings */
        public static final double ROTATE_kP   = 0.1;
        public static final double ROTATE_kI  = 0.001;
        public static final double ROTATE_kD  = 0;

        /* Gyro */
        public static final SPI.Port GYRO_ID = SPI.Port.kMXP;
        public static final boolean INVERT_GYRO = true;

        /* Drivetrain */
        public static final double TRACK_WIDTH = Units.inchesToMeters(23.25);
        public static final double WHEEL_BASE = Units.inchesToMeters(23.5);
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.176);
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        public static final double DRIVE_GEAR_RATIO = (10 / 1.0); // 10:1
        public static final double ANGLE_GEAR_RATIO = (19 / 1.0); // 19:1
        public static final double DRIVE_CNTS_PER_REV = 2048; // FALCON 500;

        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
        /*
        * WHEEL_BASE is distance from front to back wheel center to center
        * TRACK_WIDTH is distance from left to right wheels center to center
        */
        new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),   // front left + +
        new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),  // front right + -
        new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),  // back left - +
        new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));// back right - - 

        /* Current Limiting */
        public static final int ANGLE_CONTINUOUS_CL = 25;
        public static final int ANGLE_PEAK_CL = 40;
        public static final double ANGLE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;
        /* Angle Motor PID Values */
        public static final double ANGLE_kP = 0.6; // 0.6
        public static final double ANGLE_kI = 0.0; // 0.0
        public static final double ANGLE_kD = 12.0; // 12.0
        public static final double ANGLE_kF = 0.0; // 0.0

        

        public static final int DRIVE_CONTINUOUS_CL = 35;
        public static final int DRIVE_PEAK_CL = 60;
        public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;
        /* Drive Motor PID Values */
        public static final double DRIVE_kP = 0.10; // 0.10
        public static final double DRIVE_kI = 0.0; // 0.0
        public static final double DRIVE_kD = 0.0; // 0.0
        public static final double DRIVE_kF = 0.0; // 0.0



        /* Drive Motor Characterization Values (FeedForward) */
        public static final double FeedForwardStaticGain = (0.632 / 12); // 0.667 --- divide by 12 to convert from volts to percent
                                                         // output for CTRE
        public static final double FeedForwardVelocityGain = (0.0514 / 12); // 2.44
        public static final double FeedForwardAccerationGain = (0.00337 / 12); // 0.27

        /* Swerve Profiling Values */
        public static final double MAX_SPEED = 4.5; // m/s
        public static final double MAX_ANGULAR_VELOCITY = 11.5; // m/s
        public static final double SLOW_SPEED_REDUCTION = 0.25; // 1/4 speed

        /* Neutral Modes */
        public static final NeutralMode ANGLE_NEUTRAL_MODE = NeutralMode.Coast;
        public static final NeutralMode DRIVE_NEUTRAL_MODE = NeutralMode.Coast; // was NeutralMode.Brake

        /* Motor Inverts */
        public static final boolean DRIVE_MOTOR_INVERTED = false;
        public static final boolean ANGLE_MOTOR_INVERTED = true;

        /* Angle Encoder Invert */
        public static final boolean CAN_CODER_INVERTED = false;

        /* Module Specific Constants */

        public static final int DRIVE_MOTOR_FRONT_LEFT = 2; // Front Left
        public static final int SWERVE_MOTOR_FRONT_LEFT = 3; // Front Left
        public static final int SWERVE_CAN_CODER_FRONT_LEFT=3;
        public static final int SWERVE_ZERO_SENSOR_FRONT_LEFT=4;
        public static final double SWERVE_ANGLE_OFFSET_FRONT_LEFT=126.0;

        public static final int DRIVE_MOTOR_FRONT_RIGHT = 7; // Front Right
        public static final int SWERVE_MOTOR_FRONT_RIGHT = 4; // Front Right
        public static final int SWERVE_CAN_CODER_FRONT_RIGHT=0;
        public static final int SWERVE_ZERO_SENSOR_FRONT_RIGHT=1;
        public static final double SWERVE_ANGLE_OFFSET_FRONT_RIGHT=235.2;

        public static final int DRIVE_MOTOR_REAR_LEFT = 8; // Rear Left
        public static final int SWERVE_MOTOR_REAR_LEFT = 1; // Rear Left
        public static final int SWERVE_CAN_CODER_REAR_LEFT=2;
        public static final int SWERVE_ZERO_SENSOR_REAR_LEFT=3;
        public static final double SWERVE_ANGLE_OFFSET_REAR_LEFT=128.5;

        public static final int DRIVE_MOTOR_REAR_RIGHT = 6; // Rear Right
        public static final int SWERVE_MOTOR_REAR_RIGHT = 5; // Rear Right
        public static final int SWERVE_CAN_CODER_REAR_RIGHT=1;
        public static final int SWERVE_ZERO_SENSOR_REAR_RIGHT=2;
        public static final double SWERVE_ANGLE_OFFSET_REAR_RIGHT=138.7;
        
        // Swerve Steer Motors (FALCON 500)

        public static final double SWERVE_ZERO_SPEED=0.2;
    
    
        public static SwerveModuleConstants Modules []  = {
            // Front left
            new SwerveModuleConstants(
                DRIVE_MOTOR_FRONT_LEFT, 
                SWERVE_MOTOR_FRONT_LEFT, 
                SWERVE_CAN_CODER_FRONT_LEFT, 
                SWERVE_ANGLE_OFFSET_FRONT_LEFT, 
                SWERVE_ZERO_SENSOR_FRONT_LEFT
                ), //FL

            // Front Right
            new SwerveModuleConstants(
                DRIVE_MOTOR_FRONT_RIGHT, 
                SWERVE_MOTOR_FRONT_RIGHT, 
                SWERVE_CAN_CODER_FRONT_RIGHT, 
                SWERVE_ANGLE_OFFSET_FRONT_RIGHT, 
                SWERVE_ZERO_SENSOR_FRONT_RIGHT
                ), //FL
            //new SwerveModuleConstants(7, 4, 0, 235.2), 

            // Rear Left
            new SwerveModuleConstants(
                DRIVE_MOTOR_REAR_LEFT, 
                SWERVE_MOTOR_REAR_LEFT, 
                SWERVE_CAN_CODER_REAR_LEFT, 
                SWERVE_ANGLE_OFFSET_REAR_LEFT, 
                SWERVE_ZERO_SENSOR_REAR_LEFT
                ), //FL
            //new SwerveModuleConstants(8, 1, 2, 128.5), 

            // Rear Right
            new SwerveModuleConstants(
                DRIVE_MOTOR_REAR_RIGHT, 
                SWERVE_MOTOR_REAR_RIGHT, 
                SWERVE_CAN_CODER_REAR_RIGHT, 
                SWERVE_ANGLE_OFFSET_REAR_RIGHT, 
                SWERVE_ZERO_SENSOR_REAR_RIGHT
                ), //FL
            //new SwerveModuleConstants(6, 5, 1, 138.7)
        };

    }
  /*   public static final class Auton {

        /* Drive Motor Characterization Values (Ramsete) * /
        public static final double RAMSETE_B = 7;
        public static final double RAMSETE_ZETA = 2;

        public static final double MAX_SPEED_MPS = 7.0; // meters per second
        public static final double MAX_ACCELERATION_MPSS = 5.0; // meters per second squared

        public static final double MAX_ANGULAR_SPEED_RPS = 2 * Math.PI; // radians per second
        public static final double MAX_ANGULAR_SPEED_RPSS = 2 * Math.PI; // radians per second squared

        // public static final PIDController PX_CONTROLLER = new PIDController(5.25, 1,
        // 0.4);
        // public static final PIDController PY_CONTROLLER = new PIDController(5.25, 1,
        // 0.4);
        public static final PIDController PX_CONTROLLER = new PIDController(6.0, 0, 0.1);
        public static final PIDController PY_CONTROLLER = new PIDController(6.0, 0, 0.1);
        // public static final double PTHETA_CONTROLLER = 1.0;

        public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONTRAINTS = new TrapezoidProfile.Constraints(
                MAX_ANGULAR_SPEED_RPS, MAX_ANGULAR_SPEED_RPSS);

        // public static final ProfiledPIDController ROT_PID_CONTROLLER = new
        // ProfiledPIDController(.13, 0, .39, THETA_CONTROLLER_CONTRAINTS);
        public static final ProfiledPIDController THETA_CONTROLLER = new ProfiledPIDController(10.0, 0.0, 0.0,
                THETA_CONTROLLER_CONTRAINTS);
    }
 */

     /**
     * @param counts Falcon Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double falconToDegrees(double counts, double gearRatio) {
        return counts * (360.0 / (gearRatio * 2048.0));
    }

    /**
     * @param degrees Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Falcon Counts
     */
    public static double degreesToFalcon(double degrees, double gearRatio) {
        double ticks =  degrees / (360.0 / (gearRatio * 2048.0));
        return ticks;
    }

    /**
     * @param velocityCounts Falcon Velocity Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return RPM of Mechanism
     */
    public static double falconToRPM(double velocityCounts, double gearRatio) {
        double motorRPM = velocityCounts * (600.0 / 2048.0);        
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }

    /**
     * @param RPM RPM of mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return RPM of Mechanism
     */
    public static double RPMToFalcon(double RPM, double gearRatio) {
        double motorRPM = RPM * gearRatio;
        double sensorCounts = motorRPM * (2048.0 / 600.0);
        return sensorCounts;
    }

    /**
     * @param velocitycounts Falcon Velocity Counts
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return Falcon Velocity Counts
     */
    public static double falconToMPS(double velocitycounts, double circumference, double gearRatio){
        double wheelRPM = falconToRPM(velocitycounts, gearRatio);
        double wheelMPS = (wheelRPM * circumference) / 60;
        return wheelMPS;
    }

    /**
     * @param velocity Velocity MPS
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return Falcon Velocity Counts
     */
    public static double MPSToFalcon(double velocity, double circumference, double gearRatio){
        double wheelRPM = ((velocity * 60) / circumference);
        double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
        return wheelVelocity;
    }

    public static double ApplyDeadBand_Scaled( double power, double deadBand, double powerLimit) {
		if (power > -deadBand && power < deadBand) return 0.0;
		double sign = (power>0)?1:((power<0)?-1:0);
		if (power > powerLimit || power < - powerLimit) return powerLimit*sign;
		return ((power - sign*deadBand)/(powerLimit - deadBand))*powerLimit;
	}
    
}
