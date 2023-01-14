// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.utils.swerve.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // Enum for Solenoid control
    public enum SolenoidPosition {
        UP, DOWN, OFF
    }

   /* MOTOR CAN ADDRESS ASSIGNMENTS */
    // NOTE: These are tentative and are subject to change
    // Drive Motors (FALCON 500)
    public static final int DRV_MTR_FT_LT         =  2;  //  Front Left
    public static final int DRV_MTR_FT_RT         =  7;  //  Front Right
    public static final int DRV_MTR_RR_LT         =  8;  //  Rear  Left
    public static final int DRV_MTR_RR_RT         =  6;  //  Rear Right
    // Swerve Steer Motors (FALCON 500)
    public static final int SWRV_MTR_FT_LT        =  3;  //  Front Left
    public static final int SWRV_MTR_FT_RT        =  4;  //  Front Right
    public static final int SWRV_MTR_RR_LT        =  1;  //  Rear  Left
    public static final int SWRV_MTR_RR_RT        =  5;  //  Rear Right


    // Ball Intake Motor (CAN TALON SRX)
    public static final int BALL_MTR_INTAKE       =  9;
    // Ball Advance Motor (FALCON 500)
    public static final int BALL_MTR_ADVANCE      = 10;
    
    // Shooter Motors (FALCON 500)
    public static final int SHOOTER_MTR_RT        = 11;
    public static final int SHOOTER_MTR_LT        = 12;

    // Robot Lift Motor (FALCON 500)
    public static final int LIFT_MTR              = 13;



    
    /* PWM OUTPUT ADDRESS ASSIGNMENTS */
    public static final int PWM_SHOOTER_ANGLE     = 0;
    public static final int PWM_CAMERA_TILT       = 1;
    public static final int PWM_CAMERA_PAN        = 2;



   /* PNEUMATIC HUB CAN ADDRESS ASSIGNMENTS */
    public static final int PNEU_HUB_CAN         =  1;

    /* PNEUMATIC ACTUATOR ADDRESS ASSIGNMENTS */
    public static final int PNEU_PRESSURE_SENSOR  =  0;
    public static final int PNEU_BALL_INTAKE_LT   = 11;
    public static final int PNEU_BALL_INTAKE_RT   =  9;
    public static final int PNEU_BALL_INTAKE_FT   = 10;
    public static final int PNEU_BALL_INTAKE_RR   = 12;
    public static final int PNEU_LIFT_TRACK       = 13;
    public static final int PNEU_SHOOTER_CAMERA   =  8;


    /* ANALOG INPUT ADDRESS ASSIGNMENTS */
    // Swerve Steer Motors Position
    public static final int ANA_SWRV_ANG_FT_LT    = 3;  //  Front Left
    public static final int ANA_SWRV_ANG_RR_LT    = 2;  //  Rear  Left
    public static final int ANA_SWRV_ANG_FT_RT    = 0;  //  Front Right
    public static final int ANA_SWRV_ANG_RR_RT    = 1;  //  Rear  Right



    /* DIGITAL INPUT ADDRESS ASSIGNMENTS */
    public static final int SW_LIFT_TRACK_TRIG    =  0;
    public static final int SW_BALL_INTAKE_LT     =  1;
    public static final int SW_BALL_INTAKE_RT     =  3;
    public static final int SW_BALL_INTAKE_FT     =  4;
    public static final int SW_BALL_INTAKE_RR     =  2;
    public static final int SW_BALL_ADVANCE_1     =  5;
    public static final int SW_BALL_ADVANCE_2     =  6;





    /* X-BOX CONTROLLER MAPPING */
    // Controller Assignments
    public static final int DRVR_CNTRLR           =  0;
    public static final int AUX_CNTRLR            =  1;
    // Button Assignments
    public static final int BUTTON_A              =  1;
    public static final int BUTTON_B              =  2;
    public static final int BUTTON_X              =  3;
    public static final int BUTTON_Y              =  4;
    public static final int BUMPER_LEFT           =  5;
    public static final int BUMPER_RIGHT          =  6;
    public static final int BUTTON_BACK           =  7;  // LEFT(SELECT)
    public static final int BUTTON_START          =  8;  // RIGHT
    public static final int STICK_LEFT_PRESS      =  9;  // JOYSTICK PRESS
    public static final int STICK_RIGHT_PRESS     = 10;  // JOYSTICK PRESS
    // Analog Assignments
    public static final int STICK_LEFT_XAXIS      = 1;
    public static final int STICK_LEFT_YAXIS      = 2;
    public static final int TRIGGERS              = 3;
    public static final int STICK_RIGHT_XAXIS     = 4;
    public static final int STICK_RIGHT_YAXIS     = 5;
    public static final int DPAD                  = 6;

    public static final class SwerveDrivetrain {

        /* Gyro */
        public static final SPI.Port GYRO_ID = SPI.Port.kMXP;
        public static final boolean INVERT_GYRO = true;

        /* Drivetrain */
        public static final double TRACK_WIDTH          = Units.inchesToMeters(23.25);
        public static final double WHEEL_BASE           = Units.inchesToMeters(23.5);
        public static final double WHEEL_DIAMETER       = Units.inchesToMeters(4.176);
        public static final double WHEEL_CIRCUMFERENCE  = WHEEL_DIAMETER * Math.PI;

        public static final double OPEN_LOOP_RAMP   = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        public static final double DRIVE_GEAR_RATIO = (10 / 1.0);  // 10:1
        public static final double ANGLE_GEAR_RATIO = (19 / 1.0);  // 19:1
        public static final double DRIVE_CNTS_PER_REV  = 2048;     // FALCON 500;

        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(  WHEEL_BASE / 2.0,   TRACK_WIDTH / 2.0),
            new Translation2d(  WHEEL_BASE / 2.0,  -TRACK_WIDTH / 2.0),
            new Translation2d( -WHEEL_BASE / 2.0,   TRACK_WIDTH / 2.0),
            new Translation2d( -WHEEL_BASE / 2.0,  -TRACK_WIDTH / 2.0)
        );

        /* Current Limiting */
        public static final int ANGLE_CONTINUOUS_CL = 25;
        public static final int ANGLE_PEAK_CL       = 40;
        public static final double ANGLE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

        public static final int DRIVE_CONTINUOUS_CL = 35;
        public static final int DRIVE_PEAK_CL       = 60;
        public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

        /* Angle Motor PID Values */
        public static final double ANGLE_kP = 0.6;   // 0.6
        public static final double ANGLE_kI = 0.0;   // 0.0
        public static final double ANGLE_kD = 12.0;  // 12.0
        public static final double ANGLE_kF = 0.0;   // 0.0

        /* Drive Motor PID Values */
        public static final double DRIVE_kP = 0.10;  // 0.10
        public static final double DRIVE_kI = 0.0;   // 0.0
        public static final double DRIVE_kD = 0.0;   // 0.0
        public static final double DRIVE_kF = 0.0;   // 0.0

        /* Drive Motor Characterization Values (FeedForward) */
        public static final double FF_kS    = (0.632 / 12);     // 0.667 --- divide by 12 to convert from volts to percent output for CTRE
        public static final double FF_kV    = (0.0514 / 12);    // 2.44
        public static final double FF_kA    = (0.00337 / 12);   // 0.27

        /* Swerve Profiling Values */
        public static final double MAX_SPEED            = 4.5;  // m/s
        public static final double MAX_ANGULAR_VELOCITY = 11.5; // m/s

        /* Neutral Modes */
        public static final NeutralMode ANGLE_NEUTRAL_MODE = NeutralMode.Coast;
        public static final NeutralMode DRIVE_NEUTRAL_MODE = NeutralMode.Coast;  // was NeutralMode.Brake 

        /* Motor Inverts */
        public static final boolean DRIVE_MOTOR_INVERTED = false;
        public static final boolean ANGLE_MOTOR_INVERTED = true;

        /* Angle Encoder Invert */
        public static final boolean CAN_CODER_INVERTED = false;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int DRIVE_MOTOR_ID  = 2;
            public static final int ANGLE_MOTOR_ID  = 3;
            public static final int CAN_CODER_ID    = 3;
            public static final double ANGLE_OFFSET = 126.0;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int DRIVE_MOTOR_ID  = 7;
            public static final int ANGLE_MOTOR_ID  = 4;
            public static final int CAN_CODER_ID    = 0;
            public static final double ANGLE_OFFSET = 235.2;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int DRIVE_MOTOR_ID  = 8;
            public static final int ANGLE_MOTOR_ID  = 1;
            public static final int CAN_CODER_ID    = 2;
            public static final double ANGLE_OFFSET = 128.5;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int DRIVE_MOTOR_ID  = 6;
            public static final int ANGLE_MOTOR_ID  = 5;
            public static final int CAN_CODER_ID    = 1;
            public static final double ANGLE_OFFSET = 138.7;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }
    }

    public static final class Auton {
        
        /* Drive Motor Characterization Values (Ramsete) */
        public static final double RAMSETE_B                = 7;
        public static final double RAMSETE_ZETA             = 2;

        public static final double MAX_SPEED_MPS            = 7.0;    // meters per second
        public static final double MAX_ACCELERATION_MPSS    = 5.0;    // meters per second squared

        public static final double MAX_ANGULAR_SPEED_RPS    = 2 * Math.PI;      // radians per second
        public static final double MAX_ANGULAR_SPEED_RPSS   = 2 * Math.PI;      // radians per second squared

        // public static final PIDController PX_CONTROLLER = new PIDController(5.25, 1, 0.4);
        // public static final PIDController PY_CONTROLLER = new PIDController(5.25, 1, 0.4);
        public static final PIDController PX_CONTROLLER = new PIDController(6.0, 0, 0.1);
        public static final PIDController PY_CONTROLLER = new PIDController(6.0, 0, 0.1);
        // public static final double PTHETA_CONTROLLER    = 1.0;

        public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONTRAINTS = new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED_RPS, MAX_ANGULAR_SPEED_RPSS);

        // public static final ProfiledPIDController ROT_PID_CONTROLLER = new ProfiledPIDController(.13, 0, .39, THETA_CONTROLLER_CONTRAINTS);
        public static final ProfiledPIDController THETA_CONTROLLER = new ProfiledPIDController(10.0, 0.0, 0.0, THETA_CONTROLLER_CONTRAINTS);
    }
}
