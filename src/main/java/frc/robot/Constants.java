// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.subsystems.swerve.SwerveModuleConstants;

public final class Constants {

    public static final double BEAM_BALANACED_DRIVE_KP = 0.015; // P (Proportional) constant of a PID loop
    public static final double BEAM_BALANCED_GOAL_DEGREES = 0;
    public static final double BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES = 1;
    public static final double BEAM_BACKWARDS_BALANCING_EXTRA_POWER_MULTIPLIER = 1.35;
    public static final double BEAM_BALANCE_POWER_MULTIPLIER = 0.6;

    
    public static final int CLAW_YAW_SERVO = 0;
    
    
    public static final int ELEVATOR_MOTOR = 10;
    public static final int ELEVATOR_PITCH_MOTOR = 13;

    public static final int CLAW_PITCH = 12;

    public static final int PNEUMATIC_COMPRESSOR = 1;
    public static final boolean PNEUMATIC_COMPRESSOR_DEBUG_ENABLE=true;

    public static final int PNEUMATIC_CLAW_0 = 11;
    public static final int PNEUMATIC_CLAW_1 = 10;

    public static final int SW_LIFT_TRACK_TRIG = 0;

    public static final int DRVR_CNTRLR = 0;
    public static final int AUX_CNTRLR = 1;

    public static final int BUTTON_A = 1;
    public static final int BUTTON_B = 2;
    public static final int BUTTON_X = 3;
    public static final int BUTTON_Y = 4;
    public static final int BUMPER_LEFT = 5;
    public static final int BUMPER_RIGHT = 6;

    public static final SPI.Port GYRO_ID = SPI.Port.kMXP;
    public static final boolean INVERT_GYRO = true;


    public static final double AUX_STICK_DEADBAND = 0.1;
    

    // TODO: Rename these
    public static final boolean KeSWRV_DebugEnable = true;
    public static final double KeSWRV_DriveSyncThresholdRotation = 0.100;
    public static final double KeSWRV_k_CL_PropGx_Rot = 0.1;
    public static final double KeSWRV_k_CL_IntglGx_Rot = 0.001;
    public static final double KeSWRV_k_CL_DerivGx_Rot = 0;

    public static final class SwerveDrivetrain {

        /* Drivetrain */
        public static final double TRACK_WIDTH = Units.inchesToMeters(23.25);
        public static final double WHEEL_BASE = Units.inchesToMeters(23.5);
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.176);
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        public static final double DRIVE_GEAR_RATIO = (10 / 1.0); // 10:1
        public static final double ANGLE_GEAR_RATIO = (19 / 1.0); // 19:1
        public static final double DRIVE_COUNTS_PER_REVOLUTION = 2048; // FALCON 500;
        public static final double FALCON_TO_DEGREES = (360.0 / ( DRIVE_COUNTS_PER_REVOLUTION));
        public static final double ANGLE_FALCON_TO_DEGREES = FALCON_TO_DEGREES/ANGLE_GEAR_RATIO;
        public static final double DRIVE_FALCON_TO_DEGREES = FALCON_TO_DEGREES/DRIVE_GEAR_RATIO;
        
        public static double DEGREES_TO_FALCON=1/FALCON_TO_DEGREES;
        public static final double DRIVE_DEGREES_TO_FALCON = DEGREES_TO_FALCON / DRIVE_GEAR_RATIO;
        public static final double ANGLE_DEGREES_TO_FALCON = DEGREES_TO_FALCON / ANGLE_GEAR_RATIO;


        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0),  //FRONT RIGHT
                new Translation2d(TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0),  //REAR RIGHT
                new Translation2d(-TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0), //FRONT LEFT
                new Translation2d(-TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0) //REAR LEFT
                );

/*
        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
            // Original: this is wrong......this is why the x and y are mixed up in drive and other places.
                new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));
 */

        /* Current Limiting */
        public static final int ANGLE_CONTINUOUS_CL = 25;
        public static final int ANGLE_PEAK_CL = 40;
        public static final double ANGLE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

        public static final int DRIVE_CONTINUOUS_CL = 35;
        public static final int DRIVE_PEAK_CL = 60;
        public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

        /* Angle Motor PID Values */
        public static final double ANGLE_kP = 0.6; // 0.6
        public static final double ANGLE_kI = 0.0; // 0.0
        public static final double ANGLE_kD = 12.0; // 12.0
        public static final double ANGLE_kF = 0.0; // 0.0

        /* Drive Motor PID Values */
        public static final double DRIVE_kP = 0.10; // 0.10
        public static final double DRIVE_kI = 0.0; // 0.0
        public static final double DRIVE_kD = 0.0; // 0.0
        public static final double DRIVE_kF = 0.0; // 0.0

        /* Drive Motor Characterization Values (FeedForward) */
        public static final double FF_kS = (0.632 / 12); // 0.667 --- divide by 12 to convert from volts to percent
                                                         // output for CTRE
        public static final double FF_kV = (0.0514 / 12); // 2.44
        public static final double FF_kA = (0.00337 / 12); // 0.27

        /* Swerve Profiling Values */
        public static final double MAX_SPEED = 4.5; // m/s
        public static final double MAX_ANGULAR_VELOCITY = 11.5; // m/s

        /* Neutral Modes */
        public static final NeutralMode ANGLE_NEUTRAL_MODE = NeutralMode.Coast;
        public static final NeutralMode DRIVE_NEUTRAL_MODE = NeutralMode.Coast; // was NeutralMode.Brake

        /* Motor Inverts */
        public static final boolean DRIVE_MOTOR_INVERTED = false;
        public static final boolean ANGLE_MOTOR_INVERTED = true;

        /* Angle Encoder Invert */
        public static final boolean CAN_CODER_INVERTED = false;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final SwerveModuleConstants Module0 = new SwerveModuleConstants(2, 3, 3);

        /* Front Right Module - Module 1 */
        public static final SwerveModuleConstants Module1 = new SwerveModuleConstants(7, 4, 0);

        /* Back Left Module - Module 2 */
        public static final SwerveModuleConstants Module2 = new SwerveModuleConstants(8, 1, 2);

        /* Back Right Module - Module 3 */
        public static final SwerveModuleConstants Module3 = new SwerveModuleConstants(6, 5, 1);
    }


}
