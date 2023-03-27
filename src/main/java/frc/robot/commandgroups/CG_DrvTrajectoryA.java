// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;

import java.util.ArrayList;
//import java.util.List;
import java.util.List;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CG_DrvTrajectoryA extends SequentialCommandGroup {
        
        /** Creates a new CG_DrvTrajectoryA. */
        public CG_DrvTrajectoryA(SwerveDrivetrain drive, LiftClaw liftClaw, LiftElevator liftElevator) {

        
                // 1. Create trajectory settings
                /*TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                                Constants.Auton.MAX_SPEED_MPS,
                                Constants.Auton.MAX_ACCELERATION_MPSS)
                                .setReversed(true)
                                .setKinematics(Constants.SwerveDrivetrain.SWERVE_KINEMATICS);
*/
                var start_P = new Pose2d(Units.feetToMeters(0.0), Units.feetToMeters(0.0),
                                Rotation2d.fromDegrees(0));
                var final_P = new Pose2d(Units.feetToMeters(-1.0), Units.feetToMeters(0.0),
                                Rotation2d.fromDegrees(0));

                List<Pose2d> poses = new ArrayList<Pose2d>();
                poses.add(start_P);
                poses.add(final_P);

                // 2. Generate trajectory
                /*
                 * Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                 * new Pose2d(0, 0, new Rotation2d(0)),
                 * List.of(
                 * new Translation2d(1, 0),
                 * new Translation2d(1, -1)),
                 * new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
                 * trajectoryConfig);
                 */
                // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(poses,
                // trajectoryConfig);
                //Trajectory trajectory = TrajectoryGenerator.generateTrajectory(start_P, List.of(), final_P,
                //                trajectoryConfig);

                // 3. Define PID controllers for tracking trajectory
                //PIDController xController = Constants.Auton.PX_CONTROLLER;
                //PIDController yController = Constants.Auton.PY_CONTROLLER;
                ProfiledPIDController thetaController = Constants.Auton.THETA_CONTROLLER;
                thetaController.enableContinuousInput(-Math.PI, Math.PI);

                /*/ 4. Construct command to follow trajectory
                SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                                trajectory,
                                drive::getPose,
                                Constants.SwerveDrivetrain.SWERVE_KINEMATICS,
                                xController,
                                yController,
                                thetaController,
                                drive::setModuleStates,
                                drive);
                                */
                // Add your commands in the addCommands() call, e.g.
                // addCommands(new FooCommand(), new BarCommand());
                addCommands(
                                /*
                                 * (new CC_SwerveResetRotEncdrs(drive)),
                                 * (new CC_SwerveResetDrvEncdrs(drive)),
                                 * (new CC_SwerveZeroGyro(drive)),
                                 * (new CC_TimeDly(0.100)),
                                 * (new CC_SwerveZeroRotEncdrs(drive)),
                                 * (new CC_TimeDly(0.150)),
                                 * // Run trajectory
                                 */
                                // new InstantCommand(() -> drive.resetOdometry(start_P)),
                                new InstantCommand(() -> drive.stopSwerveDriveMotors()),
                                new CA_Elevator(liftElevator, true),
                                new CA_PitchElevator(liftElevator, true),
                                new CA_ToggleClaw(liftClaw),
                                new CA_PitchElevator(liftElevator, false),
                                new RunCommand(() -> drive.drive(new Translation2d(-0.5, 0.0), 0, false, true), drive)
                                                .withTimeout(1),
                                new InstantCommand(() -> drive.stopSwerveDriveMotors())
                // new InstantCommand(() -> drive.stopSwerveRotMotors())
                );
        }
}
