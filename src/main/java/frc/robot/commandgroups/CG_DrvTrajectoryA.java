// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CG_DrvTrajectoryA extends SequentialCommandGroup {
  private SwerveDrivetrain drive;

  /** Creates a new CG_DrvTrajectoryA. */
  public CG_DrvTrajectoryA(SwerveDrivetrain drive) {
    this.drive = drive;
     
    // 1. Create trajectory settings
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        Constants.Auton.MAX_SPEED_MPS,
        Constants.Auton.MAX_ACCELERATION_MPSS)
            .setReversed(true) 
            .setKinematics(Constants.SwerveDrivetrain.SWERVE_KINEMATICS);

    var initStart = new Pose2d(Units.feetToMeters(0.0), Units.feetToMeters(0.0),
        Rotation2d.fromDegrees(0));
    var firstBall = new Pose2d(Units.feetToMeters(0.3), Units.feetToMeters(4.2),
        Rotation2d.fromDegrees(-2.0));

    var interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(new Translation2d(Units.feetToMeters(0.00), Units.feetToMeters(0.3)));
        interiorWaypoints.add(new Translation2d(Units.feetToMeters(0.05), Units.feetToMeters(1.0)));
        interiorWaypoints.add(new Translation2d(Units.feetToMeters(0.10), Units.feetToMeters(2.0)));
        interiorWaypoints.add(new Translation2d(Units.feetToMeters(0.20), Units.feetToMeters(3.0)));

                      
    // 2. Generate trajectory
    /*
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
              new Pose2d(0, 0, new Rotation2d(0)),
              List.of(
                      new Translation2d(1, 0),
                      new Translation2d(1, -1)),
              new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
              trajectoryConfig);
    */
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                      initStart,
                      interiorWaypoints,
                      firstBall,
                      trajectoryConfig);
                      
      // 3. Define PID controllers for tracking trajectory
      PIDController xController = Constants.Auton.PX_CONTROLLER;
      PIDController yController = Constants.Auton.PY_CONTROLLER;
      ProfiledPIDController thetaController = Constants.Auton.THETA_CONTROLLER;
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

      // 4. Construct command to follow trajectory
      SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
              trajectory,
              drive::getPose,
              Constants.SwerveDrivetrain.SWERVE_KINEMATICS,
              xController,
              yController,
              thetaController,
              drive::setModuleStates,
              drive);        
              
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //Run trajectory
      new InstantCommand(() -> this.drive.resetOdometry(trajectory.getInitialPose())),
      swerveControllerCommand,
      new InstantCommand(() -> this.drive.stopSwerveDrvMotors())
      //new InstantCommand(() -> this.drive.stopSwerveRotMotors())        
    );
  }
}
