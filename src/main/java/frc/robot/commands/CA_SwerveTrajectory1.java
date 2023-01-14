// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.subsystems.SwerveDrivetrain;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.CommandBase;
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


public class CA_SwerveTrajectory1 extends CommandBase {
  private SwerveDrivetrain drive;

  /** Creates a new CA_SwerveTrajectory1. */
  public CA_SwerveTrajectory1(SwerveDrivetrain drive) {
    this.drive = drive;

    addRequirements(drive);
  }


  private void generateTrajectory() {

      // 1. Create trajectory settings
      TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
          Auton.MAX_SPEED_MPS,
          Auton.MAX_ACCELERATION_MPSS)
              .setReversed(false) 
              .setKinematics(Constants.SwerveDrivetrain.SWERVE_KINEMATICS);


      var initStart = new Pose2d(Units.feetToMeters(0.0), Units.feetToMeters(0.0),
          Rotation2d.fromDegrees(0));
      var firstBall = new Pose2d(Units.feetToMeters(5.0), Units.feetToMeters(6.0),
          Rotation2d.fromDegrees(90));
  
      var interiorWaypoints = new ArrayList<Translation2d>();
          interiorWaypoints.add(new Translation2d(Units.feetToMeters(0.0), Units.feetToMeters(2.0)));
          interiorWaypoints.add(new Translation2d(Units.feetToMeters(2.0), Units.feetToMeters(4.0)));
          interiorWaypoints.add(new Translation2d(Units.feetToMeters(3.5), Units.feetToMeters(5.0)));
  
                        
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
        PIDController xController = Auton.PX_CONTROLLER;
        PIDController yController = Auton.PY_CONTROLLER;
        ProfiledPIDController thetaController = Auton.THETA_CONTROLLER;
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

  }

}
