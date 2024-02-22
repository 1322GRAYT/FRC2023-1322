package frc.robot.commandgroups;

import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Path;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class FollowTrajectory extends SequentialCommandGroup {

    private SwerveDrivetrain _swerveSubsystem;

    public Pose2d getPose() {
        return _swerveSubsystem.getPose();
      }
    
      public void setStates(SwerveModuleState[] states) {
        _swerveSubsystem.setModuleStatesOpenLoop(states);
      }

      public Trajectory getTrajectory(String trajectoryName) {
        Trajectory trajectory = new Trajectory();
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryName);
        try {
           trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
          System.out.println("Unable to open trajectory: " + trajectoryPath);
          ex.printStackTrace((new PrintWriter(System.out)));
        }
        return trajectory;
      }
    public FollowTrajectory(
            String trajectory, 
            SwerveDrivetrain swerveSubsystem
            ) {
        this._swerveSubsystem = swerveSubsystem;
        ProfiledPIDController thetaController = Constants.Auton.THETA_CONTROLLER;
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        addCommands(
            new SwerveControllerCommand(
                getTrajectory(trajectory),
                this::getPose, 
                swerveSubsystem.getKinematics(),
                Constants.Auton.PX_CONTROLLER,
                Constants.Auton.PY_CONTROLLER,
                thetaController,
                this::setStates,
                new Subsystem [] {swerveSubsystem}
              )
        );
    }

    
}
