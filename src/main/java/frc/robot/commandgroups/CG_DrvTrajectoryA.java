package frc.robot.commandgroups;

import frc.robot.subsystems.SwerveDrivetrain;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.*;

public class CG_DrvTrajectoryA extends SequentialCommandGroup {

        /** Creates a new CG_DrvTrajectoryA. */
        public CG_DrvTrajectoryA(SwerveDrivetrain drive, Claw claw, LiftElevator liftElevator) {
                var start_P = new Pose2d(Units.feetToMeters(0.0), Units.feetToMeters(0.0),
                                Rotation2d.fromDegrees(0));
                var final_P = new Pose2d(Units.feetToMeters(-1.0), Units.feetToMeters(0.0),
                                Rotation2d.fromDegrees(0));

                List<Pose2d> poses = new ArrayList<Pose2d>();
                poses.add(start_P);
                poses.add(final_P);

                TrapezoidProfile.Constraints THETA_CONTROLLER_CONTRAINTS = new TrapezoidProfile.Constraints(2 * Math.PI,
                                2 * Math.PI);
                ProfiledPIDController thetaController = new ProfiledPIDController(10.0, 0.0, 0.0,
                                THETA_CONTROLLER_CONTRAINTS);
                thetaController.enableContinuousInput(-Math.PI, Math.PI);

                addCommands(
                                new InstantCommand(() -> drive.stopSwerveDriveMotors()),
                                new CA_Elevator(liftElevator, true),
                                new CA_PitchElevator(liftElevator, true),
                                new CA_ToggleClaw(claw),
                                new CA_PitchElevator(liftElevator, false),
                                new RunCommand(() -> drive.drive(new Translation2d(0.0, -0.5), 0, false, true), drive)
                                                .withTimeout(1),
                                new InstantCommand(() -> drive.stopSwerveDriveMotors()));
        }
}
