// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups;


import frc.robot.commands.LiftZeroPosition;
import frc.robot.commands.SwerveResetOdometry;
import frc.robot.commands.TiltZeroPosition;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.TiltSubsystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetRobotPositions extends ParallelCommandGroup {
  /** Creates a new CG_DrvBack. */
  public ResetRobotPositions(SwerveDrivetrain swerveDrivetrain,
  LiftSubsystem liftSubsystem,
  TiltSubsystem tiltSubsystem
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetSwerveDrive(swerveDrivetrain),
      new LiftZeroPosition(liftSubsystem),
      new TiltZeroPosition(tiltSubsystem),
      new SwerveResetOdometry(swerveDrivetrain, new Pose2d(0, 0, new Rotation2d(0) ))
    );

  }
}
