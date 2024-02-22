// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups;


import frc.robot.commands.SwerveResetDriveEncoders;
import frc.robot.commands.SwerveResetRotateEncoders;
import frc.robot.commands.SwerveZeroGyro;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetSwerveDrive extends ParallelCommandGroup {
  /** Creates a new CG_DrvBack. */
  public ResetSwerveDrive(SwerveDrivetrain swerveDrivetrain
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SwerveResetRotateEncoders(swerveDrivetrain),
      new SwerveResetDriveEncoders(swerveDrivetrain),
      new SwerveZeroGyro(swerveDrivetrain)
    );

  }
}
